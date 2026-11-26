// ktane_uno_r4.ino -- two KTaNE IRL modules on one Uno R4:
//   1) Helldivers-style NEEDY module (OLED + dial + slider)
//   2) PASSWORDS regular module      (1602A LCD + 5 buttons)
// Board: Arduino Uno R4 (Minima or WiFi)  |  Link to Pi 4: USB serial (115200)
//
// Libraries (Library Manager): "Adafruit SSD1306" + "Adafruit GFX Library"
//                              + "LiquidCrystal" (bundled with the IDE)
//
// Wiring:
//   OLED 0.96" SSD1306 : SDA -> A4 (SDA), SCL -> A5 (SCL), VCC -> 5V, GND -> GND
//   Rotary pot (dial)  : wiper -> A0, ends -> 5V / GND
//   Slide pot (slider) : wiper -> A1, ends -> 5V / GND
//   1602A LCD          : VSS,RW,K -> GND | VDD -> 5V | V0 -> GND (or pot wiper)
//                        RS -> D12, E -> D11, D4 -> D5, D5 -> D4, D6 -> D3, D7 -> D2
//                        A (backlight) -> 5V through 220 ohm, or leave unconnected
//   Buttons (to GND)   : LEFT D6 (blue), UP D7 (red), SUBMIT D8 (white),
//                        DOWN D9 (red), RIGHT D10 (blue)
//   A3                 : leave unconnected (random seed)
//
// Game (Helldivers 2 "Raise signal strength to 100%" terminal):
// The Pi decides WHEN (random intervals, see NEEDY_* in ktane_test.py) and sends
// START: the terminal then wakes up with a weak, noisy signal. The
// correct slider position and dial angle are HIDDEN. Move the vertical slider
// (left gauge) and turn the dial (right gauge) and watch SIGNAL STRENGTH % and
// the waveform: the closer both controls are to the hidden setting, the higher
// the percentage and the cleaner the wave. Reach 100% and hold it for HOLD_MS to
// lock the signal before the countdown ends. Locking reports "SOLVED needy" (the
// Pi just re-arms its timer: a needy module is never "defused"). Timeout reports
// "STRIKE needy", which counts towards the ONE global 3-mistake limit on the Pi.
//
// Robustness: the OLED bring-up never blocks or crashes the game. It frees a
// stuck I2C bus, probes 0x3C/0x3D, and keeps retrying in the background. With
// no display the game still runs headless over serial, and the onboard LED
// flickers fast. Every step is reported as "INFO ..." lines.
//
// Serial protocol (ASCII, one command per line) -- same as the Pico/ESP32 versions:
//   Pi -> Uno:  ID?            -> ID helldivers-needy unor4 v1
//               START [secs]   -> ACTIVE <secs>      (or BUSY if already running)
//               STOP           -> IDLE               (cancel, screen off)
//               STATUS         -> STATE <IDLE|ACTIVE|DONE|FAIL> <secs_left> <signal_pct>
//               RAW            -> RAW <dial> <slider>   (raw ADC counts, for calibration)
//               OLED?          -> OLED OK|MISSING
//               RECONNECT      -> retry the display now (INFO lines, then OLED ..)
//               PING           -> PONG
//               SCAN           -> list every I2C device that answers (OLED is usually 0x3C)
//               DIAG           -> toggle the once-a-second DBG status line on/off
//               MODULES?       -> MODULES needy passwords
//   Round control from the Pi (the Pi owns timer + the shared mistake counter):
//               STRIKES <n> <max> -> (no reply)   global mistakes so far; shown on the Passwords LCD
//               BOOM              -> needy off, Passwords locked ("** BOOM **")
//               DEFUSED           -> needy off, Passwords locked ("BOMB DEFUSED")
//               (STOP also locks Passwords; PW RESET unlocks it with a fresh puzzle)
//   Passwords (regular module; LOCKED until the Pi sends PW RESET at round start):
//               PW RESET       -> PW READY           (new puzzle, unlock, clears solved + strikes)
//               PW STATUS      -> PW STATE <PLAYING|SOLVED> <strikes> <letters on screen>
//               PW ANSWER      -> PW ANSWER <word>   (only while PW_DEBUG is true)
//   Uno -> Pi (unsolicited), same shape as the ESP32 ("<EVENT> <module>"):
//                             READY (boot)
//                             STRIKE needy / SOLVED needy          <- Helldivers needy module
//                             STRIKE passwords / SOLVED passwords  <- Passwords module
//                             INFO <text> (ignore these)

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <LiquidCrystal.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// Declared up here so the Arduino IDE's auto-generated function prototypes
// (inserted above the first function) can see it.
enum State : uint8_t { IDLE, ACTIVE, DONE, FAIL };
const char *STATE_NAMES[] = {"IDLE", "ACTIVE", "DONE", "FAIL"};

// Passwords module button type -- also up here for the same reason.
struct PwButton {
  uint8_t       pin;
  bool          stableState;   // HIGH = released (INPUT_PULLUP)
  bool          lastReading;
  unsigned long lastChange;
};
enum { PW_B_LEFT, PW_B_UP, PW_B_SUBMIT, PW_B_DOWN, PW_B_RIGHT, PW_NUM_BUTTONS };

// ---------------------------------------------------------------- config ----
const uint8_t  PIN_DIAL   = A0;
const uint8_t  PIN_SLIDER = A1;
const uint8_t  I2C_SDA_PIN    = A4;          // Wire on the Uno R4 (only used to free a stuck bus)
const uint8_t  I2C_SCL_PIN    = A5;
const uint8_t  OLED_ADDRS[] = {0x3C, 0x3D};
const uint32_t RETRY_MS   = 5000;        // how often to retry a missing display
const uint32_t PROBE_MS   = 1000;        // how often a working display is pinged

const bool  DIAL_INVERT   = false;       // flip if turning clockwise lowers the value
const bool  SLIDER_INVERT = false;       // flip if pushing "up" lowers the value
const float ADC_LO = 0.01f;              // fraction of ADC full scale at the physical ends
const float ADC_HI = 0.99f;              // (use RAW to check, then tune)

const float    TOL_SLIDER = 0.04f;       // +/- fraction of travel that counts as 100%
const float    TOL_DIAL   = 0.04f;
const float    FALLOFF    = 0.22f;       // bigger = signal rises earlier (easier to find)
const uint32_t HOLD_MS    = 600;         // how long 100% must be held to lock
const uint16_t DEFAULT_SECS = 45;
const float    MIN_TARGET_DIST = 0.25f;  // hidden setting is at least this far from the start handles
const uint32_t RESULT_MS  = 1600;        // how long SOLVED / FAILED screens stay up
const uint32_t FRAME_MS   = 50;          // ~20 fps

const bool     AUTO_DEMO  = false;       // true: self-start every DEMO_GAP_S (bench test). false = the Pi starts it.
const uint16_t DEMO_GAP_S = 8;

const bool     DEBUG_ON   = false;       // true: print a DBG status line (pots, state, OLED) once a second. DIAG toggles it.
const uint32_t DEBUG_MS   = 1000;        // ... this often
const bool     BUS_RECOVERY = false;     // true: clock out a stuck I2C bus before every OLED connect
                                         // (RECONNECT always does it). Leave off unless SDA sticks low.

// ADC: the R4 does 14-bit; on anything else we fall back to 10-bit.
#if defined(ARDUINO_ARCH_RENESAS) || defined(ARDUINO_ARCH_RENESAS_UNO)
const uint8_t ADC_BITS = 14;
#else
const uint8_t ADC_BITS = 10;
#endif
const uint32_t ADC_FULL = (1UL << ADC_BITS) - 1;

// ------------------------------------------------------------- serial out ----
// Only talk when a host has the port open, so a standalone (power-only) module
// can never stall on a full USB buffer.
#define SAY(...)  do { if (Serial) { char _b[192]; snprintf(_b, sizeof(_b), __VA_ARGS__); Serial.println(_b); } } while (0)

// ----------------------------------------------------------------- display ----
Adafruit_SSD1306 oled(128, 64, &Wire, -1, 400000UL, 400000UL);
bool     oledOk = false;
uint8_t  oledAddr = 0;
uint32_t oledLastTry = 0, oledLastProbe = 0;
bool     debugOn = DEBUG_ON;

void oledPower(bool on);

bool i2cProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// Un-stick an I2C bus (a slave holding SDA low after an interrupted transfer):
// clock SCL 9 times with SDA released, then send a STOP. Returns SDA/SCL idle levels.
void freeBus(uint8_t &sdaLevel, uint8_t &sclLevel) {
  Wire.end();
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  pinMode(I2C_SCL_PIN, INPUT_PULLUP);
  delayMicroseconds(20);
  for (uint8_t i = 0; i < 9; i++) {
    pinMode(I2C_SCL_PIN, OUTPUT); digitalWrite(I2C_SCL_PIN, LOW);  delayMicroseconds(10);
    pinMode(I2C_SCL_PIN, INPUT_PULLUP);                        delayMicroseconds(10);
  }
  pinMode(I2C_SDA_PIN, OUTPUT); digitalWrite(I2C_SDA_PIN, LOW);    delayMicroseconds(10);
  pinMode(I2C_SCL_PIN, INPUT_PULLUP);                          delayMicroseconds(10);
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);                          delayMicroseconds(20);
  sdaLevel = digitalRead(I2C_SDA_PIN);
  sclLevel = digitalRead(I2C_SCL_PIN);
  Wire.begin();
}

// List every device that ACKs on the bus. An SSD1306 shows up as 0x3C (or 0x3D).
void scanBus() {
  uint8_t n = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    if (i2cProbe(a)) { SAY("INFO scan: device at 0x%02x", a); n++; }
  }
  if (n) SAY("INFO scan: %u device(s) found", (unsigned)n);
  else   SAY("INFO scan: NO I2C devices answered (check power, SDA/SCL wiring, solder joints)");
}

// Find and initialise the display. Returns true on success. Never blocks long.
bool oledConnect(bool verbose, bool recover) {
  oledLastTry = millis();
  oledOk = false;
  if (recover) {
    uint8_t sda, scl;
    freeBus(sda, scl);
    if (verbose) SAY("INFO bus idle SDA=%d SCL=%d (both must be 1; 0 = shorted/wrong hole)", sda, scl);
  }
  for (uint8_t i = 0; i < sizeof(OLED_ADDRS); i++) {
    uint8_t a = OLED_ADDRS[i];
    bool seen = i2cProbe(a);
    if (verbose) SAY("INFO i2c probe 0x%02x: %s", a, seen ? "ACK" : "no answer");
    if (!seen) continue;
    if (oled.begin(SSD1306_SWITCHCAPVCC, a, true, true)) {
      oledAddr = a;
      oledOk = true;
      oled.clearDisplay();
      oled.display();
      oledLastProbe = millis();
      return true;
    }
    if (verbose) SAY("INFO 0x%02x answered but oled.begin() failed", a);
  }
  if (verbose) scanBus();
  return false;
}

// Short "I am alive" screen, so you can tell the display works without starting a game.
void oledSplash() {
  if (!oledOk) return;
  oledPower(true);
  oled.clearDisplay();
  oled.drawRect(0, 0, 128, 64, SSD1306_WHITE);
  oled.setTextSize(2);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(22, 10);
  oled.print("OLED OK");
  oled.setTextSize(1);
  oled.setCursor(34, 40);
  oled.print("addr 0x");
  oled.print(oledAddr, HEX);
  oled.display();
}

void oledPower(bool on) {
  if (oledOk) oled.ssd1306_command(on ? SSD1306_DISPLAYON : SSD1306_DISPLAYOFF);
}

void oledReport(bool ok) {
  if (ok) SAY("INFO oled OK addr 0x%02x", oledAddr);
  else    SAY("INFO oled MISSING: running headless. Check VCC/GND, SDA=A4, SCL=A5; retry every %lu s", (unsigned long)(RETRY_MS / 1000));
}

// ------------------------------------------------------------------ pots ----
struct Pot {
  uint8_t pin;
  bool    invert;
  float   v;
  bool    primed;

  uint16_t raw() {                       // 16-sample average of the ADC counts
    uint32_t t = 0;
    for (uint8_t i = 0; i < 16; i++) t += analogRead(pin);
    return (uint16_t)(t >> 4);
  }
  float read() {                         // smoothed 0..1
    float lo = ADC_LO * ADC_FULL, hi = ADC_HI * ADC_FULL;
    float x = ((float)raw() - lo) / (hi - lo);
    x = x < 0 ? 0 : (x > 1 ? 1 : x);
    if (invert) x = 1.0f - x;
    if (!primed) { v = x; primed = true; }
    else v += 0.35f * (x - v);
    return v;
  }
};

Pot dial   = {PIN_DIAL,   DIAL_INVERT,   0.5f, false};
Pot slider = {PIN_SLIDER, SLIDER_INVERT, 0.5f, false};

// ------------------------------------------------------------------ game ----

State    state = IDLE;
uint16_t secsTotal = DEFAULT_SECS;
uint32_t tStart = 0, tEnd = 0, resultUntil = 0, lastDraw = 0, nextDemo = 0;
bool     holding = false;
uint32_t holdT0 = 0;
float    tgtS = 0.5f, tgtD = 0.5f;       // hidden correct settings
float    sig = 0.0f;                     // signal strength 0..1
uint8_t  pct = 0;

// static geometry for the OLED
const int DIAL_CX = 106, DIAL_CY = 39, DIAL_R = 18;
const int SLIDER_X = 12, SLIDER_TOP = 17, SLIDER_BOT = 60;
const int WAVE_X0 = 24, WAVE_X1 = 86, WAVE_Y = 52;
const uint8_t ARC_N = 46;                // -135..135 deg in 6 deg steps
int8_t arcX[ARC_N], arcY[ARC_N];

void dialXY(float v, int r, int &x, int &y) {
  float a = (-135.0f + 270.0f * v) * (float)M_PI / 180.0f;
  x = DIAL_CX + (int)lroundf(r * sinf(a));
  y = DIAL_CY - (int)lroundf(r * cosf(a));
}

float rnd() { return random(0, 10000) / 10000.0f; }

float pickTarget(float cur) {
  for (uint8_t i = 0; i < 20; i++) {
    float t = 0.08f + rnd() * 0.84f;
    if (fabsf(t - cur) >= MIN_TARGET_DIST) return t;
  }
  return cur < 0.5f ? 0.9f : 0.1f;
}

// 1.0 inside the tolerance, then a smooth fall-off that never reaches 0, so there
// is always some signal to climb toward, however far off you are.
float closeness(float e, float tol) {
  if (e <= tol) return 1.0f;
  float x = (e - tol) / FALLOFF;
  return 1.0f / (1.0f + x * x);
}

int32_t msLeft(uint32_t now) { return (int32_t)(tEnd - now); }
uint16_t secsLeft(uint32_t now) {
  int32_t m = msLeft(now);
  return m <= 0 ? 0 : (uint16_t)((m + 999) / 1000);
}

void startGame(uint32_t now, uint16_t secs) {
  tgtS = pickTarget(slider.v);
  tgtD = pickTarget(dial.v);
  secsTotal = secs;
  tStart = now;
  tEnd = now + (uint32_t)secs * 1000UL;
  holding = false;
  pct = 0;
  sig = 0;
  state = ACTIVE;
  oledPower(true);
  lastDraw = 0;
  SAY("ACTIVE %u", secs);
}

void setIdle() {
  state = IDLE;
  holding = false;
  pct = 0;
  if (oledOk) {
    oled.clearDisplay();
    oled.display();
    oledPower(false);
  }
}

void finish(uint32_t now, State s, const char *ev) {
  state = s;
  resultUntil = now + RESULT_MS;
  lastDraw = 0;
  SAY("%s", ev);
}

// ---------------------------------------------------------------- drawing ----
void drawWave(float clarity, float phase) {
  int noise = (int)(5 * (1.0f - clarity));
  int px = -1, py = 0;
  for (int x = WAVE_X0; x < WAVE_X1; x += 2) {
    int y = WAVE_Y + (int)(6 * sinf(phase + x * 0.35f));
    if (noise) y += (int)random(-noise, noise + 1);
    y = y < 43 ? 43 : (y > 63 ? 63 : y);
    if (px >= 0) oled.drawLine(px, py, x, y, SSD1306_WHITE);
    px = x; py = y;
  }
}

void text(const char *s, int x, int y, uint8_t size) {
  oled.setTextSize(size);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(x, y);
  oled.print(s);
}

void draw(uint32_t now, float d, float s) {
  char b[16];
  oled.clearDisplay();
  if (state == DONE) {
    text("100%", 28, 8, 3);
    text("SIGNAL LOCKED", 25, 38, 1);
    drawWave(1.0f, now / 120.0f);
  } else if (state == FAIL) {
    text("SIGNAL", 28, 12, 2);
    text("LOST", 40, 36, 2);
  } else {
    text(holding ? "LOCKING" : "SIGNAL", 0, 0, 1);
    snprintf(b, sizeof(b), "%3ds", secsLeft(now));
    text(b, 96, 0, 1);
    int32_t rem = msLeft(now);
    if (rem < 0) rem = 0;
    oled.drawRect(0, 10, 128, 4, SSD1306_WHITE);
    oled.fillRect(0, 10, (int)(128L * rem / ((int32_t)secsTotal * 1000L)), 4, SSD1306_WHITE);

    // vertical slider gauge (left): track, scale ticks, handle. Target is hidden.
    oled.drawFastVLine(SLIDER_X, SLIDER_TOP, SLIDER_BOT - SLIDER_TOP, SSD1306_WHITE);
    for (uint8_t t = 0; t <= 4; t += 2) {
      int ty = SLIDER_BOT - t * (SLIDER_BOT - SLIDER_TOP) / 4;
      oled.drawFastHLine(SLIDER_X - 5, ty, 11, SSD1306_WHITE);
    }
    int hy = SLIDER_BOT - (int)(s * (SLIDER_BOT - SLIDER_TOP));
    oled.fillRect(SLIDER_X - 6, hy - 1, 13, 3, SSD1306_WHITE);

    // dial gauge (right): 270 degree arc + needle. Target is hidden.
    for (uint8_t i = 0; i < ARC_N; i++) oled.drawPixel(arcX[i], arcY[i], SSD1306_WHITE);
    int nx, ny;
    dialXY(d, DIAL_R - 2, nx, ny);
    oled.drawLine(DIAL_CX, DIAL_CY, nx, ny, SSD1306_WHITE);
    oled.fillRect(DIAL_CX - 1, DIAL_CY - 1, 3, 3, SSD1306_WHITE);

    // signal strength readout + hold bar + waveform
    snprintf(b, sizeof(b), "%u", pct);
    text(b, 24 + 12 * (3 - (int)strlen(b)), 16, 2);
    text("%", 64, 24, 1);
    oled.drawRect(24, 34, 54, 5, SSD1306_WHITE);
    if (holding) {
      uint32_t held = now - holdT0;
      if (held > HOLD_MS) held = HOLD_MS;
      oled.fillRect(24, 34, (int)(54UL * held / HOLD_MS), 5, SSD1306_WHITE);
    }
    drawWave(sig, now / 120.0f);
  }
  oled.display();
}

// ============================================================== PASSWORDS ====
// Regular module (KTaNE "Passwords"): 1602A LCD in 4-bit mode + 5 buttons.
// It runs alongside the Helldivers needy module and never blocks.
//
// Puzzle: 5 letter columns, 6 letters each. Exactly ONE word from the manual's
// list can be spelled. Blue = move column, Red = change letter, White = submit.

// ---- passwords config ----
const uint8_t PW_LCD_RS = 12;
const uint8_t PW_LCD_EN = 11;
const uint8_t PW_LCD_D4 = 5;
const uint8_t PW_LCD_D5 = 4;
const uint8_t PW_LCD_D6 = 3;
const uint8_t PW_LCD_D7 = 2;

const uint8_t PW_PIN_LEFT   = 6;   // blue
const uint8_t PW_PIN_UP     = 7;   // red
const uint8_t PW_PIN_SUBMIT = 8;   // white
const uint8_t PW_PIN_DOWN   = 9;   // red
const uint8_t PW_PIN_RIGHT  = 10;  // blue

const bool          PW_DEBUG       = false;  // true: print the answer as an INFO line + allow PW ANSWER (bench testing)
const bool          PW_LIVE_AT_BOOT = false; // true: buttons work before the Pi sends PW RESET (bench testing without the Pi)
const unsigned long PW_DEBOUNCE_MS = 25;
const unsigned long PW_MESSAGE_MS  = 1500;   // how long "** STRIKE **" stays up

const char* const PW_WORDS[] = {
  "ABOUT", "AFTER", "AGAIN", "BELOW", "COULD",
  "EVERY", "FIRST", "FOUND", "GREAT", "HOUSE",
  "LARGE", "LEARN", "NEVER", "OTHER", "PLACE",
  "PLANT", "POINT", "RIGHT", "SMALL", "SOUND",
  "SPELL", "STILL", "STUDY", "THEIR", "THERE",
  "THESE", "THING", "THINK", "THREE", "WATER",
  "WHERE", "WHICH", "WORLD", "WOULD", "WRITE"
};
const uint8_t PW_NUM_WORDS = sizeof(PW_WORDS) / sizeof(PW_WORDS[0]);
const uint8_t PW_COLS    = 5;   // letters in a word
const uint8_t PW_LETTERS = 6;   // letters per column

// ---- passwords state ----
LiquidCrystal lcd(PW_LCD_RS, PW_LCD_EN, PW_LCD_D4, PW_LCD_D5, PW_LCD_D6, PW_LCD_D7);

char          pwColumn[PW_COLS][PW_LETTERS];
uint8_t       pwPos[PW_COLS];          // which letter each column is showing
uint8_t       pwCursor   = 0;
uint8_t       pwAnswer   = 0;          // index into PW_WORDS
bool          pwSolved   = false;
uint8_t       pwStrikes  = 0;          // strikes caused by this module this round
bool          pwDirty    = true;
bool          pwMsgOn    = false;
unsigned long pwMsgStart = 0;
const char*   pwMsg      = "";
bool          pwLocked   = !PW_LIVE_AT_BOOT;   // inactive until the Pi starts a round (and again after BOOM / DEFUSED / STOP)
const char*   pwLockMsg  = "  Waiting START";
uint8_t       gStrikes   = 0;          // GLOBAL mistakes, told to us by the Pi (STRIKES n max)
uint8_t       gMax       = 3;

PwButton pwButtons[PW_NUM_BUTTONS] = {
  { PW_PIN_LEFT,   HIGH, HIGH, 0 },
  { PW_PIN_UP,     HIGH, HIGH, 0 },
  { PW_PIN_SUBMIT, HIGH, HIGH, 0 },
  { PW_PIN_DOWN,   HIGH, HIGH, 0 },
  { PW_PIN_RIGHT,  HIGH, HIGH, 0 },
};

// Returns true once, at the moment a button goes down (INPUT_PULLUP: LOW = pressed).
bool pwButtonPressed(PwButton &b) {
  bool reading = digitalRead(b.pin);
  if (reading != b.lastReading) {
    b.lastReading = reading;
    b.lastChange  = millis();
  }
  if ((millis() - b.lastChange) > PW_DEBOUNCE_MS && reading != b.stableState) {
    b.stableState = reading;
    if (reading == LOW) return true;
  }
  return false;
}

// ---- puzzle generation ----
bool pwColumnHas(uint8_t col, char ch) {
  for (uint8_t i = 0; i < PW_LETTERS; i++) if (pwColumn[col][i] == ch) return true;
  return false;
}

uint8_t pwCountSpellable() {
  uint8_t count = 0;
  for (uint8_t w = 0; w < PW_NUM_WORDS; w++) {
    bool ok = true;
    for (uint8_t c = 0; c < PW_COLS; c++) {
      if (!pwColumnHas(c, PW_WORDS[w][c])) { ok = false; break; }
    }
    if (ok) count++;
  }
  return count;
}

void pwFillColumn(uint8_t col, char mustHave) {
  pwColumn[col][0] = mustHave;
  uint8_t n = 1;
  while (n < PW_LETTERS) {
    char ch = 'A' + random(26);
    bool dup = false;
    for (uint8_t i = 0; i < n; i++) if (pwColumn[col][i] == ch) { dup = true; break; }
    if (!dup) pwColumn[col][n++] = ch;
  }
  for (int8_t i = PW_LETTERS - 1; i > 0; i--) {   // Fisher-Yates shuffle
    uint8_t j = random(i + 1);
    char t = pwColumn[col][i]; pwColumn[col][i] = pwColumn[col][j]; pwColumn[col][j] = t;
  }
}

bool pwShowingAnswer() {
  for (uint8_t c = 0; c < PW_COLS; c++) {
    if (pwColumn[c][pwPos[c]] != PW_WORDS[pwAnswer][c]) return false;
  }
  return true;
}

// Current letters on screen, e.g. "PLACE" (buf needs 6 bytes).
void pwCurrentWord(char *buf) {
  for (uint8_t c = 0; c < PW_COLS; c++) buf[c] = pwColumn[c][pwPos[c]];
  buf[PW_COLS] = 0;
}

void pwNewPuzzle() {
  do {                                            // re-roll until exactly one word fits
    pwAnswer = random(PW_NUM_WORDS);
    for (uint8_t c = 0; c < PW_COLS; c++) pwFillColumn(c, PW_WORDS[pwAnswer][c]);
  } while (pwCountSpellable() != 1);
  do {                                            // never start on the answer
    for (uint8_t c = 0; c < PW_COLS; c++) pwPos[c] = random(PW_LETTERS);
  } while (pwShowingAnswer());

  pwCursor  = 0;
  pwSolved  = false;
  pwStrikes = 0;
  pwMsgOn   = false;
  pwDirty   = true;

  if (PW_DEBUG) {
    SAY("INFO pw answer %s | cols %.6s %.6s %.6s %.6s %.6s", PW_WORDS[pwAnswer],
        pwColumn[0], pwColumn[1], pwColumn[2], pwColumn[3], pwColumn[4]);
  }
}

// ---- LCD ----
// Row 0: " P  L [A] C  E "   (brackets = selected column)
// Row 1: status line
void pwPrintRow(uint8_t row, const char *s) {
  char buf[17];
  uint8_t i = 0;
  for (; i < 16 && s[i]; i++) buf[i] = s[i];
  for (; i < 16; i++) buf[i] = ' ';
  buf[16] = 0;
  lcd.setCursor(0, row);
  lcd.print(buf);
}

void pwDraw() {
  char row0[17];
  memset(row0, ' ', 16);
  row0[16] = 0;
  for (uint8_t c = 0; c < PW_COLS; c++) {
    uint8_t x = 1 + 3 * c;                        // x = 1, 4, 7, 10, 13
    row0[x] = pwColumn[c][pwPos[c]];
    if (!pwLocked && !pwSolved && c == pwCursor) { row0[x - 1] = '['; row0[x + 1] = ']'; }
  }
  pwPrintRow(0, row0);

  if (pwLocked) {
    pwPrintRow(1, pwLockMsg);
  } else if (pwSolved) {
    pwPrintRow(1, "    SOLVED!");
  } else if (pwMsgOn) {
    pwPrintRow(1, pwMsg);
  } else {
    char row1[17];                                // the bomb's shared mistake counter, not just ours
    snprintf(row1, sizeof(row1), "Strikes: %u/%u", (unsigned)gStrikes, (unsigned)gMax);
    pwPrintRow(1, row1);
  }
}

void pwLock(const char *msg) {
  pwLocked  = true;
  pwLockMsg = msg;
  pwMsgOn   = false;
  pwDirty   = true;
}

void pwShowMessage(const char *m) {
  pwMsg = m;
  pwMsgOn = true;
  pwMsgStart = millis();
  pwDirty = true;
}

// ---- module setup / loop ----
void passwordsSetup() {
  for (uint8_t i = 0; i < PW_NUM_BUTTONS; i++) pinMode(pwButtons[i].pin, INPUT_PULLUP);
  lcd.begin(16, 2);
  lcd.clear();
  pwNewPuzzle();
  pwDraw();
  pwDirty = false;
}

void passwordsLoop() {
  bool pressed[PW_NUM_BUTTONS];                   // read every button every pass
  for (uint8_t i = 0; i < PW_NUM_BUTTONS; i++) pressed[i] = pwButtonPressed(pwButtons[i]);

  if (pwMsgOn && (millis() - pwMsgStart) >= PW_MESSAGE_MS) {
    pwMsgOn = false;
    pwDirty = true;
  }

  if (!pwSolved && !pwLocked) {
    if (pressed[PW_B_LEFT])  { pwCursor = (pwCursor + PW_COLS - 1) % PW_COLS; pwDirty = true; }
    if (pressed[PW_B_RIGHT]) { pwCursor = (pwCursor + 1) % PW_COLS;           pwDirty = true; }
    if (pressed[PW_B_UP])    { pwPos[pwCursor] = (pwPos[pwCursor] + PW_LETTERS - 1) % PW_LETTERS; pwDirty = true; }
    if (pressed[PW_B_DOWN])  { pwPos[pwCursor] = (pwPos[pwCursor] + 1) % PW_LETTERS;              pwDirty = true; }
    if (pressed[PW_B_SUBMIT]) {
      if (pwShowingAnswer()) {
        pwSolved = true;
        SAY("SOLVED passwords");
      } else {
        pwStrikes++;
        pwShowMessage("  ** STRIKE **");
        SAY("STRIKE passwords");
      }
      pwDirty = true;
    }
  }

  if (pwDirty) {
    pwDraw();
    pwDirty = false;
  }
}

// Serial commands for this module: "PW <sub>"
void pwHandle(const char *sub) {
  char w[PW_COLS + 1];
  if (!sub || !*sub || !strcmp(sub, "STATUS")) {
    pwCurrentWord(w);
    SAY("PW STATE %s %u %s", pwSolved ? "SOLVED" : "PLAYING", (unsigned)pwStrikes, w);
  } else if (!strcmp(sub, "RESET")) {
    randomSeed(random(0x7FFFFFFF) ^ micros());    // stir in timing so rounds differ
    pwNewPuzzle();
    pwLocked = false;
    gStrikes = 0;                                 // the Pi follows up with STRIKES <n> <max>
    SAY("PW READY");
  } else if (!strcmp(sub, "ANSWER") && PW_DEBUG) {
    SAY("PW ANSWER %s", PW_WORDS[pwAnswer]);
  } else {
    SAY("ERR PW %s", sub);
  }
}

// ---------------------------------------------------------------- serial ----
char    rx[24];
uint8_t rxLen = 0;

void handle(char *line) {
  char *arg = strchr(line, ' ');
  if (arg) { *arg++ = 0; while (*arg == ' ') arg++; }
  uint32_t now = millis();
  if (!strcmp(line, "ID?")) {
    SAY("ID helldivers-needy unor4 v1");
  } else if (!strcmp(line, "PING")) {
    SAY("PONG");
  } else if (!strcmp(line, "MODULES?")) {
    SAY("MODULES needy passwords");
  } else if (!strcmp(line, "PW")) {
    pwHandle(arg);
  } else if (!strcmp(line, "START")) {
    if (state == ACTIVE) {
      SAY("BUSY");
    } else {
      long s = (arg && *arg) ? atol(arg) : DEFAULT_SECS;
      if (s <= 0) s = DEFAULT_SECS;
      if (s < 5) s = 5;
      if (s > 600) s = 600;
      startGame(now, (uint16_t)s);
    }
  } else if (!strcmp(line, "STOP")) {
    setIdle();
    pwLock("  Waiting START");
    SAY("IDLE");
  } else if (!strcmp(line, "STRIKES")) {          // global mistake counter from the Pi
    int n = 0, m = 0;
    if (arg && sscanf(arg, "%d %d", &n, &m) >= 1) {
      gStrikes = (uint8_t)(n < 0 ? 0 : (n > 9 ? 9 : n));
      if (m > 0 && m <= 9) gMax = (uint8_t)m;
      pwDirty = true;
    }
  } else if (!strcmp(line, "BOOM")) {
    setIdle();                                    // needy off
    pwLock("  ** BOOM **");
  } else if (!strcmp(line, "DEFUSED")) {
    setIdle();
    pwLock("BOMB DEFUSED");
  } else if (!strcmp(line, "STATUS")) {
    SAY("STATE %s %u %u", STATE_NAMES[state], state == ACTIVE ? secsLeft(now) : 0, pct);
  } else if (!strcmp(line, "RAW")) {
    SAY("RAW %u %u", dial.raw(), slider.raw());
  } else if (!strcmp(line, "OLED?")) {
    SAY(oledOk ? "OLED OK" : "OLED MISSING");
  } else if (!strcmp(line, "SCAN")) {
    scanBus();
  } else if (!strcmp(line, "DIAG")) {
    debugOn = !debugOn;
    SAY("INFO debug stream %s", debugOn ? "ON" : "OFF");
  } else if (!strcmp(line, "RECONNECT")) {
    bool ok = oledConnect(true, true);
    oledReport(ok);
    if (ok) { oledSplash(); delay(1500); }
    oledPower(state != IDLE);
    lastDraw = 0;
    SAY(ok ? "OLED OK" : "OLED MISSING");
  } else {
    SAY("ERR %s", line);
  }
}

void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (rxLen) {
        rx[rxLen] = 0;
        rxLen = 0;
        handle(rx);
      }
    } else if (rxLen < sizeof(rx) - 1) {
      rx[rxLen++] = (c >= 'a' && c <= 'z') ? c - 32 : c;
    } else {
      rxLen = 0;                         // overlong line: drop it
    }
  }
}

// ------------------------------------------------------------------ loop ----
void blinkLed(uint32_t now) {
  uint32_t period = !oledOk ? 100 : (state == ACTIVE ? 250 : 1000);
  digitalWrite(LED_BUILTIN, (now / period) & 1);
}

void step(uint32_t now) {
  float d = dial.read(), s = slider.read();

  if (!oledOk && (now - oledLastTry) >= RETRY_MS) {
    if (oledConnect(debugOn, BUS_RECOVERY)) {
      SAY("INFO oled OK addr 0x%02x", oledAddr);
      oledPower(state != IDLE);
      lastDraw = 0;
    }
  } else if (oledOk && (now - oledLastProbe) >= PROBE_MS) {
    oledLastProbe = now;
    if (!i2cProbe(oledAddr)) {           // display vanished mid-run
      SAY("INFO oled lost");
      oledOk = false;
      oledLastTry = now;
    }
  }

  static uint32_t lastDbg = 0;
  if (debugOn && (now - lastDbg) >= DEBUG_MS) {
    lastDbg = now;
    SAY("DBG dial raw=%u (%d%%) | slider raw=%u (%d%%) | state=%s left=%us signal=%u%% | oled=%s",
        (unsigned)dial.raw(), (int)(d * 100 + 0.5f),
        (unsigned)slider.raw(), (int)(s * 100 + 0.5f),
        STATE_NAMES[state], (unsigned)(state == ACTIVE ? secsLeft(now) : 0), (unsigned)pct,
        oledOk ? "OK" : "MISSING");
  }

  if (state == ACTIVE) {
    float ed = fabsf(d - tgtD), es = fabsf(s - tgtS);
    bool locked = ed <= TOL_DIAL && es <= TOL_SLIDER;
    // geometric mean: always some signal to climb toward, but 100% needs BOTH right
    sig = sqrtf(closeness(ed, TOL_DIAL) * closeness(es, TOL_SLIDER));
    int p = (int)(100 * sig);
    pct = locked ? 100 : (p > 99 ? 99 : p);
    if (locked) {
      if (!holding) { holding = true; holdT0 = now; }
      if (now - holdT0 >= HOLD_MS) finish(now, DONE, "SOLVED needy");
    } else {
      holding = false;
    }
    if (state == ACTIVE && msLeft(now) <= 0) finish(now, FAIL, "STRIKE needy");
  } else if (state == DONE || state == FAIL) {
    if ((int32_t)(now - resultUntil) >= 0) setIdle();
  } else if (AUTO_DEMO && (int32_t)(now - nextDemo) >= 0) {
    nextDemo = now + (uint32_t)DEMO_GAP_S * 1000UL;
    startGame(now, DEFAULT_SECS);
  }

  if (state != IDLE && oledOk && (now - lastDraw) >= FRAME_MS) {
    lastDraw = now;
    draw(now, d, s);
  }
  blinkLed(now);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}   // give a host a moment, never wait forever

#if defined(ARDUINO_ARCH_RENESAS) || defined(ARDUINO_ARCH_RENESAS_UNO)
  analogReadResolution(ADC_BITS);
#endif
  // A3 left floating on purpose: mix several noisy reads with timing jitter.
  uint32_t seed = 0;
  for (uint8_t i = 0; i < 32; i++) seed = (seed << 1) ^ analogRead(A3) ^ micros();
  randomSeed(seed);

  passwordsSetup();                       // LCD + buttons; prints "INFO pw answer ..." in debug

  for (uint8_t i = 0; i < ARC_N; i++) {
    float a = (-135.0f + 6.0f * i) * (float)M_PI / 180.0f;
    arcX[i] = DIAL_CX + (int)lroundf(DIAL_R * sinf(a));
    arcY[i] = DIAL_CY - (int)lroundf(DIAL_R * cosf(a));
  }
  for (uint8_t i = 0; i < 12; i++) { dial.read(); slider.read(); }   // prime the smoothing

  SAY("INFO ---- boot: ADC %u-bit, full scale %lu ----", (unsigned)ADC_BITS, (unsigned long)ADC_FULL);
  SAY("INFO pots at boot: dial raw=%u slider raw=%u (move them: values should sweep 0..%lu)",
      (unsigned)dial.raw(), (unsigned)slider.raw(), (unsigned long)ADC_FULL);
  Wire.begin();
  SAY("INFO scanning I2C bus...");
  scanBus();
  bool ok = oledConnect(true, BUS_RECOVERY);
  oledReport(ok);
  if (ok) { oledSplash(); delay(2000); }
  oledPower(false);
  nextDemo = millis() + (uint32_t)DEMO_GAP_S * 1000UL;
  SAY("READY");
}

void loop() {
  pollSerial();
  step(millis());
  passwordsLoop();
  delay(10);
}