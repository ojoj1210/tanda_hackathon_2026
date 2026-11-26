/*
  KTANE - Keypads module                       Board: Arduino Uno R3
  Link to the Raspberry Pi: USB serial, 115200 baud, ASCII lines.

  How it works
  ------------
  - Each key has a fixed symbol. Set KEY_SYMBOL[] below to match your caps.
  - At boot the sketch finds the one manual column that holds all four symbols
    and works out the press order from it.
  - The module is LOCKED until the Pi sends START at the beginning of a round.
  - Press the keys in order. A correct press lights that key's LED.
  - A wrong press is a STRIKE: the red LED flashes, progress is kept (like the game),
    and the Pi is told "STRIKE keypads". The Pi owns the ONE shared mistake counter
    (3 mistakes from any module = BOOM); this module never decides that itself.
  - After all four correct presses the green LED stays on and the Pi is told
    "SOLVED keypads". The Pi then counts it towards defusing the bomb.

  If no column holds all four symbols the red LED blinks fast forever, the module
  stays silent (the Pi will show "Keypads offline") and an INFO line says why.

  Serial protocol (same style as the ESP32 / Uno R4 modules)
    Pi -> Uno R3:  ID?                 -> ID keypads-uno-r3 v1
                   PING                -> PONG
                   START               -> ACTIVE      (new round: LEDs off, progress cleared, unlocked)
                   STOP                -> IDLE        (locked, all LEDs off)
                   STRIKES <n> <max>   (global mistakes; stored, no reply)
                   BOOM                (locked; LEDs stay as they are)
                   DEFUSED             (locked)
                   STATUS              -> STATE <IDLE|ACTIVE|SOLVED> <correct presses>
    Uno R3 -> Pi:  READY (boot)   STRIKE keypads   SOLVED keypads   INFO <text> (ignored)

  Note: opening the USB port resets an Uno R3 (normal for this board). The Pi
  copes with that: it simply sends START again if the board reboots mid-round.
*/

// ---------------------------------------------------------------------------
// Symbols (names for the 27 keypad glyphs in the manual)
// ---------------------------------------------------------------------------
enum Symbol : uint8_t {
  BALLOON,        // Ϙ  (balloon / lollipop)
  AT,             // Ѧ  (A with a tail, "at")
  LAMBDA,         // ƛ  (lambda with a bar)
  LIGHTNING,      // Ϟ  (squiggly lightning)
  SPIDER,         // Ѭ  (spider / squid-knife)
  HOOK_N,         // ϗ  (N with a hook)
  C_DOT_REVERSED, // Ͽ  (backwards C with a dot)
  EURO,           // Ӭ  (backwards E with two dots)
  CURSIVE,        // Ҩ  (cursive loop)
  HOLLOW_STAR,    // ☆
  QUESTION,       // ¿  (upside-down question mark)
  COPYRIGHT,      // ©
  PUMPKIN,        // Ѽ  (pumpkin / butt)
  DOUBLE_K,       // Җ  (two Ks back to back)
  MELTED_3,       // Ԇ  (melted 3)
  SIX,            // б
  PARAGRAPH,      // ¶
  BT,             // Ѣ  (b with a T)
  SMILEY,         // ټ  (smiley face)
  TRIDENT,        // Ψ  (pitchfork)
  C_DOT,          // Ͼ  (C with a dot)
  DRAGON,         // Ѯ  (3 with a tail and a hat)
  FILLED_STAR,    // ★
  TRACKS,         // ҂  (railway tracks)
  AE,             // æ
  N_HAT,          // Ҋ  (backwards N with a hat)
  OMEGA           // Ω
};

// The six columns from the bomb defusal manual, top to bottom.
const Symbol COLUMNS[6][7] = {
  { BALLOON,   AT,          LAMBDA,    LIGHTNING, SPIDER,   HOOK_N,  C_DOT_REVERSED },
  { EURO,      BALLOON,     C_DOT_REVERSED, CURSIVE, HOLLOW_STAR, HOOK_N, QUESTION  },
  { COPYRIGHT, PUMPKIN,     CURSIVE,   DOUBLE_K,  MELTED_3, LAMBDA,  HOLLOW_STAR    },
  { SIX,       PARAGRAPH,   BT,        SPIDER,    DOUBLE_K, QUESTION, SMILEY        },
  { TRIDENT,   SMILEY,      BT,        C_DOT,     PARAGRAPH, DRAGON, FILLED_STAR    },
  { SIX,       EURO,        TRACKS,    AE,        TRIDENT,  N_HAT,   OMEGA          }
};

// ===========================================================================
// >>> EDIT THIS: the symbol printed on each key cap <<<
//     KEY_SYMBOL[0] is the key wired to KEY_PIN[0] (D2), and so on.
// ===========================================================================
const Symbol KEY_SYMBOL[4] = { HOLLOW_STAR, COPYRIGHT, MELTED_3, PUMPKIN };

// ---------------------------------------------------------------------------
// Pins
// ---------------------------------------------------------------------------
const uint8_t KEY_PIN[4]     = { 2, 3, 4, 5 };   // key wires (other side to GND)
const uint8_t KEY_LED_PIN[4] = { 6, 7, 8, 9 };   // optional: one LED per key
const uint8_t LED_SOLVED     = 10;               // green status LED (module defused)
const uint8_t LED_STRIKE     = 11;               // red status LED (flashes on a mistake)

const unsigned long DEBOUNCE_MS   = 25;
const unsigned long STRIKE_LED_MS = 1000;        // how long red stays on per strike
const unsigned long SERIAL_BAUD   = 115200;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
uint8_t  solutionOrder[4];       // key indexes in the order they must be pressed
uint8_t  step = 0;               // how many correct presses so far
bool     keyDone[4] = { false, false, false, false };
bool     solved = false;
bool     active = false;         // true between the Pi's START and BOOM / DEFUSED / STOP
bool     configError = false;
uint8_t  globalStrikes = 0, globalMax = 3;   // told to us by the Pi (STRIKES n max)
unsigned long strikeLedOffAt = 0;

// Debounce state per key
bool          stableState[4];    // HIGH = released (pull-up), LOW = pressed
bool          lastReading[4];
unsigned long lastChange[4];

// ---------------------------------------------------------------------------
// Work out the press order from the key symbols
// ---------------------------------------------------------------------------
bool computeSolution() {
  for (uint8_t col = 0; col < 6; col++) {
    int8_t pos[4];
    bool allFound = true;

    for (uint8_t k = 0; k < 4 && allFound; k++) {
      pos[k] = -1;
      for (uint8_t row = 0; row < 7; row++) {
        if (COLUMNS[col][row] == KEY_SYMBOL[k]) { pos[k] = row; break; }
      }
      if (pos[k] < 0) allFound = false;
    }

    if (allFound) {
      // Sort key indexes by their row in this column (top of column first).
      for (uint8_t k = 0; k < 4; k++) solutionOrder[k] = k;
      for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3 - i; j++) {
          if (pos[solutionOrder[j]] > pos[solutionOrder[j + 1]]) {
            uint8_t t = solutionOrder[j];
            solutionOrder[j] = solutionOrder[j + 1];
            solutionOrder[j + 1] = t;
          }
        }
      }
      Serial.print(F("INFO column "));
      Serial.print(col + 1);
      Serial.print(F(", press order (keys 1-4): "));
      for (uint8_t i = 0; i < 4; i++) {
        Serial.print(solutionOrder[i] + 1);
        Serial.print(i < 3 ? F(",") : F(""));
      }
      Serial.println();
      return true;
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
// Returns true once each time key k goes from released to pressed
// ---------------------------------------------------------------------------
bool keyPressedEdge(uint8_t k) {
  bool reading = digitalRead(KEY_PIN[k]);
  unsigned long now = millis();

  if (reading != lastReading[k]) {
    lastReading[k] = reading;
    lastChange[k] = now;
  }

  if ((now - lastChange[k]) >= DEBOUNCE_MS && reading != stableState[k]) {
    stableState[k] = reading;
    if (stableState[k] == LOW) return true;   // just pressed
  }
  return false;
}

void allLedsOff() {
  for (uint8_t k = 0; k < 4; k++) digitalWrite(KEY_LED_PIN[k], LOW);
  digitalWrite(LED_SOLVED, LOW);
  digitalWrite(LED_STRIKE, LOW);
  strikeLedOffAt = 0;
}

// Fresh round: clear progress, LEDs off, unlock, forget stale key states.
void startRound() {
  step = 0;
  solved = false;
  for (uint8_t k = 0; k < 4; k++) {
    keyDone[k] = false;
    stableState[k] = lastReading[k] = digitalRead(KEY_PIN[k]);
    lastChange[k] = millis();
  }
  allLedsOff();
  globalStrikes = 0;
  active = true;
}

void strike(uint8_t k) {
  digitalWrite(LED_STRIKE, HIGH);
  strikeLedOffAt = millis() + STRIKE_LED_MS;
  Serial.println(F("STRIKE keypads"));              // the Pi counts it in the shared total
  Serial.print(F("INFO wrong key "));
  Serial.println(k + 1);
}

void handlePress(uint8_t k) {
  if (keyDone[k]) return;                  // already lit; pressing again does nothing

  if (k == solutionOrder[step]) {
    keyDone[k] = true;
    digitalWrite(KEY_LED_PIN[k], HIGH);
    step++;
    if (step == 4) {
      solved = true;
      digitalWrite(LED_SOLVED, HIGH);
      Serial.println(F("SOLVED keypads"));
    }
  } else {
    strike(k);
  }
}

// ---------------------------------------------------------------------------
// Serial commands from the Pi
// ---------------------------------------------------------------------------
char    rx[24];
uint8_t rxLen = 0;

void handle(char *line) {
  char *arg = strchr(line, ' ');
  if (arg) { *arg++ = 0; while (*arg == ' ') arg++; }

  if (!strcmp(line, "ID?")) {
    Serial.println(F("ID keypads-uno-r3 v1"));
  } else if (!strcmp(line, "PING")) {
    Serial.println(F("PONG"));
  } else if (!strcmp(line, "START")) {
    startRound();
    Serial.println(F("ACTIVE"));
  } else if (!strcmp(line, "STOP")) {
    active = false;
    step = 0;
    solved = false;
    for (uint8_t k = 0; k < 4; k++) keyDone[k] = false;
    allLedsOff();
    Serial.println(F("IDLE"));
  } else if (!strcmp(line, "STRIKES")) {          // global mistakes from the Pi (no reply)
    int n = 0, m = 0;
    if (arg && sscanf(arg, "%d %d", &n, &m) >= 1) {
      globalStrikes = (uint8_t)(n < 0 ? 0 : (n > 9 ? 9 : n));
      if (m > 0 && m <= 9) globalMax = (uint8_t)m;
    }
  } else if (!strcmp(line, "BOOM") || !strcmp(line, "DEFUSED")) {
    active = false;                               // locked; LEDs stay as they were
  } else if (!strcmp(line, "STATUS")) {
    Serial.print(F("STATE "));
    Serial.print(solved ? F("SOLVED") : (active ? F("ACTIVE") : F("IDLE")));
    Serial.print(' ');
    Serial.println(step);
  } else {
    Serial.print(F("ERR "));
    Serial.println(line);
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

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(SERIAL_BAUD);

  for (uint8_t k = 0; k < 4; k++) {
    pinMode(KEY_PIN[k], INPUT_PULLUP);
    pinMode(KEY_LED_PIN[k], OUTPUT);
    stableState[k] = lastReading[k] = digitalRead(KEY_PIN[k]);
    lastChange[k] = millis();
  }
  pinMode(LED_SOLVED, OUTPUT);
  pinMode(LED_STRIKE, OUTPUT);
  allLedsOff();

  if (!computeSolution()) {
    configError = true;                  // stay silent: the Pi shows "Keypads offline"
    Serial.println(F("INFO ERROR: no manual column contains all four KEY_SYMBOL values."));
    Serial.println(F("INFO Check KEY_SYMBOL[] - a real keypad always uses 4 symbols from one column."));
    return;
  }
  Serial.println(F("READY"));
}

void loop() {
  if (configError) {                       // fast red blink = bad symbol setup
    digitalWrite(LED_STRIKE, (millis() / 150) % 2);
    return;
  }

  pollSerial();

  // Turn the strike LED off after its flash
  if (strikeLedOffAt && (long)(millis() - strikeLedOffAt) >= 0) {
    digitalWrite(LED_STRIKE, LOW);
    strikeLedOffAt = 0;
  }

  if (!active || solved) return;           // locked until START; ignore keys once solved

  for (uint8_t k = 0; k < 4; k++) {
    if (keyPressedEdge(k)) handlePress(k);
  }
}
