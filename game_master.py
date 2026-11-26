#!/usr/bin/env python3
"""
KTANE bomb master - Raspberry Pi 4.

The Pi is the game logic for the WHOLE bomb: timer, the single shared mistake
counter (3 mistakes in total, from any module, = BOOM), win/lose and the start button.
Modules (the ESP32 with Wires + Maze, later the Uno etc.) are plugged in over USB serial
and only report events; they never decide anything.

Pins (BCM):
  START button      GPIO17   (other leg to GND, internal pull-up)
  Passive buzzer    GPIO26
  Strike LEDs       GPIO6, GPIO5 (via resistors)
  TM1637 timer      DIO=GPIO20, CLK=GPIO21
  LCD1602 + I2C     SCL=GPIO22, SDA=GPIO27  (software I2C, bus 3 - see setup below)
  ESP32             USB cable (appears as /dev/ttyUSB0 or /dev/ttyACM0, found automatically)

Behaviour:
  - Boot self-test: LEDs, buzzer, TM1637 "88:88", LCD message.
  - Idle: the LCD says why the bomb can't start yet ("ESP32 offline", "Connect wires")
    or "Press START".
  - START -> 5:00 countdown, serial number + batteries on the LCD, ESP32 gets START.
  - Every STRIKE the ESP32 reports adds one to the shared counter (LED + sound + LCD).
    The 3rd strike, or the timer reaching 00:00, = BOOM.
  - When every module in REQUIRED_MODULES has reported SOLVED -> bomb DEFUSED.
  - Press START to play again.

Setup (once):
  pip install RPLCD smbus2 gpiozero raspberrypi-tm1637 pyserial
  sudo usermod -aG dialout $USER      (then log out/in, so you may open /dev/ttyUSB0)
  GPIO22/27 are not the Pi's hardware I2C pins, so add this line to
  /boot/firmware/config.txt (or /boot/config.txt on older OS), then reboot:
      dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=27,i2c_gpio_scl=22
  Check the LCD shows up:  i2cdetect -y 3     (expect 0x27 or 0x3F)

Run:  python3 ktane_test.py            (add -v to also print the ESP32's INFO lines)

Serial protocol with the ESP32 (115200 baud, ASCII lines):
  Pi -> ESP32   ID?  PING  START  STRIKES <n> <max>  BOOM  DEFUSED  STOP
  ESP32 -> Pi   READY  ID <name> <ver>  PONG  WIRES <0|1>  LIVE
                STRIKE <module>  SOLVED <module>  INFO <text> (ignored)
"""
import glob
import os
import queue
import random
import string
import sys
import threading
import time

# ----------------------------------------------------------------------------- pins (BCM)
PIN_BUTTON = 17
PIN_BUZZER = 26
PIN_LED1 = 6
PIN_LED2 = 5
PIN_TM_DIO = 20
PIN_TM_CLK = 21

# LCD on software I2C bus created by the dtoverlay above
I2C_BUS = 3
LCD_ADDRESS = None          # None = auto-detect (0x27 / 0x3F)

# ----------------------------------------------------------------------------- USB modules
ESP32_PORT = None           # None = auto-detect, or e.g. "/dev/ttyUSB0"
ESP32_ID_PREFIX = "esp32"   # the ESP32 answers "ID esp32-maze-wires v1"
SERIAL_BAUD = 115200

# Modules that must report SOLVED to defuse the bomb
REQUIRED_MODULES = {"wires", "maze"}

# True: START is refused until the ESP32 is connected and all 5 wires are plugged in
REQUIRE_MODULES_READY = True
# True: a press of the START button during a game counts as a mistake (bench testing only)
BUTTON_SIMULATES_STRIKE = False

# ----------------------------------------------------------------------------- settings
GAME_SECONDS = 300
MAX_STRIKES = 3             # LED1 = strike 1, LED2 = strike 2, 3rd = boom (shared by ALL modules)
TIMER_RATE = [1.0, 1.25, 1.5]   # timer speed-up per strike (1.0 = off)
END_SCREEN_SECONDS = 4      # how long the BOOM / DEFUSED text stays before the idle text returns

# ----------------------------------------------------------------------------- sounds (freq Hz, seconds)
STRIKE_SOUND = [(180, 0.35), (0, 0.05), (140, 0.35)]
EXPLODE_SOUND = [(f, 0.04) for f in range(900, 80, -40)] + [(70, 1.2)]
BOOT_SOUND = [(523, 0.1), (659, 0.1), (784, 0.1)]
START_SOUND = [(880, 0.08), (0, 0.04), (1320, 0.15)]
SOLVED_SOUND = [(1047, 0.08), (0, 0.03), (1568, 0.15)]
DEFUSED_SOUND = [(523, 0.1), (659, 0.1), (784, 0.1), (1047, 0.1), (0, 0.05), (1047, 0.3)]
ERROR_SOUND = [(200, 0.12), (0, 0.05), (200, 0.12)]

VERBOSE = "-v" in sys.argv


def gen_serial(rng):
    letters = [c for c in string.ascii_uppercase if c not in "OY"]
    alnum = letters + list(string.digits)
    return (rng.choice(alnum) + rng.choice(alnum) + rng.choice(string.digits)
            + rng.choice(letters) + rng.choice(letters) + rng.choice(string.digits))


# ============================================================================= USB module link
class ModuleLink:
    """USB-serial link to one module controller (the ESP32).

    A background thread finds the port (by asking every candidate "ID?"), reads lines,
    reconnects if the cable is pulled, and turns messages into events on `events`:
        ("strike", module)   ("solved", module)
        ("link_up", name)    ("link_down", name)    ("reboot", name)
    State you can poll: .connected, .wires_ready
    """
    claimed_ports = set()       # so a second link (Arduino) never grabs the same port

    def __init__(self, name_prefix, events, port=None, baud=SERIAL_BAUD):
        import serial           # imported here so the script can still be read/tested without it
        self._serial = serial
        self.prefix = name_prefix
        self.events = events
        self.forced_port = port
        self.baud = baud
        self.ser = None
        self.port = None
        self.device_id = ""
        self.connected = False
        self.wires_ready = False
        self._lock = threading.Lock()
        threading.Thread(target=self._run, daemon=True).start()

    # ---------------------------------------------------------------- sending
    def send(self, line):
        with self._lock:
            if not self.ser:
                return False
            try:
                self.ser.write((line + "\n").encode())
                return True
            except (self._serial.SerialException, OSError):
                self._drop()
                return False

    # ---------------------------------------------------------------- connection
    def _candidates(self):
        if self.forced_port:
            return [self.forced_port]
        found = (sorted(glob.glob("/dev/serial/by-id/*")) + sorted(glob.glob("/dev/ttyUSB*"))
                 + sorted(glob.glob("/dev/ttyACM*")))
        out, seen = [], set()
        for p in found:
            real = os.path.realpath(p)
            if real not in seen and real not in ModuleLink.claimed_ports:
                seen.add(real)
                out.append(real)
        return out

    def _probe(self, port, seconds=5.0):
        """Open `port` and ask "ID?" until the right device answers. True = connected."""
        ser = self._serial.Serial()
        ser.port, ser.baudrate, ser.timeout, ser.write_timeout = port, self.baud, 0.3, 1
        ser.dtr = ser.rts = False       # try not to reset the board when we open the port
        try:
            ser.open()
        except (self._serial.SerialException, OSError):
            return False
        deadline, last_ask = time.monotonic() + seconds, 0.0
        try:
            while time.monotonic() < deadline:
                if time.monotonic() - last_ask >= 1.0:
                    ser.write(b"ID?\n")
                    last_ask = time.monotonic()
                line = ser.readline().decode("utf-8", "replace").strip()
                parts = line.split()
                if len(parts) >= 2 and parts[0] == "ID" and parts[1].startswith(self.prefix):
                    with self._lock:
                        self.ser, self.port, self.device_id = ser, port, parts[1]
                    ModuleLink.claimed_ports.add(port)
                    self.connected = True
                    print(f"[link] {parts[1]} connected on {port}")
                    self.events.put(("link_up", parts[1]))
                    return True
        except (self._serial.SerialException, OSError):
            pass
        ser.close()
        return False

    def _drop(self):
        """Forget the port. Caller holds the lock (or is the reader thread after an error)."""
        was = self.connected
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        ModuleLink.claimed_ports.discard(self.port)
        self.ser, self.port = None, None
        self.connected = False
        self.wires_ready = False
        if was:
            print(f"[link] {self.device_id} disconnected")
            self.events.put(("link_down", self.device_id))

    # ---------------------------------------------------------------- reader thread
    def _run(self):
        while True:
            if not self.ser:
                if not any(self._probe(p) for p in self._candidates()):
                    time.sleep(1.0)
                continue
            try:
                raw = self.ser.readline()
            except (self._serial.SerialException, OSError, TypeError, AttributeError):
                with self._lock:
                    self._drop()
                continue
            if raw:
                self._handle(raw.decode("utf-8", "replace").strip())

    def _handle(self, line):
        if not line:
            return
        parts = line.split()
        cmd, args = parts[0], parts[1:]
        if cmd == "INFO":
            if VERBOSE:
                print(f"[{self.prefix}] {line[5:]}")
        elif cmd == "READY":                       # the ESP32 just (re)booted
            self.wires_ready = False
            self.events.put(("reboot", self.device_id))
            self.send("ID?")                       # makes it report WIRES again
        elif cmd == "WIRES" and args:
            self.wires_ready = args[0] == "1"
        elif cmd in ("STRIKE", "SOLVED") and args:
            self.events.put((cmd.lower(), args[0]))
        elif cmd in ("ID", "PONG", "LIVE"):
            if VERBOSE:
                print(f"[{self.prefix}] {line}")
        else:
            print(f"[{self.prefix}] ?? {line}")


# ============================================================================= hardware
class LcdDisplay:
    """LCD1602 + PCF8574 backpack. Only rewrites rows that changed."""

    def __init__(self, address=None, port=I2C_BUS):
        from RPLCD.i2c import CharLCD
        self.address = address or self.find_address(port)
        self.lcd = CharLCD(i2c_expander="PCF8574", address=self.address, port=port,
                           cols=16, rows=2, charmap="A02", auto_linebreaks=False)
        self.lcd.clear()
        self.shown = [None, None]

    @staticmethod
    def find_address(port):
        from smbus2 import SMBus
        with SMBus(port) as bus:
            for a in [0x27, 0x3F] + list(range(0x20, 0x27)) + list(range(0x38, 0x3F)):
                try:
                    bus.read_byte(a)
                    return a
                except OSError:
                    pass
        raise RuntimeError(f"no I2C LCD found - check wiring/overlay and run: i2cdetect -y {port}")

    def lines(self, l1, l2=""):
        for row, text in enumerate((l1[:16].ljust(16), l2[:16].ljust(16))):
            if text != self.shown[row]:
                try:
                    self.lcd.cursor_pos = (row, 0)
                    self.lcd.write_string(text)
                    self.shown[row] = text
                except OSError as e:
                    print(f"[warn] LCD write failed: {e}")
                    self.shown = [None, None]


class TimerDisplay:
    """TM1637 4-digit display showing MM:SS."""

    def __init__(self, clk, dio):
        import tm1637
        self.tm = tm1637.TM1637(clk=clk, dio=dio)
        self.tm.brightness(val=4)

    def time(self, secs):
        m, s = divmod(max(0, int(secs)), 60)
        self.tm.numbers(m, s, colon=True)

    def text(self, s):
        self.tm.show(s)

    def all_on(self):
        self.tm.numbers(88, 88, colon=True)


class Buzzer:
    """Non-blocking passive-buzzer patterns on a worker thread."""

    def __init__(self, pin):
        from gpiozero import PWMOutputDevice
        self.dev = PWMOutputDevice(pin, frequency=2000, initial_value=0)
        self.q = queue.Queue()
        threading.Thread(target=self._run, daemon=True).start()

    def play(self, notes, interrupt=False):
        if interrupt:
            while not self.q.empty():
                try:
                    self.q.get_nowait()
                except queue.Empty:
                    break
        self.q.put(notes)

    def tick(self):
        if self.q.empty():
            self.q.put([(2200, 0.03)])

    def _run(self):
        while True:
            for freq, dur in self.q.get():
                if freq:
                    self.dev.frequency = freq
                    self.dev.value = 0.5
                time.sleep(dur)
                self.dev.value = 0


class Hardware:
    def __init__(self):
        from gpiozero import LED, Button
        self.led1, self.led2 = LED(PIN_LED1), LED(PIN_LED2)
        # One event queue for everything: ("button",) and the ESP32's events
        self.events = queue.Queue()
        self.button = Button(PIN_BUTTON, pull_up=True, bounce_time=0.05)
        self.button.when_pressed = lambda: self.events.put(("button",))
        self.buzzer = Buzzer(PIN_BUZZER)
        self.timer = TimerDisplay(PIN_TM_CLK, PIN_TM_DIO)
        self.lcd = LcdDisplay(LCD_ADDRESS)
        print(f"[ok] LCD at 0x{self.lcd.address:02X} on I2C bus {I2C_BUS}")
        self.esp32 = ModuleLink(ESP32_ID_PREFIX, self.events, ESP32_PORT)

    def strikes(self, n):
        self.led1.value, self.led2.value = n >= 1, n >= 2

    def clear_events(self):
        while not self.events.empty():
            try:
                self.events.get_nowait()
            except queue.Empty:
                break

    def self_test(self):
        print("[test] LEDs, buzzer, timer, LCD...")
        self.lcd.lines("  KTANE BOMB", " Hardware check")
        self.timer.all_on()
        self.buzzer.play(BOOT_SOUND)
        for led in (self.led1, self.led2):
            led.on()
            time.sleep(0.4)
        time.sleep(0.4)
        self.strikes(0)


# ============================================================================= game
class Game:
    def __init__(self, hw):
        self.hw = hw
        self.link = hw.esp32
        self.strikes = 0
        self.solved = set()
        self.remaining = float(GAME_SECONDS)
        self.link_ok = True

    def info_lines(self):
        l2 = "ESP32 LOST!" if not self.link_ok else \
            f"STRK {self.strikes}/{MAX_STRIKES} MOD {len(self.solved)}/{len(REQUIRED_MODULES)}"
        return f"SN:{self.sn} BAT:{self.batteries}", l2

    def sync_esp32(self):
        """Tell the ESP32 to start a round and show the current mistakes."""
        self.link.send("START")
        self.link.send(f"STRIKES {self.strikes} {MAX_STRIKES}")

    def run(self):
        hw = self.hw
        rng = random.Random()
        self.sn = gen_serial(rng)
        self.batteries = rng.randint(0, 4)
        self.strikes, self.solved = 0, set()
        self.link_ok = self.link.connected
        print(f"[game] serial {self.sn}, batteries {self.batteries}")

        hw.strikes(0)
        hw.lcd.lines(*self.info_lines())
        hw.timer.time(GAME_SECONDS)
        hw.buzzer.play(START_SOUND, interrupt=True)
        hw.clear_events()
        self.sync_esp32()
        time.sleep(0.4)
        hw.clear_events()

        self.remaining, last, shown = float(GAME_SECONDS), time.monotonic(), None
        while True:
            now = time.monotonic()
            self.remaining -= (now - last) * TIMER_RATE[min(self.strikes, len(TIMER_RATE) - 1)]
            last = now
            secs = int(self.remaining + 0.999)
            if secs != shown:
                shown = secs
                hw.timer.time(secs)
                hw.buzzer.tick()
            if self.remaining <= 0:
                return self.boom("time ran out")
            try:
                ev = hw.events.get(timeout=0.02)
            except queue.Empty:
                continue
            if self.on_event(ev):
                return

    # ---------------------------------------------------------------- events
    def on_event(self, ev):
        """Handle one event. Returns True when the game is over."""
        kind = ev[0]
        if kind == "button":
            if BUTTON_SIMULATES_STRIKE:
                return self.strike("button")
        elif kind == "strike":
            return self.strike(ev[1])
        elif kind == "solved":
            return self.solve(ev[1])
        elif kind == "link_down":
            print("[warn] ESP32 disconnected during the game!")
            self.link_ok = False
            self.hw.lcd.lines(*self.info_lines())
        elif kind in ("link_up", "reboot"):
            # ESP32 came back (cable re-plugged or it rebooted): put it back in the round.
            # Its maze is new and its modules start unsolved; the Pi keeps the strikes.
            print(f"[warn] ESP32 {kind} during the game - resyncing")
            self.link_ok = True
            self.sync_esp32()
            self.hw.lcd.lines(*self.info_lines())
        return False

    def strike(self, source):
        hw = self.hw
        self.strikes += 1
        print(f"[STRIKE {self.strikes}/{MAX_STRIKES}] from {source}")
        hw.strikes(self.strikes)
        self.link.send(f"STRIKES {self.strikes} {MAX_STRIKES}")
        hw.lcd.lines(*self.info_lines())
        if self.strikes >= MAX_STRIKES:
            self.boom("too many strikes")
            return True
        hw.buzzer.play(STRIKE_SOUND, interrupt=True)
        return False

    def solve(self, module):
        if module not in REQUIRED_MODULES or module in self.solved:
            return False
        self.solved.add(module)
        print(f"[solved] {module} ({len(self.solved)}/{len(REQUIRED_MODULES)})")
        self.hw.lcd.lines(*self.info_lines())
        if self.solved >= REQUIRED_MODULES:
            self.defuse()
            return True
        self.hw.buzzer.play(SOLVED_SOUND, interrupt=True)
        return False

    # ---------------------------------------------------------------- endings
    def defuse(self):
        print(">>> BOMB DEFUSED <<<")
        self.link.send("DEFUSED")
        self.hw.buzzer.play(DEFUSED_SOUND, interrupt=True)
        self.hw.timer.time(self.remaining)          # freeze the clock on the time left
        self.hw.lcd.lines("BOMB DEFUSED!", "  Press START")

    def boom(self, why):
        print(f">>> BOOM ({why}) <<<")
        self.link.send("BOOM")
        self.hw.buzzer.play(EXPLODE_SOUND, interrupt=True)
        self.hw.timer.text("boom" if why != "time ran out" else "0000")
        self.hw.lcd.lines("**** BOOM! ****", "  Press START")


# ============================================================================= idle / main
def idle_status(hw):
    """(line1, line2, ok): ok = the bomb can be started right now."""
    link = hw.esp32
    if not link.connected:
        return "ESP32 offline", "check USB cable", False
    if not link.wires_ready:
        return "Connect wires", "(all 5 needed)", False
    return "   KTANE BOMB", "  Press START", True


def wait_for_start(hw, hold_seconds=0.0):
    """Block until START is pressed and the bomb is allowed to start."""
    hw.clear_events()
    hold_until = time.monotonic() + hold_seconds     # keep the BOOM/DEFUSED text readable
    shown = None
    while True:
        l1, l2, ok = idle_status(hw)
        if time.monotonic() >= hold_until and (l1, l2) != shown:
            hw.lcd.lines(l1, l2)
            shown = (l1, l2)
        try:
            ev = hw.events.get(timeout=0.1)
        except queue.Empty:
            continue
        if ev[0] == "button":
            if ok or not REQUIRE_MODULES_READY:
                return
            hw.buzzer.play(ERROR_SOUND, interrupt=True)
            print(f"[idle] can't start: {l1} {l2}".rstrip())


def main():
    hw = Hardware()
    try:
        hw.self_test()
        hw.timer.time(GAME_SECONDS)
        hold = 0.0
        while True:
            print("Press START...")
            wait_for_start(hw, hold)
            Game(hw).run()
            hold = END_SCREEN_SECONDS
    finally:
        hw.esp32.send("STOP")        # ESP32 back to standby when the Pi script quits


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nbye")
