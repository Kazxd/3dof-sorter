"""
arm_link.py - USB serial link to the arm controller (Arduino Nano / ESP32 + PCA9685).

Packet format (ASCII, one per line):

    <PAYLOAD*CS>\n      CS = XOR of every byte of PAYLOAD, two hex digits

Host -> board
    M,seq,a1,a2,a3,a4,a5,m,ms   move: 5 joint servo angles + magnet (1 on / 0 off), in `ms` milliseconds
    H,seq,ms                    go to the firmware's HOME_POSE
    S,seq                       stop where you are
    R,seq                       relax: switch every PWM output off (arm goes limp!)
    Q,seq                       query current pose
    P,seq                       ping

Board -> host
    <READY,ARM5,1>              after reset
    <ACK,seq[,ms]>              packet accepted; for moves, ms = real duration after speed limits
    <DONE,seq>                  motion for seq finished (DONE,seq,PREEMPTED if replaced)
    <ERR,seq,CODE>              rejected (CHECKSUM, FORMAT, RANGE, UNKNOWN, ...)
    <POS,seq,a1..a6,moving>     answer to Q
    <PONG,seq>                  answer to P
    <WARN,...>                  informational (CLAMP when an angle hit a limit)

Every command is retried until ACKed. The board treats a repeated seq as the same
command (re-ACKs it, does not run it twice), so a lost ACK cannot cause a double move.

Why not a bare <a1,a2,a3,a4,a5,magnet>: with no sequence number or checksum the
host cannot tell a garbled line from a good one, or know when a move finished.
"""
import logging
import threading
import time

import config as C

log = logging.getLogger("arm")

try:
    import serial
    from serial.tools import list_ports
except ImportError:                    # lets --sim and the tests run without pyserial
    serial = None
    list_ports = None


class ArmError(Exception):
    pass


# =============================================================================
# framing
# =============================================================================
def checksum(payload):
    x = 0
    for b in payload.encode("ascii"):
        x ^= b
    return f"{x:02X}"


def encode(payload):
    return f"<{payload}*{checksum(payload)}>\n".encode("ascii")


def decode(frame):
    """Text between '<' and '>' -> list of fields, or None if the checksum is wrong."""
    if "*" in frame:
        payload, cs = frame.rsplit("*", 1)
        if checksum(payload) != cs.strip().upper():
            return None
    else:
        payload = frame                # the board's replies carry no checksum
    return [f.strip() for f in payload.split(",")]


def fmt(a):
    return f"{a:.1f}"


# =============================================================================
# real board
# =============================================================================
class ArmLink:
    def __init__(self, port=None, baud=None):
        if serial is None:
            raise ArmError("pyserial is not installed: pip install pyserial")
        self.port = port or C.SERIAL_PORT
        self.baud = baud or C.SERIAL_BAUD
        self.ser = None
        self._seq = 0
        self._cv = threading.Condition()
        self._replies = {}             # (kind, seq) -> fields
        self._ready = False
        self._reader = None
        self._running = False
        self.pose = list(C.HOME_SERVO)

    # ------------------------------------------------------------ connection
    @staticmethod
    def find_port():
        if list_ports is None:
            return None
        ports = list(list_ports.comports())
        keys = ("usb", "ch340", "ch910", "cp210", "ftdi", "arduino", "uart", "wch")
        for p in ports:
            text = f"{p.device} {p.description} {p.manufacturer or ''}".lower()
            if any(k in text for k in keys):
                return p.device
        return ports[0].device if ports else None

    def connect(self):
        port = self.find_port() if self.port == "auto" else self.port
        if not port:
            raise ArmError("no serial port found - is the board plugged in? (set SERIAL_PORT)")
        try:
            self.ser = serial.Serial(port, self.baud, timeout=0.05)
        except serial.SerialException as e:
            raise ArmError(f"cannot open {port}: {e}  (is the Arduino IDE serial monitor open?)")
        self.port = port
        self._running = True
        self._reader = threading.Thread(target=self._read_loop, daemon=True, name="ArmReader")
        self._reader.start()
        log.info("serial open on %s @ %d, waiting for board", port, self.baud)

        deadline = time.time() + C.READY_TIMEOUT_S        # Nano / ESP32 reset on open
        with self._cv:
            while not self._ready and time.time() < deadline:
                self._cv.wait(0.1)
        if not self._ready:
            log.info("no READY banner (board may not have reset) - trying ping")
        self.ping()
        log.info("arm controller online")

    def close(self):
        self._running = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass

    # ------------------------------------------------------------ rx thread
    def _read_loop(self):
        buf = b""
        while self._running:
            try:
                chunk = self.ser.read(256)
            except Exception as e:
                log.error("serial read failed: %s", e)
                self._running = False
                with self._cv:
                    self._cv.notify_all()
                return
            if not chunk:
                continue
            buf += chunk
            while b">" in buf:
                start = buf.find(b"<")
                end = buf.find(b">")
                if start == -1 or start > end:          # junk before a frame
                    buf = buf[end + 1:]
                    continue
                frame = buf[start + 1:end].decode("ascii", "replace")
                buf = buf[end + 1:]
                self._handle(frame)
            if len(buf) > 1024:
                buf = buf[-256:]

    def _handle(self, frame):
        fields = decode(frame)
        if not fields:
            log.warning("bad frame from board: %r", frame)
            return
        kind = fields[0]
        if kind == "READY":
            with self._cv:
                self._ready = True
                self._cv.notify_all()
            log.info("board: %s", ",".join(fields))
            return
        if kind == "WARN":
            log.warning("board: %s", ",".join(fields))
            return
        if kind in ("ACK", "DONE", "ERR", "POS", "PONG") and len(fields) >= 2:
            try:
                seq = int(fields[1])
            except ValueError:
                seq = -1
            with self._cv:
                self._replies[(kind, seq)] = fields
                if len(self._replies) > 200:          # forget very old replies
                    for k in list(self._replies)[:100]:
                        del self._replies[k]
                self._cv.notify_all()
            if kind == "ERR":
                log.error("board rejected seq %s: %s", fields[1], ",".join(fields[2:]))
            return
        log.debug("board: %s", frame)

    # ------------------------------------------------------------ tx helpers
    def _next_seq(self):
        self._seq = self._seq % 9999 + 1
        return self._seq

    def _wait(self, kinds, seq, timeout, on_wait=None):
        """Wait for one of `kinds` with this seq. Calls on_wait() ~every 30 ms (UI refresh)."""
        deadline = time.time() + timeout
        while True:
            with self._cv:
                for k in kinds:
                    if (k, seq) in self._replies:
                        return self._replies.pop((k, seq))
                if ("ERR", seq) in self._replies:
                    err = self._replies.pop(("ERR", seq))
                    raise ArmError(f"board error for seq {seq}: {','.join(err[2:])}")
                if not self._running:
                    raise ArmError("serial link closed")
                remaining = deadline - time.time()
                if remaining <= 0:
                    return None
                self._cv.wait(min(remaining, 0.03))
            if on_wait:
                on_wait()

    def _command(self, payload_fields, done_timeout=None, reply=("ACK",), on_wait=None):
        seq = self._next_seq()
        cmd, rest = payload_fields[0], payload_fields[1:]
        frame = encode(",".join([cmd, str(seq)] + [str(f) for f in rest]))
        got = None
        for attempt in range(C.SEND_RETRIES):
            try:
                self.ser.write(frame)
            except Exception as e:
                raise ArmError(f"serial write failed: {e}")
            got = self._wait(reply, seq, C.ACK_TIMEOUT_S, on_wait)
            if got:
                break
            log.warning("no reply to %s (attempt %d/%d)", frame.strip(), attempt + 1, C.SEND_RETRIES)
        if not got:
            raise ArmError(f"board did not answer {cmd} after {C.SEND_RETRIES} tries")
        if done_timeout is not None:
            if len(got) >= 3 and got[2].isdigit():      # ACK carries the real duration
                done_timeout = max(done_timeout, int(got[2]) / 1000.0 + C.DONE_MARGIN_S)
            if not self._wait(("DONE",), seq, done_timeout, on_wait):
                raise ArmError(f"move {seq} did not finish within {done_timeout:.1f} s")
        return got

    # ------------------------------------------------------------ public API
    def ping(self):
        self._command(["P"], reply=("PONG",))

    def move(self, pose6, ms, on_wait=None):
        """Move to a 6-value pose (5 servo angles + magnet 0/1) and block until DONE.
        Returns the duration the board actually used, in ms."""
        if len(pose6) != 6:
            raise ValueError("pose must have 6 values")
        fields = ["M"] + [fmt(a) for a in pose6] + [str(int(ms))]
        # the board may stretch `ms` to respect its speed limits; allow for a full-range move
        ack = self._command(fields, done_timeout=ms / 1000.0 + C.DONE_MARGIN_S, on_wait=on_wait)
        self.pose = list(pose6)
        # the board's real duration after its speed limits (None if it didn't report one)
        return int(ack[2]) if len(ack) >= 3 and ack[2].isdigit() else None

    def home(self, ms=1500, on_wait=None):
        self._command(["H", str(int(ms))], done_timeout=ms / 1000.0 + C.DONE_MARGIN_S, on_wait=on_wait)
        self.pose = list(C.HOME_SERVO)

    def stop(self):
        self._command(["S"])

    def relax(self):
        self._command(["R"])

    def query(self):
        f = self._command(["Q"], reply=("POS",))
        return [float(v) for v in f[2:8]], f[8] == "1"


# =============================================================================
# simulator (no hardware)
# =============================================================================
SIM_SPEED_DPS = (90, 60, 75, 120, 180)     # mirrors MAX_SPEED_DPS in the firmware


class SimArm:
    """Same interface as ArmLink; prints packets instead of sending them."""

    def __init__(self, speed=4.0, quiet=False):
        self.speed = speed           # 4 = runs 4x faster than real time
        self.quiet = quiet
        self.pose = list(C.HOME_SERVO)
        self.sent = []               # every payload, for tests
        self._seq = 0

    def _log(self, payload):
        self.sent.append(payload)
        if not self.quiet:
            log.info("[sim] -> %s", encode(payload).decode().strip())

    def _sleep(self, ms, on_wait):
        end = time.time() + ms / 1000.0 / self.speed
        while time.time() < end:
            if on_wait:
                on_wait()
            time.sleep(0.02)

    def _next(self):
        self._seq = self._seq % 9999 + 1
        return self._seq

    def connect(self):
        log.info("[sim] simulated arm ready")

    def close(self):
        pass

    def ping(self):
        self._log(f"P,{self._next()}")

    def move(self, pose6, ms, on_wait=None):
        if len(pose6) != 6:
            raise ValueError("pose must have 6 values")
        self._log(",".join(["M", str(self._next())] + [fmt(a) for a in pose6] + [str(int(ms))]))
        # same speed limits as the firmware, so the simulator reports realistic durations
        need = max([int(1875 * abs(a - b) / v) for a, b, v in zip(pose6[:5], self.pose[:5], SIM_SPEED_DPS)] + [ms, 150])
        self._sleep(need, on_wait)
        self.pose = list(pose6)
        return need

    def home(self, ms=1500, on_wait=None):
        self._log(f"H,{self._next()},{int(ms)}")
        self._sleep(ms, on_wait)
        self.pose = list(C.HOME_SERVO)

    def stop(self):
        self._log(f"S,{self._next()}")

    def relax(self):
        self._log(f"R,{self._next()}")

    def query(self):
        return list(self.pose), False
