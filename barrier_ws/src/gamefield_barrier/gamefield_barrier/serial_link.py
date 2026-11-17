"""Threaded serial link with automatic reconnect for the barrier Arduino."""
import threading

import serial


class SerialLink:
    def __init__(self, port, baud, on_line, on_connection, logger,
                 reconnect_period=2.0, settle_time=2.0):
        self._port = port
        self._baud = baud
        self._on_line = on_line
        self._on_connection = on_connection
        self._log = logger
        self._reconnect_period = reconnect_period
        self._settle_time = settle_time

        self._ser = None
        self._write_lock = threading.Lock()
        self._stop = threading.Event()
        self._reconnect = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._last_err = None
        self.connected = False

    # ---------------- public ----------------
    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=3.0)

    def request_reconnect(self):
        self._reconnect.set()

    def write(self, text):
        with self._write_lock:
            if self._ser is None:
                return False
            try:
                self._ser.write(text.encode('ascii'))
                self._ser.flush()
                return True
            except (serial.SerialException, OSError) as e:
                self._log.warn(f'Serial write failed: {e}')
                self._reconnect.set()
                return False

    # ---------------- internal ----------------
    def _open(self):
        try:
            # exclusive=True: a second process (another node, picocom) cannot open it too
            ser = serial.Serial(self._port, self._baud, timeout=0.1, exclusive=True)
        except (serial.SerialException, OSError, ValueError) as e:
            msg = str(e)
            if msg != self._last_err:   # only log when the error changes
                self._log.warn(f'Cannot open {self._port}: {msg}')
                self._last_err = msg
            return None
        self._log.info(
            f'Opened {self._port}; waiting {self._settle_time:.1f}s for Arduino reset')
        self._stop.wait(self._settle_time)   # opening the port resets the Uno
        return ser

    def _read_loop(self, ser):
        buf = bytearray()
        try:
            while not self._stop.is_set() and not self._reconnect.is_set():
                chunk = ser.read(max(1, ser.in_waiting))
                if not chunk:
                    continue
                buf.extend(chunk)
                while True:
                    idx = buf.find(b'\n')
                    if idx < 0:
                        break
                    raw = bytes(buf[:idx])
                    del buf[:idx + 1]
                    text = raw.decode('ascii', errors='ignore').strip()
                    if text:
                        self._on_line(text)
                if len(buf) > 512:   # garbage protection
                    buf.clear()
        except (serial.SerialException, OSError) as e:
            self._log.warn(f'Serial read error: {e}')

    def _run(self):
        while not self._stop.is_set():
            ser = self._open()
            if ser is None:
                self._stop.wait(self._reconnect_period)
                continue

            self._last_err = None
            self._reconnect.clear()
            with self._write_lock:
                self._ser = ser
            self.connected = True
            self._on_connection(True)

            self._read_loop(ser)

            self.connected = False
            with self._write_lock:
                self._ser = None
            try:
                ser.close()
            except Exception:
                pass
            self._on_connection(False)
            self._stop.wait(self._reconnect_period)