import socket
import time
import os 


class GalilController:
    def __init__(self):
        self._sock = None
        self._ip = None
        self._port = None
        self._connected = False

        self._sock = None
        self._log_only = True  # Enable offline logging

    def connect(self, ip="192.168.42.100", port=23) -> bool:
        """Open a TCP connection and flush any pending input."""
        if self._log_only:
            print("[GALIL] Offline log-only mode enabled – commands will be printed only")
            return True
        try:
            self._sock = socket.create_connection((ip, port), timeout=2)
            # After connect, switch to shorter read timeout
            self._sock.settimeout(0.2)

            # Flush any residual data on the socket
            try:
                while True:
                    chunk = self._sock.recv(1024)
                    if not chunk:
                        break
            except socket.timeout:
                pass

            self._ip = ip
            self._port = port
            self._connected = True
            print(f"[Galil] Connected to {ip}:{port}")
            return True
        except Exception as e:
            print(f"[Galil] Connection failed: {e}")
            self._connected = False
            return False

    def disconnect(self):
        """Close the socket and mark as disconnected."""
        if self._sock:
            self._sock.close()
            self._sock = None
        self._connected = False
        print("[Galil] Disconnected.")

    @property
    def is_connected(self) -> bool:
        return self._log_only or self._sock is not None

        # return self._connected

    def send(self, cmd: str, retries: int = 1) -> str:
        """
        Send a Galil command (with CR terminator) and return the response
        (stripped of whitespace and any trailing colon). Retries once on error.
        """
        if self._log_only:
            print(f"[GALIL CMD] {cmd.strip()}")
            return "0"  # Dummy safe reply

        full_cmd_test = cmd.strip() + "\r"
        print(full_cmd_test)
        if not self._connected or not self._sock:
            print("[Galil] Not connected.")
            return ""
        last_error = ""
        for attempt in range(retries):
            try:
                full_cmd = cmd.strip() + "\r"
                self._sock.sendall(full_cmd.encode("ascii"))
                response = self._read_response()
                # Strip whitespace and any trailing colon from Galil responses
                return response.strip().rstrip(":")
            except Exception as e:
                last_error = str(e)
                print(f"[Galil] Attempt {attempt + 1} failed for cmd '{cmd}': {last_error}")
                time.sleep(0.05)
        print(f"[Galil] All retries failed for cmd '{cmd}': {last_error}")
        return ""

    def _read_response(self) -> str:
        """Read until terminator (‘\r’) or timeout (0.5 s)."""
        data = b""
        start = time.time()
        while time.time() - start < 0.5:
            try:
                chunk = self._sock.recv(1024)
                if not chunk:
                    break
                data += chunk
                if b"\r" in chunk:
                    print(data)
                    break
            except socket.timeout:
                break
        return data.decode("ascii", errors="ignore")
