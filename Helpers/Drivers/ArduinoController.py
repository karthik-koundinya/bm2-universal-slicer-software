# ArduinoController.py
from .ArduinoConnect import ArduinoConnectLogic

class ArduinoController:
    def __init__(self):
        self.arduinoLogic = ArduinoConnectLogic()

    def connect(self, port, baud="9600", samplesPerSecond="10"):
        """Connect to Arduino with port, baud rate, and polling rate."""
        print(f"[INFO] Attempting to connect to Arduino at {port} with baud {baud}...")
        self.disconnect()  # Ensure any existing connection is closed before reconnecting
        connected = self.arduinoLogic.connect(port, baud, samplesPerSecond)
        if connected:
            print(f"[SUCCESS] Connected to Arduino at {port}")
        else:
            print(f"[ERROR] Failed to connect to Arduino at {port}")
        return connected

    def sendCommand(self, command):
        """Send command to Arduino if connected."""
        if self.arduinoLogic.arduinoConnection and self.arduinoLogic.arduinoConnection.isOpen():
            print(f"[INFO] Sending command: {command}")
            return self.arduinoLogic.sendMessage(command)
        else:
            print("[ERROR] Cannot send command: Arduino is not connected.")
            return False

    def disconnect(self):
        """Disconnect Arduino safely, stop polling thread, and free the port."""
        if self.arduinoLogic.arduinoConnection:
            print("[INFO] Disconnecting Arduino...")
            self.arduinoLogic.disconnect()
            print("[SUCCESS] Arduino disconnected successfully.")
        else:
            print("[INFO] No active Arduino connection to disconnect.")
