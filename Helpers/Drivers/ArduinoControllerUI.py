import qt
import slicer
import vtk

from .ArduinoController import ArduinoController

class ArduinoControllerUI(qt.QWidget):
    def __init__(self, controller):
        super().__init__()
        self.controller = controller
        print(f"[DEBUG] UI received controller ID: {id(self.controller)}")

        self.arduinoNode = None
        self.observerTag = None
        self.initUI()

    def initUI(self):
        layout = qt.QVBoxLayout()
        self.setLayout(layout)

        self.portInput = qt.QLineEdit()
        self.portInput.setPlaceholderText("Enter Arduino Port (e.g., COM3)")

        self.connectBtn = qt.QPushButton("Connect to Arduino")
        self.connectBtn.connect('clicked()', self.onConnect)

        self.commandInput = qt.QLineEdit()
        self.commandInput.setPlaceholderText("Enter command (e.g., PING)")

        self.sendBtn = qt.QPushButton("Send to Arduino")
        self.sendBtn.connect('clicked()', self.onSend)

        self.responseLabel = qt.QLabel("Arduino Response: None")

        layout.addWidget(qt.QLabel("Arduino Controller Settings"))
        layout.addWidget(self.portInput)
        layout.addWidget(self.connectBtn)
        layout.addWidget(self.commandInput)
        layout.addWidget(self.sendBtn)
        layout.addWidget(self.responseLabel)

    def onConnect(self):
        port = self.portInput.text.strip()
        slicer.util.infoDisplay(f"Entered port is {port}")

        if not port:
            slicer.util.infoDisplay("Please enter a valid Arduino port.")
            return

        connected = self.controller.connect(port=port, baud="9600", samplesPerSecond="10")
        if connected:
            slicer.util.infoDisplay(f"Connected to Arduino on {port}")
            self.observeArduinoData()  # 🔥 Make sure this happens again after reconnection
        else:
            slicer.util.errorDisplay(f"Failed to connect to {port}")

    def onSend(self):
        cmd = self.commandInput.text.strip()
        if not cmd:
            slicer.util.errorDisplay("Please enter a command.")
            return

        success = self.controller.sendCommand(cmd)
        if success:
            slicer.util.infoDisplay(f"Sent: {cmd}")
        else:
            slicer.util.errorDisplay("Command failed or Arduino not connected.")

    def observeArduinoData(self):
        if self.arduinoNode and self.observerTag:
            self.arduinoNode.RemoveObserver(self.observerTag)

        self.arduinoNode = slicer.mrmlScene.GetFirstNodeByName("arduinoNode")
        if self.arduinoNode:
            self.observerTag = self.arduinoNode.AddObserver(
                vtk.vtkCommand.ModifiedEvent,
                self.onArduinoDataReceived
            )
            print("[DEBUG] Observer added to arduinoNode")


    def onArduinoDataReceived(self, caller, event):
        message = caller.GetParameter("Data")
        print(f"[DEBUG] Received from Arduino: {message}")  # ← Add this
        self.responseLabel.setText(f"Arduino Response: {message}")


    def cleanup(self):
        if self.arduinoNode and self.observerTag:
            self.arduinoNode.RemoveObserver(self.observerTag)
            self.observerTag = None

        # Properly disconnect the serial port
        if self.controller:
            self.controller.disconnect()
