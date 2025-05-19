# URDFLoader.py
import slicer
from slicer.ScriptedLoadableModule import *
import qt
import vtk
import os

from Helpers.urdf_loader_logic import URDFLoaderLogic
from Helpers.Drivers.galil_controller import GalilController
from Helpers.ICH_Robot.ich_robot import ICHRobot
from Helpers.ICH_Robot.ich_custom_ui_wrapper import ICH_CustomUIWrapper

ROBOT_PLUGINS = {
    "ICHRobotUI.ui": {
        "robot_cls": ICHRobot,
        "controller_type": GalilController,
    },
}

class URDFLoader(ScriptedLoadableModule):
    def __init__(self, parent):
        ScriptedLoadableModule.__init__(self, parent)
        self.parent.title = "BM2 Universal Slicer Software"
        self.parent.categories = ["BM2 Lab Georgia Tech"]
        self.parent.contributors = [""]
        self.parent.helpText = (
            ""
        )
        self.parent.acknowledgementText = "Developed by Karthik Koundinya."

class URDFLoaderWidget(ScriptedLoadableModuleWidget):
    def setup(self):
        ScriptedLoadableModuleWidget.setup(self)
        self.logic = URDFLoaderLogic()
        self.controllerBlocks = []

        self.urdfLabel = qt.QLabel("Select a URDF file to load and render in 3D.")
        self.layout.addWidget(self.urdfLabel)

        self.urdfFileButton = qt.QPushButton("Select URDF File")
        self.urdfFileButton.connect('clicked()', self.onSelectURDFFile)
        self.layout.addWidget(self.urdfFileButton)

        self.urdfFileLabel = qt.QLabel("No file selected")
        self.layout.addWidget(self.urdfFileLabel)

        self.loadURDFButton = qt.QPushButton("Load and Render URDF")
        self.loadURDFButton.enabled = False
        self.loadURDFButton.connect('clicked()', self.onLoadURDF)
        self.layout.addWidget(self.loadURDFButton)

        self.addControllerButton = qt.QPushButton("➕ Add Controller")
        self.addControllerButton.connect('clicked()', self.onAddControllerBlock)
        self.layout.addWidget(self.addControllerButton)

        self.controllerBlocksLayout = qt.QVBoxLayout()
        self.layout.addLayout(self.controllerBlocksLayout)
        self.layout.addStretch()

        self.customUIFrame = qt.QFrame()
        self.customUIFrame.setFrameStyle(qt.QFrame.Box | qt.QFrame.Raised)
        self.customUIFrame.setLineWidth(1)
        customUIFrameLayout = qt.QVBoxLayout(self.customUIFrame)

        self.loadUIBtn = qt.QPushButton("➕ Load Custom UI")
        self.loadUIBtn.connect('clicked()', self.onLoadCustomUI)
        customUIFrameLayout.addWidget(self.loadUIBtn)

        self.customUILayout = qt.QVBoxLayout()
        customUIFrameLayout.addLayout(self.customUILayout)

        self.layout.addWidget(self.customUIFrame)

    def onSelectURDFFile(self):
        urdfPath = qt.QFileDialog.getOpenFileName(None, "Select URDF File", "", "URDF Files (*.urdf)")
        if urdfPath:
            self.urdfFileLabel.setText(urdfPath)
            self.urdfFilePath = urdfPath
            self.loadURDFButton.enabled = True

    def onLoadURDF(self):
        if hasattr(self, 'urdfFilePath'):
            self.logic.load_and_render(self.urdfFilePath)

    def onAddControllerBlock(self):
        blockWidget = qt.QFrame()
        blockWidget.setFrameStyle(qt.QFrame.Box | qt.QFrame.Plain)
        blockWidget.setLineWidth(1)
        blockLayout = qt.QVBoxLayout()
        blockWidget.setLayout(blockLayout)

        topLayout = qt.QHBoxLayout()
        dropdown = qt.QComboBox()
        dropdown.addItems(["-- Select Controller --", "Galil", "Arduino", "PMD-401"])
        topLayout.addWidget(dropdown)

        deleteButton = qt.QPushButton("➖")
        topLayout.addWidget(deleteButton)
        blockLayout.addLayout(topLayout)

        finalizeButton = qt.QPushButton("Add this controller")
        blockLayout.addWidget(finalizeButton)

        controllerUIContainer = qt.QWidget()
        controllerUILayout = qt.QVBoxLayout()
        controllerUIContainer.setLayout(controllerUILayout)
        controllerUIContainer.setVisible(False)
        blockLayout.addWidget(controllerUIContainer)

        controllerBlock = {
            "widget": blockWidget,
            "controllerWidget": None,
            "controllerLogic": None,
            "container": controllerUIContainer
        }
        self.controllerBlocks.append(controllerBlock)

        def onFinalize():
            controllerName = dropdown.currentText
            while controllerUILayout.count():
                child = controllerUILayout.takeAt(0)
                if child.widget():
                    child.widget().deleteLater()
            controllerUIContainer.setVisible(False)

            if controllerName == "Galil":
                controller = GalilController()
                controllerBlock["controllerLogic"] = controller
                controllerUIContainer.setVisible(True)
            if controllerName == "Arduino":
                controller = GalilController()
                controllerBlock["controllerLogic"] = controller
                controllerUIContainer.setVisible(True)
            if controllerName == "PMD-401":
                controller = GalilController()
                controllerBlock["controllerLogic"] = controller
                controllerUIContainer.setVisible(True)

        def onDelete():
            self.controllerBlocksLayout.removeWidget(blockWidget)
            self.controllerBlocks.remove(controllerBlock)
            blockWidget.hide()
            qt.QTimer.singleShot(0, blockWidget.deleteLater)

        finalizeButton.connect('clicked()', onFinalize)
        deleteButton.connect('clicked()', onDelete)

        self.controllerBlocksLayout.addWidget(blockWidget)

    def onLoadCustomUI(self):
        uiPath = qt.QFileDialog.getOpenFileName(None, "Select .ui File", "", "Qt UI Files (*.ui)")
        if not uiPath:
            return

        basename = os.path.basename(uiPath)
        plugin = ROBOT_PLUGINS.get(basename)
        if not plugin:
            slicer.util.infoDisplay("This UI is currently not supported")
            return

        controller = self._findController(plugin["controller_type"])
        if not controller:
            slicer.util.infoDisplay("Please add the required controller block first.")
            return

        robot = plugin["robot_cls"](controller)
        wrapper = ICH_CustomUIWrapper(ui_path=uiPath, parent=self.parent)

        if not wrapper.get_widget():
            slicer.util.errorDisplay("Failed to load custom UI.")
            return

        self.customUILayout.addWidget(wrapper.get_widget())
        slicer.util.infoDisplay(f"Loaded custom UI from {uiPath}")

    def _findController(self, cls):
        for block in self.controllerBlocks:
            logic = block.get("controllerLogic")
            if isinstance(logic, cls):
                return logic
        return None
