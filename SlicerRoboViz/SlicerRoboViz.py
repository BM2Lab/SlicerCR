import sys
import os

import logging
import os
from typing import Annotated, Optional
import math
import vtk
import time
import slicer
from slicer.i18n import tr as _
from slicer.i18n import translate
from slicer.ScriptedLoadableModule import *
from slicer.util import VTKObservationMixin
from slicer.parameterNodeWrapper import (
    parameterNodeWrapper,
    WithinRange,
)
import qt
from slicer import vtkMRMLScalarVolumeNode
from Scripts.Logic.robot_visualizer import RobotVisualizer
import numpy as np
import csv
from datetime import datetime
#
# SlicerRoboViz
#


class SlicerRoboViz(ScriptedLoadableModule):
    """Uses ScriptedLoadableModule base class, available at:
    https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self, parent):
        ScriptedLoadableModule.__init__(self, parent)
        self.parent.title = _("SlicerRoboViz")        
        self.parent.categories = ["SlicerCR"]  
        self.parent.dependencies = []  
        self.parent.contributors = ["Letian Ai, BM2 Lab in Georgia Institute of Technology"] 
        self.parent.helpText = _("""This a scripted loadable module that provides functionality to visualize robot motion in Slicer.""")
        self.parent.acknowledgementText = _(""" This file was originally developed letian Ai (BM2 Lab)""")

#
# SlicerRoboVizWidget
#


class SlicerRoboVizWidget(ScriptedLoadableModuleWidget, VTKObservationMixin):
    """Uses ScriptedLoadableModuleWidget base class, available at:
    https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self, parent=None) -> None:
        """Called when the user opens the module the first time and the widget is initialized."""
        ScriptedLoadableModuleWidget.__init__(self, parent)
        VTKObservationMixin.__init__(self)  # needed for parameter node observation
        self.logic = None
        self._parameterNode = None
        self._parameterNodeGuiTag = None

        self.robot_count = 1  # Track number of robots
        self.robot_panels = []  # Store references to robot panels


    def setup(self) -> None:
        """Called when the user opens the module the first time and the widget is initialized."""
        ScriptedLoadableModuleWidget.setup(self)

        uiWidget = slicer.util.loadUI(self.resourcePath("UI/SlicerRoboViz.ui"))
        self.layout.addWidget(uiWidget)
        self.ui = slicer.util.childWidgetVariables(uiWidget)
        uiWidget.setMRMLScene(slicer.mrmlScene)
        self.logic = SlicerRoboVizLogic()
        self.addObserver(slicer.mrmlScene, slicer.mrmlScene.StartCloseEvent, self.on_scene_start_close)
        self.addObserver(slicer.mrmlScene, slicer.mrmlScene.EndCloseEvent, self.on_scene_end_close)
        # Connect the "Add New Robot" button
        self.ui.AddNewRobotPushBtn.clicked.connect(self.onAddNewRobot)
        self.ui.LoadRobotPushBtn.clicked.connect(lambda: self.onLoadRobot(self.robot_count, self.ui.inputsCollapsibleButton))
        self.ui.RemoveRobotPushBtn.clicked.connect(lambda: self.onRemoveRobot(self.ui.inputsCollapsibleButton))
        # Store reference to the main vertical layout
        self.main_layout = self.ui.verticalLayout
        
        # Find the position of the vertical spacer for insertion
        self.spacer_index = None
        for i in range(self.main_layout.count()):
            item = self.main_layout.itemAt(i)
            if item.spacerItem():
                self.spacer_index = i
                break
  
    def onAddNewRobot(self):
        """Create and add a new robot panel from template UI"""
        self.robot_count += 1
        robot_panel = self.createRobotPanelFromTemplate(self.robot_count)
        
        if robot_panel:
            self.robot_panels.append(robot_panel)
            
            # Insert before the spacer
            insert_index = self.spacer_index if self.spacer_index is not None else self.main_layout.count()
            self.main_layout.insertWidget(insert_index-1, robot_panel)
            
            # Update spacer index
            if self.spacer_index is not None:
                self.spacer_index += 1
    
    def createRobotPanelFromTemplate(self, robot_number):
        """Load robot panel from template UI file"""
        
        # Get the path to your template UI file
        # Assuming the template is in the same directory as your module
        module_dir = os.path.dirname(os.path.abspath(__file__))
        template_path = os.path.join(module_dir,"Resources", "UI", "RobotPanelTemplate.ui")  # Your template UI file
        
        if not os.path.exists(template_path):
            print(f"Template UI file not found: {template_path}")
            return None
        
        try:
            # Load the UI file
            loader = qt.QUiLoader()
            ui_file = qt.QFile(template_path)
            ui_file.open(qt.QFile.ReadOnly)
            robot_panel = loader.load(ui_file)
            ui_file.close()
            
            if not robot_panel:
                print("Failed to load template UI")
                return None
                
            # Customize the loaded panel for this specific robot
            self.customizeRobotPanel(robot_panel, robot_number)
            
            return robot_panel
            
        except Exception as e:
            print(f"Error loading template UI: {str(e)}")
            return None
    
    def customizeRobotPanel(self, robot_panel, robot_number):
        """Customize the loaded template for specific robot instance"""
        
        if robot_panel: # ctkCollapsibleButton
           robot_panel.setText(f"Robot {robot_number}")
        
        # Find and connect the Load Robot button
        load_button = robot_panel.findChild(qt.QPushButton, "LoadRobotPushBtn")
        if load_button:
            load_button.clicked.connect(lambda: self.onLoadRobot(robot_number, robot_panel))
        
        # Find and connect the Remove Robot button
        remove_button = robot_panel.findChild(qt.QPushButton, "RemoveRobotPushBtn")
        if remove_button:
            remove_button.clicked.connect(lambda: self.onRemoveRobot( robot_panel))

    def __getPath(self, robot_panel):
        path_edit = getattr(robot_panel, 'PathLineEdit', None)
        if not path_edit:
            qt.QMessageBox.warning(None, "Warning", "Path edit widget not found")
            return
        urdf_path = path_edit.currentPath if hasattr(path_edit, 'currentPath') else ""
        return urdf_path

    def onLoadRobot(self, robot_number, robot_panel):
        """Handle loading a robot from URDF file"""
        urdf_path = self.__getPath(robot_panel)
        if not urdf_path:
            qt.QMessageBox.warning(None, "Warning", "Please select a URDF file first")
            return
            
        print(f"Loading robot {robot_number} from: {urdf_path}")
        # robot loading logic here
        if self.logic.loadRobot(urdf_path, robot_number):
            robot_name = self.logic._getRobotNameFromPath(urdf_path)
            robot_panel.setText(f"Robot {robot_number}: {robot_name}")
        
    def onRemoveRobot(self, robot_panel):
        
        robot_number =int(robot_panel.text.split(":")[0].split(" ")[1])
        reply = qt.QMessageBox.question(None, "Confirm Removal", 
                                      f"Are you sure you want to remove Robot {robot_number}?",
                                      qt.QMessageBox.Yes | qt.QMessageBox.No)
        
        if reply == qt.QMessageBox.Yes:
            
            # self.logic.RemoveRobot(self._getPath(robot_panel))
            self.logic.RemoveRobot(robot_number)
            self.main_layout.removeWidget(robot_panel)
            if robot_panel in self.robot_panels:
                self.robot_panels.remove(robot_panel)
            robot_panel.hide()
            qt.QTimer.singleShot(0, robot_panel.deleteLater)
            if self.spacer_index is not None:
                self.spacer_index -= 1
            
            print(f"Removed Robot {robot_number}")
    
    def getAllRobotData(self):
        """Helper method to get data from all robot panels"""
        robot_data = []
        for panel in self.robot_panels:
            if hasattr(panel, 'urdf_file_path') and panel.urdf_file_path:
                robot_data.append({
                    'robot_number': panel.robot_number,
                    'urdf_path': panel.urdf_file_path
                })
        return robot_data
    
    def setParameterNode(self) -> None:
        print("Setting parameter node for SlicerRobotViz")
        param = slicer.mrmlScene.GetFirstNodeByName("SlicerRobotVizParameters")
        if not param:
            print("No parameter node.")
            return
        param.SetParameter("robotName", "ExampleRobot" )  

    def cleanup(self) -> None:
        """Called when the application closes and the module widget is destroyed."""
        self.logic.RemoveAllRobots()
        self.logic.cleanup()

    def enter(self) -> None:
        """Called each time the user opens this module."""
        pass

    def exit(self) -> None:
        """Called each time the user opens a different module."""
        pass

    def on_scene_start_close(self, caller, event) -> None:
        """Called just before the scene is closed."""
        pass

    def on_scene_end_close(self, caller, event) -> None:
        """Called just after the scene is closed."""
        pass


@parameterNodeWrapper
class JointStateNode:
    """Parameter node wrapper for joint states."""
    time_stamp: float = 0.0
    joint_names: list[str] = []
    joint_positions: list[float] = []

@parameterNodeWrapper
class RobotNode:
    """Parameter node wrapper for robot."""
    robot_names: list[str] = []
    urdf_file_paths: list[str] = []


#
# SlicerRoboVizLogic
#


class SlicerRoboVizLogic(ScriptedLoadableModuleLogic):
    """This class should implement all the actual
    computation done by your module.  The interface
    should be such that other python code can import
    this class and make use of the functionality without
    requiring an instance of the Widget.
    Uses ScriptedLoadableModuleLogic base class, available at:
    https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self) -> None:
        """Called when the logic class is instantiated. Can be used for initializing member variables."""
        ScriptedLoadableModuleLogic.__init__(self)
        self.robot_visualizers = {}
        
        # Initialize robot node
        parameter_node = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLScriptedModuleNode')
        parameter_node.SetName("SRVRobotsNode")
        self.robot_node = RobotNode(parameter_node)
        


    def _getRobotNameFromPath(self, path: str) -> Optional[str]:
        """Get the robot name from the path if it exists."""
        for key, robot_data in self.robot_visualizers.items():
            if robot_data["path"] == path:
                return robot_data["robot_name"]
        return None
    
    def __getRobotFromNumber(self, robot_number: int) -> Optional[str]:
        """Get the robot name from the number if it exists."""
        for key, robot_data in self.robot_visualizers.items():
            if f"robot_{robot_number}" in key:
                return key
        return None
    
    def isRobotLoaded(self, robot_name: str) -> bool:
        return self.robot_visualizers.get(robot_name) is not None
    # we need to link visualizer to each robot panel instead of path

    def loadRobot(self, urdf_file_path: str, robot_number: int) -> bool:
        """Load a robot from a URDF file and render it in the scene."""    
        # try:
        if not self.__getRobotFromNumber(robot_number): #TODO update the exisiting robot
            
            robot_visualizer = RobotVisualizer()
            robot_visualizer.visualizeRobot(urdf_file_path)
            # avoid duplicate robot name
            if self.__getVisualizerFromName(robot_visualizer.robot_name):
                qt.QMessageBox.critical(None, "Error", f"Robot {robot_visualizer.robot_name} already loaded")
                return False
            self.robot_visualizers[f"robot_{robot_number}"] = {
                "path": urdf_file_path,
                "robot_number": robot_number,
                "robot_name": robot_visualizer.robot_name,
                "visualizer": robot_visualizer
            }
            self.robot_node.robot_names.append(robot_visualizer.robot_name)
            self.robot_node.urdf_file_paths.append(urdf_file_path)
        else:
            robot_visualizer = self.robot_visualizers[f"robot_{robot_number}"]["visualizer"]
            robot_visualizer.cleanup()
            robot_visualizer.visualizeRobot(urdf_file_path)
            if self.robot_visualizers[f"robot_{robot_number}"]["path"] != urdf_file_path:
                self.robot_node.urdf_file_paths.remove(self.robot_visualizers[f"robot_{robot_number}"]["path"])
                self.robot_node.urdf_file_paths.append(urdf_file_path)
                self.robot_visualizers[f"robot_{robot_number}"]["path"] = urdf_file_path
                
            if self.robot_visualizers[f"robot_{robot_number}"]["robot_name"] != robot_visualizer.robot_name:
                self.robot_node.robot_names.remove(self.robot_visualizers[f"robot_{robot_number}"]["robot_name"])
                self.robot_node.robot_names.append(robot_visualizer.robot_name)
                self.robot_visualizers[f"robot_{robot_number}"]["robot_name"] = robot_visualizer.robot_name
            print(f"Robot {robot_visualizer.robot_name} updated")
        return True
        # except Exception as e:
        #     qt.QMessageBox.critical(None, "Error", f"Failed to load robot: {str(e)}")
        #     return False
        
    # methods for update the robot
    def __getVisualizerFromName(self, robot_name:str) -> Optional[RobotVisualizer]:
        """Find the visualizer from the robot name."""
        for key, visualizer in self.robot_visualizers.items():
            if robot_name == visualizer["robot_name"]:
                return visualizer["visualizer"]
        return None
    
    def updateRobotState(self, robot_name:str, joint_positions: list[float]=None, backbone_SPs: np.ndarray=None, segment_end_transforms: np.ndarray=None) -> bool:
        
        start_time = time.time()
        
        visualizer = self.__getVisualizerFromName(robot_name)
        if not visualizer:
            qt.QMessageBox.critical(None, "Error", f"Robot {robot_name} is not loaded")
            return False
            
        # was_modified = visualizer.robot_state_node.StartModify()
        was_modified = visualizer.robot_state_node.StartModify()
        if joint_positions is not None:
            visualizer.updateJointState(joint_positions)
        if backbone_SPs is not None:
            visualizer.updateSegmentState(backbone_SPs, segment_end_transforms)
        visualizer.robot_state_node.EndModify(was_modified)
        
        execution_time = time.time() - start_time
        
        return True, execution_time

    def getRobotClass(self, robot_name:str):
        visualizer = self.__getVisualizerFromName(robot_name)
        if not visualizer:
            print(f"Robot {robot_name} is not loaded")
            return None
        return visualizer.robot

    def getSegmentGlobalWaypoints(self, robot_name:str) -> np.ndarray:
        visualizer = self.__getVisualizerFromName(robot_name)
        if not visualizer:
            print(f"Robot {robot_name} is not loaded")
            return None
        return visualizer.segment_global_waypoints
    
    def getTransformsHierarchy(self, robot_name:str) -> dict:
        visualizer = self.__getVisualizerFromName(robot_name)
        if not visualizer:
            qt.QMessageBox.critical(None, "Error", f"Robot {robot_name} is not loaded")
            return None
        return visualizer.getTransformsHierarchy()
    
    def RemoveRobot(self, robot_number: int) -> bool:
        """Remove a robot from the scene."""
        if f"robot_{robot_number}" in self.robot_visualizers:
            visualizer = self.robot_visualizers[f"robot_{robot_number}"]
            visualizer["visualizer"].cleanup()
            self.robot_node.urdf_file_paths.remove(visualizer["path"])
            self.robot_node.robot_names.remove(visualizer["robot_name"])
            self.robot_visualizers.pop(f"robot_{robot_number}", None)
            
            qt.QMessageBox.information(None, "Success", f"Robot {visualizer['robot_name']} removed successfully")

    def RemoveAllRobots(self):
        for name, visualizer in self.robot_visualizers.items():
            visualizer["visualizer"].cleanup()
            self.robot_node.urdf_file_paths.remove(visualizer["path"])
            self.robot_node.robot_names.remove(visualizer["robot_name"])
            # self.robot_visualizers.pop(name, None)
    
    def cleanup(self):
        """Cleanup resources when module is destroyed."""
        pass

#
# SlicerRoboVizTest
#


class SlicerRoboVizTest(ScriptedLoadableModuleTest):
    """
    This is the test case for your scripted module.
    Uses ScriptedLoadableModuleTest base class, available at:
    https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def setUp(self):
        """Do whatever is needed to reset the state - typically a scene clear will be enough."""
        slicer.mrmlScene.Clear()

    def runTest(self):
        """Run as few or as many tests as needed here."""
        self.setUp()
        self.test_SlicerRoboViz1()

    def test_SlicerRoboViz1(self):
        """Ideally you should have several levels of tests.  At the lowest level
        tests should exercise the functionality of the logic with different inputs
        (both valid and invalid).  At higher levels your tests should emulate the
        way the user would interact with your code and confirm that it still works
        the way you intended.
        One of the most important features of the tests is that it should alert other
        developers when their changes will have an impact on the behavior of your
        module.  For example, if a developer removes a feature that you depend on,
        your test should break so they know that the feature is needed.
        """

        pass
