import sys
import os
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, parent_dir)

import slicer
from slicer.ScriptedLoadableModule import *
import qt
import vtk
import time
from Scripts.Logic.robot_nodes import RobotDescriptionNode, RobotStateNode
from Dependencies.urdf_parser_py.urdf import URDF_continuum


class RobotLoader:
    def __init__(self):
        # Mapping of joints and segments

        self._cached_transforms = {}
        self._last_joint_positions = None
        self._last_segment_waypoints = None
        self._cached_vtk_matrices = {}
        self._rendering_enabled = True
        self.link_model_nodes = {}
        self.disk_model_nodes = {}
        self.transform_nodes = []
        self.root_transform_nodes = {}
        self.robot = None
        self.urdf_dir = None

    def loadRobot(self, urdfFilePath):
        self.robot = self.loadURDF(urdfFilePath)
        self.updateParameterNode(self.robot)
        return self.robot

    def loadURDF(self, urdfFilePath):
        urdfFilePath = os.path.normpath(urdfFilePath)
        robot = URDF_continuum.from_xml_file(urdfFilePath)
        return robot

    

    def updateParameterNode(self, robot):
        # create a parameter node for the robot description
        parameter_node = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLScriptedModuleNode')
        parameter_node.SetName(f"SRVRobotDescriptionNode({robot.name})")
        self.robot_description_node = RobotDescriptionNode(parameter_node)
        # update the robot description node
        self.robot_description_node.robot_name = robot.name
        joint_names = [joint.name for joint in robot.joints]
        self.robot_description_node.joint_names ="\n"+ ', '.join(joint_names)
        link_names = [link.name for link in robot.links]
        self.robot_description_node.link_names = "\n"+ ', '.join(link_names)
        self.robot_description_node.joint_mapping = "\n"+ '\n'.join([f"Joint: {joint.name}, Parent: {joint.parent}, Child: {joint.child}" for joint in robot.joints])
        segment_names = [segment.name for segment in robot.segments]
        self.robot_description_node.segment_names = "\n"+ ', '.join(segment_names)

        # create a parameter node for the joint state
        parameter_node_2 = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLScriptedModuleNode') 
        parameter_node_2.SetName(f"SRVRobotStateNode({robot.name})")
        self.robot_state_node = RobotStateNode(parameter_node_2)
        self.robot_state_node.time_stamp = time.time()
        self.robot_state_node.joint_names = joint_names
        self.robot_state_node.joint_positions = [0.0] * len(joint_names)
        self.robot_state_node.segment_names = segment_names
        self.robot_state_node.AddObserver(vtk.vtkCommand.ModifiedEvent, self.__onStateUpdate)