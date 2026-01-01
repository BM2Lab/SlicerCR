import sys
import os
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, parent_dir)

import slicer
from slicer.ScriptedLoadableModule import *
import qt
import vtk

from Dependencies.urdf_parser_py.urdf import URDF, URDF_continuum
import numpy as np
import csv
import time
import threading
from slicer.parameterNodeWrapper import (
    parameterNodeWrapper,
    WithinRange,
)
import json
from Scripts.Logic.robot_nodes import RobotDescriptionNode, RobotStateNode
from Scripts.Utils.rendering_manager import RenderingManager
from Scripts.Utils.segment_constructor import StateParser, WaypointFitter
from Scripts.Utils.math_helper import MathHelper
from scipy.spatial.transform import Rotation as R
from Scripts.Logic.kinematics_manager import KinematicsManager



class RobotVisualizer:

    CONVERSION_SCALE = 1000
    Euler_ANGLE_ORDER = 'xyz'
    def __init__(self):
        self.kinematics_manager = KinematicsManager(self)

        # Add performance optimization attributes
        self._cached_transforms = {}
        self._last_joint_positions = None
        self._last_segment_SPs = None
        self._cached_vtk_matrices = {}
        self._rendering_enabled = True
        self.link_model_nodes = {}
        self.segment_model_nodes = {}
        self.vertebra_model_nodes = {}
        self.robot = None
        self.urdf_dir = None
        ############
        self.default_segment_direction = np.array([0, 0, 1])
        self.default_u_new = np.array([1])
        self.default_mesh_direction = np.array([0, 0, 1])
        self.vtk_matrix_base = vtk.vtkMatrix4x4()
        self.vtk_matrix_base.Identity()
        ############

        self.rendering_manager = RenderingManager()
        self.state_parser = StateParser()
        self.SP_fitter = WaypointFitter()
        # Initialize the parameter node
        

    def visualizeRobot(self, urdfFilePath):
        self.robot = self.loadURDF(urdfFilePath)
        if not self.robot:
            qt.QMessageBox.critical(None, "Error", f"Failed to load URDF: {urdfFilePath}")
            return
        self._updateParameterNode(self.robot)
        self._renderLinksInSlicer(self.robot, urdfFilePath)
        self.kinematics_manager.setupTransformHierarchy()
        self._renderContinuumBodyInSlicer(self.robot, urdfFilePath)


    def loadURDF(self, urdfFilePath):
        urdfFilePath = os.path.normpath(urdfFilePath)
        robot = URDF_continuum.from_xml_file(urdfFilePath)
        return robot

    
    def _updateParameterNode(self, robot):
        # create a parameter node for the robot description
        parameter_node = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLScriptedModuleNode')
        parameter_node.SetName(f"SRVRobotDescriptionNode({robot.name})")
        self.robot_description_node = RobotDescriptionNode(parameter_node)
        self.robot_description_node.robot_name = robot.name
        joint_names = [joint.name for joint in robot.joints]
        self.robot_description_node.joint_names ="\n"+ ', '.join(joint_names)
        link_names = [link.name for link in robot.links]
        self.robot_description_node.link_names = "\n"+ ', '.join(link_names)
        self.robot_description_node.joint_mapping = "\n"+ '\n'.join([f"Joint: {joint.name}, Parent: {joint.parent}, Child: {joint.child}" for joint in robot.joints])
        
        segment_names = [segment.name for segment in robot.segments]
        self.robot_description_node.segment_names = "\n"+ ', '.join(segment_names)

        # create a parameter node for the joint state
        parameter_node2 = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLScriptedModuleNode') 
        parameter_node2.SetName(f"SRVRobotStateNode({robot.name})")
        self.robot_state_node = RobotStateNode(parameter_node2)
        self.robot_state_node.time_stamp = time.time()
        self.robot_state_node.joint_names = joint_names
        self.robot_state_node.joint_positions = [0.0] * len(joint_names)
        self.robot_state_node.segment_names = segment_names
        self.robot_state_node.AddObserver(vtk.vtkCommand.ModifiedEvent, self.__onStateUpdate)

        
    def _renderLinksInSlicer(self, robot, urdfFilePath):
        self.urdf_dir = os.path.dirname(urdfFilePath)
        self.link_model_nodes.clear()

        for link in robot.links:
            if not link.visual or not link.visual.geometry or not link.visual.geometry.filename:
                print(f"Skipping link {link.name}: No visual geometry defined.")
                continue

            meshFilePath = os.path.normpath(os.path.join(self.urdf_dir, link.visual.geometry.filename))
            if not os.path.exists(meshFilePath):
                qt.QMessageBox.critical(None, "Error", f"Mesh file not found: {meshFilePath}")
                continue

            position = link.visual.origin.xyz if link.visual.origin else [0, 0, 0]
            orientation = link.visual.origin.rpy if link.visual.origin else [0, 0, 0]
            color = link.visual.material.color.rgba if link.visual.material and link.visual.material.color else [1, 1, 1, 1]
            modelNode,_ = self._renderMeshInSlicer(meshFilePath, link.name, position, orientation, color, scale= self.CONVERSION_SCALE)
            
            self.link_model_nodes[link.name] = modelNode

    

    def _renderMeshInSlicer(self, meshFilePath,model_name, position, orientation, color, scale=None):
        modelNode = slicer.modules.models.logic().AddModel(meshFilePath)
        if not modelNode:
            qt.QMessageBox.critical(None, "Error", f"Failed to load mesh")
            return None
        # Set the color
        modelNode.GetDisplayNode().SetColor(color[0], color[1], color[2])
        modelNode.GetDisplayNode().SetOpacity(color[3])
        # Set the visual transform
        transform = vtk.vtkTransform()
        transform.Translate(position)
        transform.RotateZ(np.degrees(orientation[2])) # intrinsic rotation
        transform.RotateY(np.degrees(orientation[1]))
        transform.RotateX(np.degrees(orientation[0]))
        if scale:
            transform.Scale([scale,scale,scale])
        transformNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLTransformNode",f"{model_name}_visual_Transform")
        transformNode.SetMatrixTransformToParent(transform.GetMatrix())
        modelNode.SetAndObserveTransformNodeID(transformNode.GetID())
        return modelNode, transformNode

    def _renderContinuumBodyInSlicer(self, robot, urdfFilePath):
        if robot.segments:
            # lengths = [self.segment_mapping[segment.name]["initial_length"]*self.CONVERSION_SCALE for segment in robot.segments]
            lengths = [segment.continuum_body.initial_length*self.CONVERSION_SCALE for segment in robot.segments]
            SP_data = self.state_parser.initializeWaypointData(lengths)
            was_modified = self.robot_state_node.StartModify()
            self.updateSegmentState(SP_data)
            self.robot_state_node.EndModify(was_modified)


    def getTransformsHierarchy(self):
        return self.kinematics_manager.getTransformsHierarchy()

    #####################################
    ####### Robot Motion ################
    #####################################
    def updateJointState(self, joint_positions: list[float]):
        if len(joint_positions) != len(self.robot_state_node.joint_names):
            # qt.QMessageBox.critical(None, "Error", "Joint positions length does not match joint names length.")
            print(f"Joint positions length does not match joint names length. {len(joint_positions)} != {len(self.robot_state_node.joint_names)}")
            return
        self.robot_state_node.old_joint_positions = self.robot_state_node.joint_positions
        self.robot_state_node.joint_positions = joint_positions
        self.robot_state_node.time_stamp = time.time()
        # print("updateJointState called")

    def updateSegmentState(self, backbone_SPs: np.ndarray, end_transforms: np.ndarray=None):
        if backbone_SPs.shape[0] != len(self.robot_state_node.segment_names):
            # qt.QMessageBox.critical(None, "Error", "Backbone sample points length does not match segment names length.")
            print(f"Backbone sample points length does not match segment names length. {backbone_SPs.shape[0]} != {len(self.robot_state_node.segment_names)}")
            return
        self.robot_state_node.old_segment_SPs = self.robot_state_node.segment_SPs
        self.robot_state_node.segment_SPs = str(backbone_SPs.tolist())
        self.robot_state_node.segment_end_transforms = str(end_transforms.tolist()) if end_transforms is not None else ""
        self.robot_state_node.time_stamp = time.time()
        # print("updateSegmentState called")

    def __onStateUpdate(self, caller, event):
        """Update the robot rendering once the robot state is updated."""
        # Cache frequently accessed values
        joint_positions = self.robot_state_node.joint_positions
        old_joint_positions = self.robot_state_node.old_joint_positions
        
        # Only update joints if positions changed
        if old_joint_positions != joint_positions:
            # Pre-allocate transform objects to avoid repeated creation
            for idx_joint in range(len(self.robot_state_node.joint_names)):
                joint_name = self.robot_state_node.joint_names[idx_joint]
                position = joint_positions[idx_joint]
                transformNode = self.kinematics_manager.joint_transform_container[joint_name]["transform_node"]
                
                if not transformNode:
                    print(f"Joint {joint_name} not found")
                    continue

                jointType = self.robot.joints[idx_joint].type
                axis = self.robot.joints[idx_joint].axis if self.robot.joints[idx_joint].axis else [0,0,1]
                initial_transform_matrix = self.kinematics_manager.joint_transform_container[joint_name]["initial_transform"]
                
                # Reuse transform object if available
                if joint_name not in self._cached_transforms:
                    self._cached_transforms[joint_name] = vtk.vtkTransform()
                
                transform = self._cached_transforms[joint_name]
                transform.SetMatrix(initial_transform_matrix)
                
                # Apply joint transformation
                if jointType == "revolute":
                    angle_deg = np.degrees(position)
                    transform.RotateWXYZ(angle_deg, -axis[0], -axis[1], axis[2])
                elif jointType == "prismatic":
                    scale_factor = position * self.CONVERSION_SCALE
                    transform.Translate(-axis[0] * scale_factor, -axis[1] * scale_factor, axis[2] * scale_factor)

                transformNode.SetMatrixTransformToParent(transform.GetMatrix())
        # print(f"Time spent in __onStateUpdate joint part: {(time.time() - start_time)*1000:.2f} ms")
        # Optimize segment sample point updates
        old_segment_SPs = self.robot_state_node.old_segment_SPs
        segment_SPs = self.robot_state_node.segment_SPs

        if old_joint_positions != joint_positions or old_segment_SPs != segment_SPs:
            
            backbone_SPs = MathHelper.string2Array(segment_SPs)
            segment_end_transforms = MathHelper.string2Array(self.robot_state_node.segment_end_transforms)

            try:
                # Batch process segments
                for i, segment in enumerate(self.robot.segments):
                    if not self.segment_model_nodes.get(segment.name): # initialize the segment model nodes
                        self.segment_model_nodes[segment.name] = []
                        print(f"Initializing segment model nodes for {segment.name}")
                        for idx_unit,unit in enumerate(segment.continuum_body.continuum_units):
                            model_node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLModelNode", f"{segment.name}_continuum_unit_{idx_unit}")
                            model_node.SetAndObserveTransformNodeID(self.kinematics_manager.segment_transform_container[segment.name]["transform_node(start)"].GetID())
                            model_node.CreateDefaultDisplayNodes()
                            model_node.GetDisplayNode().SetColor(unit.color.rgba[0], unit.color.rgba[1], unit.color.rgba[2])
                            model_node.GetDisplayNode().SetOpacity(unit.color.rgba[3])
                            self.segment_model_nodes[segment.name].append(model_node)
                    start_transform_node = self.kinematics_manager.segment_transform_container[segment.name]["transform_node(start)"]
                    # Handle end transforms
                    
                    if  segment_end_transforms is None:
                        _, _, end_poses = self.SP_fitter.getIntermediatePoses(
                            self.default_segment_direction, self.Euler_ANGLE_ORDER, backbone_SPs[i], self.default_u_new
                        )
                        end_pose = MathHelper.npMatrixToVtkMatrix(end_poses[0])
                        self.kinematics_manager.segment_transform_container[segment.name]["transform_node(end)"].SetMatrixTransformToParent(end_pose)
                    else:
                        end_pose = MathHelper.npMatrixToVtkMatrix(np.array(segment_end_transforms[i].squeeze()))
                        self.kinematics_manager.segment_transform_container[segment.name]["transform_node(end)"].SetMatrixTransformToParent(end_pose)
                    
                    
                    if segment.vertebrae and segment.vertebrae.geometry.type == 'mesh':
                        self._updateVertebrae(segment, backbone_SPs[i], start_transform_node)
                    
                    self._updateContinuumUnits(segment, self.vtk_matrix_base, backbone_SPs[i])

            finally:
                
                # self.rendering_manager.show(self.robot, self.segment_mapping)
                self.rendering_manager.show(self.robot, self.segment_model_nodes)


    def _updateVertebrae(self, segment, backbone_SPs, start_transform_node):
        """Optimized vertebrae update"""
        
        vertebra_span = segment.vertebrae.span or [0, 1]
        u_new = np.linspace(vertebra_span[0], vertebra_span[1], segment.vertebrae.count)
        
        vertebra_centers, vertebra_directions, transform_matrices = self.SP_fitter.getIntermediatePoses(
            self.default_mesh_direction, self.Euler_ANGLE_ORDER, backbone_SPs, u_new
        )
        
        for j in range(segment.vertebrae.count):
            model_name = f"{segment.name}_vertebra_{j}"
            
            if model_name in self.vertebra_model_nodes:

                vertebra_model_node_transform_node = self.vertebra_model_nodes[model_name+"_transform_node"]
                transform_matrix = transform_matrices[j]
                transform_matrix[:3,:3] = transform_matrix[:3,:3]*self.CONVERSION_SCALE
                vertebra_model_node_transform_node.SetMatrixTransformToParent(MathHelper.npMatrixToVtkMatrix(transform_matrix))
            else:
                # Create new model
                meshFilePath = os.path.normpath(os.path.join(self.urdf_dir, segment.vertebrae.geometry.filename))
                vertebra_model_node, vertebra_model_node_transform_node = self._renderMeshInSlicer(
                    meshFilePath, model_name, vertebra_centers[j], vertebra_directions[j], 
                    segment.vertebrae.color.rgba, scale=self.CONVERSION_SCALE
                )
                vertebra_model_node_transform_node.SetAndObserveTransformNodeID(start_transform_node.GetID())
                self.vertebra_model_nodes[model_name] = vertebra_model_node
                self.vertebra_model_nodes[model_name+"_transform_node"] = vertebra_model_node_transform_node

    def _updateContinuumUnits(self, segment, vtk_matrix_world, backbone_SPs):
        """Optimized continuum unit update"""

        configs = []
        for unit in segment.continuum_body.continuum_units:
            if unit.offset is None or unit.angle is None:
                config = [0, 0]
            else:
                config = [unit.offset*self.CONVERSION_SCALE, unit.angle]
            configs.append(config)
        
        unit_SPs = self.SP_fitter.getContinuumUnitWaypoints(vtk_matrix_world, backbone_SPs, configs)
        for unit, SPs in zip(segment.continuum_body.continuum_units, unit_SPs):
            unit.trajectory = SPs


    #####################################
    ####### attribute access #############
    #####################################

    @property
    def robot_name(self):
        return self.robot_description_node.robot_name
    
    @property
    def joint_names(self):
        return self.robot_description_node.joint_names

    @property
    def segment_global_waypoints(self):
        return None
    
    ######################################
    ############## Cleanup ###############
    ######################################

    def cleanup(self):
        """Cleanup method to remove all nodes and data created by the visualizer."""
        if self.robot_description_node:
            slicer.mrmlScene.RemoveNode(self.robot_description_node.parameterNode)
        if self.robot_state_node:
            slicer.mrmlScene.RemoveNode(self.robot_state_node.parameterNode)
        for link_name, model_node in self.link_model_nodes.items():
            if model_node:
                slicer.mrmlScene.RemoveNode(model_node)

        for vertebra_model_node in self.vertebra_model_nodes.values():
            if vertebra_model_node:
                slicer.mrmlScene.RemoveNode(vertebra_model_node)
       
        for segment_name, model_nodes in self.segment_model_nodes.items():
            for model_node in model_nodes:
                if model_node:
                    slicer.mrmlScene.RemoveNode(model_node)
        self.segment_model_nodes={}
        self.kinematics_manager.cleanUp()
        self.link_model_nodes.clear()
        self.vertebra_model_nodes.clear()
        self.robot = None
        self.robot_description_node = None
        self.robot_state_node = None
        print("RobotVisualizer cleanup completed.")
    
