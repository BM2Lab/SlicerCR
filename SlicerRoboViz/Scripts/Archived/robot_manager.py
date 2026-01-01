import sys
import os
from typing import Any
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
from Scripts.Logic.robot_nodes import RobotDescriptionNode, RobotStateNode
import json
from Scripts.Logic.robot_loader import RobotLoader
from Scripts.Utils.rendering_helper import RenderingHelper
from Scripts.Utils.state_parser import StateParser, WaypointFitter
from Scripts.Utils.math_helper import MathHelper
from scipy.spatial.transform import Rotation as R
from Scripts.Logic.transform_manager import TransformManager

class RobotVisualizer:

    CONVERSION_SCALE = 1000
    Euler_ANGLE_ORDER = 'xyz'
    def __init__(self):
        self.robot_loader = RobotLoader()
        self.transform_manager = TransformManager(self)
        # Add performance optimization attributes
        self._cached_transforms = {}
        self._last_joint_positions = None
        self._last_segment_waypoints = None
        self._cached_vtk_matrices = {}
        self._rendering_enabled = True
        self.link_model_nodes = {}
        self.disk_model_nodes = {}
        self.robot = None
        self.urdf_dir = None
        ############
        self.default_segment_direction = np.array([0, 0, 1])
        self.default_u_new = np.array([1])
        self.default_mesh_direction = np.array([0, 0, 1])
        ############
        
        self.rendering_helper = RenderingHelper()
        self.state_parser = StateParser()
        self.waypoint_fitter = WaypointFitter()
        # Initialize the parameter node
        

    def visualizeRobot(self, urdfFilePath):
        self.urdf_dir = os.path.dirname(urdfFilePath)
        self.robot_loader.loadRobot(urdfFilePath)
        self._renderLinksInSlicer(self.robot_loader.robot)
        self.transform_manager.setupTransformHierarchy()
        self.transform_manager.modifyTransformHierarchy()
        self._renderContinuumBodyInSlicer(self.robot_loader.robot, urdfFilePath)


        
    def _renderLinksInSlicer(self, robot):
        """Render the links in the slicer scene"""

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


    def findJointbyChild(self, child_name):
        """Find joint that has the specified child link"""
        for joint_name, joint_data in self.joint_mapping.items():
            if joint_data["child"] == child_name:
                return joint_name, joint_data
        return None, None
    

    
    def __getSegmentInitialTransform(self, length):
        # rot is identity, trans is [0,0,length]
        vtk_matrix = vtk.vtkMatrix4x4()
        vtk_matrix.SetElement(0, 0, 1)
        vtk_matrix.SetElement(1, 1, 1)
        vtk_matrix.SetElement(2, 2, 1)
        vtk_matrix.SetElement(2, 3, length*self.CONVERSION_SCALE)
        vtk_matrix.SetElement(3, 3, 1)
        return vtk_matrix
    

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
            lengths = [self.segment_mapping[segment.name]["initial_length"]*self.CONVERSION_SCALE for segment in robot.segments]
            waypoint_data = self.state_parser.initializeWaypointData(lengths)
            self.updateSegmentState(waypoint_data)

    def getTransformsHierarchy(self):
        # get the key transform nodes trough joint names
        transforms_hierarchy = {}
        # assign the first name and value of root_transform_nodes to the transforms_hierarchy
        transforms_hierarchy[list(self.root_transform_nodes.keys())[0]] = list(self.root_transform_nodes.values())[0]
        for jointName, jointData in self.joint_mapping.items():
            joint_transform_node = jointData["transform_node"]
            if joint_transform_node:
                transforms_hierarchy[jointName] = joint_transform_node
        for segment_name, segment_data in self.segment_mapping.items():
            segment_transform_node_end = segment_data["transform_node(end)"]
            segment_transform_node_start = segment_data["transform_node(start)"]
            if segment_transform_node_end:
                transforms_hierarchy[segment_name] = {"end": segment_transform_node_end}
            if segment_transform_node_start:
                if segment_name in transforms_hierarchy:
                    transforms_hierarchy[segment_name]["start"] = segment_transform_node_start
                else:
                    transforms_hierarchy[segment_name] = {"start": segment_transform_node_start}
        return transforms_hierarchy
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

    def updateSegmentState(self, backbone_waypoints: np.ndarray, end_transforms: np.ndarray=None):
        if backbone_waypoints.shape[0] != len(self.robot_state_node.segment_names):
            # qt.QMessageBox.critical(None, "Error", "Backbone waypoints length does not match segment names length.")
            print(f"Backbone waypoints length does not match segment names length. {backbone_waypoints.shape[0]} != {len(self.robot_state_node.segment_names)}")
            return
        self.robot_state_node.old_segment_waypoints = self.robot_state_node.segment_waypoints
        self.robot_state_node.segment_waypoints = str(backbone_waypoints.tolist())
        self.robot_state_node.segment_end_transforms = str(end_transforms.tolist()) if end_transforms is not None else ""
        self.robot_state_node.time_stamp = time.time()

    def clearCache(self):
        """Clear all cached objects to free memory"""
        self._cached_transforms.clear()
        self._cached_vtk_matrices.clear()
        self._last_joint_positions = None
        self._last_segment_waypoints = None

    def __onStateUpdate(self, caller, event):
        # start_time = time.time()
        
        # Cache frequently accessed values
        joint_positions = self.robot_state_node.joint_positions
        old_joint_positions = self.robot_state_node.old_joint_positions
        
        # Only update joints if positions changed
        if old_joint_positions != joint_positions:
            # Pre-allocate transform objects to avoid repeated creation

            for joint_name, position in zip(self.robot_state_node.joint_names, joint_positions):
                joint_data = self.joint_mapping[joint_name]
                transformNode = joint_data["transform_node"]
                
                if not transformNode:
                    print(f"Joint {joint_name} not found")
                    continue

                jointType = joint_data["type"]
                axis = joint_data["axis"]
                initial_transform_matrix = joint_data["initial_transform"]
                
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
        
        # Optimize segment waypoint updates
        old_segment_waypoints = self.robot_state_node.old_segment_waypoints
        segment_waypoints = self.robot_state_node.segment_waypoints

        if old_joint_positions != joint_positions or old_segment_waypoints != segment_waypoints:
            # backbone_waypoints = np.array(segment_waypoints)
            backbone_waypoints = MathHelper.string2Array(segment_waypoints)
            segment_end_transforms = MathHelper.string2Array(self.robot_state_node.segment_end_transforms)
            
            # Pre-compute common values
            segments = self.robot.segments
            segment_count = len(segments)
            
            # Temporarily disable rendering for batch updates
            original_rendering_state = self._rendering_enabled
            self._rendering_enabled = False
            
            try:
                # Batch process segments
                for i in range(segment_count):
                    segment = segments[i]
                    segment_data = self.segment_mapping[segment.name]
                    start_transform_node = segment_data["transform_node(start)"]
                    # Handle end transforms
                    
                    if  segment_end_transforms is None:
                        _, _, end_poses = self.waypoint_fitter.getIntermediatePoses(
                            self.default_segment_direction, self.Euler_ANGLE_ORDER, backbone_waypoints[i], self.default_u_new
                        )
                        end_pose = MathHelper.npMatrixToVtkMatrix(end_poses[0])
                        self.segment_mapping[segment.name]["transform_node(end)"].SetMatrixTransformToParent(end_pose)
                    else:
                        end_pose = MathHelper.npMatrixToVtkMatrix(np.array(segment_end_transforms[i].squeeze()))
                        self.segment_mapping[segment.name]["transform_node(end)"].SetMatrixTransformToParent(end_pose)
                    
                    # Optimize mesh disk rendering
                    if segment.disks and segment.disks.geometry.type == 'mesh':
                        self._updateMeshDisks(segment, backbone_waypoints[i], start_transform_node)

                    # Transform waypoints to world coordinates
                    vtk_matrix_world = vtk.vtkMatrix4x4()
                    start_transform_node.GetMatrixTransformToWorld(vtk_matrix_world)
                    backbone_waypoints[i] = MathHelper.transformWaypoints(backbone_waypoints[i], vtk_matrix_world)

                    # Update continuum units
                    self._updateContinuumUnits(segment, vtk_matrix_world, backbone_waypoints[i])
                    
                    # Handle cylinder disks
                    if segment.disks and segment.disks.geometry.type == 'cylinder':
                        self._updateCylinderDisks(segment, backbone_waypoints[i])
            finally:
                self.robot_state_node.segment_global_waypoints = str(backbone_waypoints.tolist())
                self._rendering_enabled = original_rendering_state

            if self._rendering_enabled:
                self.rendering_helper.show(self.robot)



    def _updateMeshDisks(self, segment, backbone_waypoint, start_transform_node):
        """Optimized mesh disk update"""
        
        disk_span = segment.disks.span or [0, 1]
        u_new = np.linspace(disk_span[0], disk_span[1], segment.disks.count)
        
        disk_centers, disk_directions, transform_matrices = self.waypoint_fitter.getIntermediatePoses(
            self.default_mesh_direction, self.Euler_ANGLE_ORDER, backbone_waypoint, u_new
        )
        
        for j in range(segment.disks.count):
            model_name = f"{segment.name}_disk_{j}"
            
            if model_name in self.disk_model_nodes:

                disk_model_node_transform_node = self.disk_model_nodes[model_name+"_transform_node"]
                transform_matrix = transform_matrices[j]
                transform_matrix[:3,:3] = transform_matrix[:3,:3]*self.CONVERSION_SCALE
                disk_model_node_transform_node.SetMatrixTransformToParent(MathHelper.npMatrixToVtkMatrix(transform_matrix))
            else:
                # Create new model
                meshFilePath = os.path.normpath(os.path.join(self.urdf_dir, segment.disks.geometry.filename))
                disk_model_node, disk_model_node_transform_node = self._renderMeshInSlicer(
                    meshFilePath, model_name, disk_centers[j], disk_directions[j], 
                    segment.disks.color.rgba, scale=self.CONVERSION_SCALE
                )
                disk_model_node_transform_node.SetAndObserveTransformNodeID(start_transform_node.GetID())
                self.disk_model_nodes[model_name] = disk_model_node
                self.disk_model_nodes[model_name+"_transform_node"] = disk_model_node_transform_node

    def _updateContinuumUnits(self, segment, vtk_matrix_world, backbone_waypoint):
        """Optimized continuum unit update"""

        configs = []
        for unit in segment.continuum_body.continuum_units:
            if unit.offset is None or unit.angle is None:
                config = [0, 0]
            else:
                config = [unit.offset*self.CONVERSION_SCALE, unit.angle]
            configs.append(config)
        
        waypoints = self.waypoint_fitter.getContinuumUnitWaypoints(vtk_matrix_world, backbone_waypoint, configs)
        for unit, waypoint in zip(segment.continuum_body.continuum_units, waypoints):
            unit.trajectory = waypoint


    def _updateCylinderDisks(self, segment, backbone_waypoint):
        """Optimized cylinder disk update"""
        start_time = time.time()
        disk_count = segment.disks.count
        disk_span = segment.disks.span or [0, 1]
        u_new = np.linspace(disk_span[0], disk_span[1], disk_count)
        default_vtk_cylinder_direction = np.array([0, 1, 0])
        disk_centers, disk_directions, _ = self.waypoint_fitter.getIntermediatePoses(
            default_vtk_cylinder_direction, 'ZXY', backbone_waypoint, u_new
        )
        segment.disks.centers = disk_centers
        segment.disks.directions = disk_directions
        end_time = time.time()
        # print(f"Time spent in _updateCylinderDisks: {(end_time - start_time)*1000:.2f} ms")     
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
        return MathHelper.string2Array(self.robot_state_node.segment_global_waypoints)
    
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
        for transformNode in self.transform_nodes:
            if transformNode:
                slicer.mrmlScene.RemoveNode(transformNode)
        for disk_model_node in self.disk_model_nodes.values():
            if disk_model_node:
                slicer.mrmlScene.RemoveNode(disk_model_node)
        self.joint_mapping.clear()
        self.link_model_nodes.clear()
        self.disk_model_nodes.clear()
        self.root_transform_nodes.clear()
        self.transform_nodes.clear()
        self.segment_mapping.clear()
        self.robot = None
        self.robot_description_node = None
        self.robot_state_node = None
        self.rendering_helper.clear()
        print("RobotVisualizer cleanup completed.")
    
