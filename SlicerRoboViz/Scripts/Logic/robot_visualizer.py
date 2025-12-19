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
from Scripts.Utils.rendering_manager import RenderingManager
from Scripts.Utils.state_parser import StateParser, WaypointFitter
from Scripts.Utils.math_helper import MathHelper
from scipy.spatial.transform import Rotation as R

@parameterNodeWrapper
class RobotDescriptionNode:
    # Basic robot info
    robot_name: str = ""
    joint_names: str = ""
    link_names: str = ""
    segment_names: str = ""
    joint_mapping: str = ""

@parameterNodeWrapper
class RobotStateNode:
    """Parameter node wrapper for joint states."""
    time_stamp: float = 0.0
    joint_names: list[str] = []
    joint_positions: list[float] = []
    old_joint_positions: list[float] = []
    segment_names: list[str] = []
    segment_SPs: str = "" # sample points
    old_segment_SPs: str = ""
    segment_end_transforms: str = ""

class RobotVisualizer:

    CONVERSION_SCALE = 1000
    Euler_ANGLE_ORDER = 'xyz'
    def __init__(self):
        self.joint_mapping = {}
        # Add performance optimization attributes
        self._cached_transforms = {}
        self._last_joint_positions = None
        self._last_segment_SPs = None
        self._cached_vtk_matrices = {}
        self._rendering_enabled = True
        self.link_model_nodes = {}
        self.vertebra_model_nodes = {}
        self.transform_nodes = []
        self.root_transform_nodes = {}
        self.robot = None
        self.urdf_dir = None
        ############
        self.default_segment_direction = np.array([0, 0, 1])
        self.default_u_new = np.array([1])
        self.default_mesh_direction = np.array([0, 0, 1])
        self.vtk_matrix_base = vtk.vtkMatrix4x4()
        self.vtk_matrix_base.Identity()
        ############
        self.segment_mapping = {}
        self.rendering_manager = RenderingManager()
        self.state_parser = StateParser()
        self.SP_fitter = WaypointFitter()
        # Initialize the parameter node
        

    def visualizeRobot(self, urdfFilePath):
        robot = self.loadURDF(urdfFilePath)
        if not robot:
            qt.QMessageBox.critical(None, "Error", f"Failed to load URDF: {urdfFilePath}")
            return
        self._updateParameterNode(robot)
        self._renderLinksInSlicer(robot, urdfFilePath)
        self._setupTransformHierarchy()
        self._ModifyTransformHierarchy()
        self._renderContinuumBodyInSlicer(robot, urdfFilePath)


    def loadURDF(self, urdfFilePath):
        urdfFilePath = os.path.normpath(urdfFilePath)
        robot = URDF_continuum.from_xml_file(urdfFilePath)
        self.robot = robot

        for joint in robot.joints:
            self.joint_mapping[joint.name] = {
                "type": joint.joint_type,
                "axis": joint.axis if joint.axis else [0, 0, 0],
                "limit": joint.limit,
                "parent": joint.parent,
                "child": joint.child,
                "origin": joint.origin,
                "transform_node": None,
                "initial_transform": None
            }
        for segment in robot.segments:
            self.segment_mapping[segment.name] = {
                "parent": segment.parent,
                "origin": segment.origin,
                "initial_length": segment.continuum_body.initial_length,
                "transform_node": None,
                "transform_node(start)": None,
                "transform_node(end)": None,
                "model_nodes": [],
                "initial_transform": None
            }
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
        self.robot_description_node.joint_mapping = "\n"+ '\n'.join([f"Joint: {jointName}, Parent: {jointData['parent']}, Child: {jointData['child']}" for jointName, jointData in self.joint_mapping.items()])
        
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


    def findJointbyChild(self, child_name):
        """Find joint that has the specified child link"""
        for joint_name, joint_data in self.joint_mapping.items():
            if joint_data["child"] == child_name:
                return joint_name, joint_data
        return None, None
    
    def __getInitialTransformFromOrigin(self, xyz, rpy):
        # Create VTK transform and apply rotation and translation

        scaled_xyz = [x * self.CONVERSION_SCALE for x in xyz]
        vtk_matrix = vtk.vtkMatrix4x4()
        rot = R.from_euler(self.Euler_ANGLE_ORDER, rpy, degrees=False) # extrinsic rotation the inverse order of intrinsic rotation: Z to X to Y
        for i in range(3):
            for j in range(3):
                vtk_matrix.SetElement(i, j, rot.as_matrix()[i, j])
        vtk_matrix.SetElement(0, 3, scaled_xyz[0])
        vtk_matrix.SetElement(1, 3, scaled_xyz[1])
        vtk_matrix.SetElement(2, 3, scaled_xyz[2])
        return vtk_matrix
    
    def __getSegmentInitialTransform(self, length):
        # rot is identity, trans is [0,0,length]
        vtk_matrix = vtk.vtkMatrix4x4()
        vtk_matrix.SetElement(0, 0, 1)
        vtk_matrix.SetElement(1, 1, 1)
        vtk_matrix.SetElement(2, 2, 1)
        vtk_matrix.SetElement(2, 3, length*self.CONVERSION_SCALE)
        vtk_matrix.SetElement(3, 3, 1)
        return vtk_matrix
    
    def _setupTransformHierarchy(self):

        for jointName, jointData in self.joint_mapping.items():
            childLinkName = jointData["child"]
            parentLinkName = jointData["parent"]
            # initialize the transform node (origin has a transform)
            initial_transform = self.__getInitialTransformFromOrigin(jointData["origin"].xyz, jointData["origin"].rpy)
            transformNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLTransformNode", f"{jointName}_Transform")
            transformNode.SetMatrixTransformToParent(initial_transform)
            jointData["transform_node"] = transformNode
            jointData["initial_transform"] = initial_transform
            # attach the visual transform to the joint transform
            if childLinkName in self.link_model_nodes:
                childModelNode = self.link_model_nodes[childLinkName]
                visual_transform_node = childModelNode.GetParentTransformNode()
                if visual_transform_node:
                    visual_transform_node.SetAndObserveTransformNodeID(transformNode.GetID())
                    print(f"Attached {childLinkName} visual to {jointName}_Transform")
            else:
                print(f"ERROR: Could not find model node for link: {childLinkName}")

            parent_joint_name,_ = self.findJointbyChild(parentLinkName)
            if  parent_joint_name: 
                parentTransformNode = self.joint_mapping[parent_joint_name]["transform_node"]
                if parentTransformNode:
                    transformNode.SetAndObserveTransformNodeID(parentTransformNode.GetID())
                    print(f"Attached {jointName}_Transform to {parentLinkName}_Transform")
                else:
                    print(f"Parent transform for {parentLinkName} not found")
            else: # no parent joint, set as the root transform
                #  find the parent root transform node first if not found, create a new one
                rootTransformNode = slicer.mrmlScene.GetFirstNodeByName(f"{self.robot_name}_{parentLinkName}_Transform(root)") #TODO: if two robots has the same parent link name, this will be wrong
                if not rootTransformNode:   
                    rootTransformNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLTransformNode", f"{self.robot_name}_{parentLinkName}_Transform(root)")
                    self.root_transform_nodes[parentLinkName] = rootTransformNode
                
                if parentLinkName in self.link_model_nodes:
                    visual_transform_node = self.link_model_nodes[parentLinkName].GetParentTransformNode()
                    if visual_transform_node:
                        visual_transform_node.SetAndObserveTransformNodeID(rootTransformNode.GetID())
                transformNode.SetAndObserveTransformNodeID(rootTransformNode.GetID())
                print(f"Set {parentLinkName} as root")

        for jointName, jointData in self.joint_mapping.items(): # in case the parent joint is not defined at the beginning
            childLinkName = jointData["child"]
            parentLinkName = jointData["parent"]
            transformNode = jointData["transform_node"]
            parent_joint_name,_ = self.findJointbyChild(parentLinkName)
            if  parent_joint_name: 
                parentTransformNode = self.joint_mapping[parent_joint_name]["transform_node"]
                if parentTransformNode:
                    transformNode.SetAndObserveTransformNodeID(parentTransformNode.GetID())

        
        for segment_name, segment_data in self.segment_mapping.items():
            parent_segment_name = segment_data["parent"]
            # child_segment_name = segment_data["child"] #TODO: child looks redundant
            if segment_data["origin"]:
                initial_transform = self.__getInitialTransformFromOrigin(segment_data["origin"].xyz, segment_data["origin"].rpy)
            else:
                initial_transform = self.__getInitialTransformFromOrigin([0,0,0],[0,0,0])
            self.segment_mapping[segment_name]["initial_transform"] = initial_transform # TODO: check its use later
            start_transformNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLTransformNode", f"{segment_name}_Transform(start)")
            start_transformNode.SetMatrixTransformToParent(initial_transform)
            self.segment_mapping[segment_name]["transform_node(start)"] = start_transformNode

            end_transformNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLTransformNode", f"{segment_name}_Transform(end)")
            self.segment_mapping[segment_name]["transform_node(end)"] = end_transformNode
            end_transformNode.SetAndObserveTransformNodeID(start_transformNode.GetID())

            if parent_segment_name in self.segment_mapping:
                parent_end_transform_node = self.segment_mapping[parent_segment_name]["transform_node(end)"]
                if parent_end_transform_node:
                    start_transformNode.SetAndObserveTransformNodeID(parent_end_transform_node.GetID())
                    print(f"Attached {segment_name}_Transform to {parent_segment_name}_Transform")
            else: # set as the root transform
                # check if the root transform node already exists
                if parent_segment_name in self.root_transform_nodes:
                    rootTransformNode = self.root_transform_nodes[parent_segment_name]
                    start_transformNode.SetAndObserveTransformNodeID(rootTransformNode.GetID())
                else:
                    rootTransformNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLTransformNode", f"{parent_segment_name}_Transform(root)")
                    self.root_transform_nodes[parent_segment_name] = rootTransformNode
                    start_transformNode.SetAndObserveTransformNodeID(rootTransformNode.GetID())
                    print(f"Set {parent_segment_name} as root")

        for segment_name, segment_data in self.segment_mapping.items(): # incase the parent segment is not defined at the beginning
            parent_segment_name = segment_data["parent"]
            start_transformNode = segment_data["transform_node(start)"]
            if parent_segment_name in self.segment_mapping:
                parent_end_transform_node = self.segment_mapping[parent_segment_name]["transform_node(end)"]
                if parent_end_transform_node:
                    start_transformNode.SetAndObserveTransformNodeID(parent_end_transform_node.GetID())
                    print(f"Attached {segment_name}_Transform to {parent_segment_name}_Transform")

    def _ModifyTransformHierarchy(self):
        # use the root transform node and IsTransformNodeMyChild to find the hierarchy
        # connect seperated transform nodes
        for parent_name, transform_node in list(self.root_transform_nodes.items()):
            if parent_name in self.segment_mapping:
                parent_transform_node = self.segment_mapping[parent_name]["transform_node(end)"]
                transform_node.SetAndObserveTransformNodeID(parent_transform_node.GetID())
                self.root_transform_nodes.pop(parent_name)

            if parent_name in self.link_model_nodes:
                joint_name, joint_data = self.findJointbyChild(parent_name)
                if joint_name:
                    joint_transform_node = self.joint_mapping[joint_name]["transform_node"]
                    transform_node.SetAndObserveTransformNodeID(joint_transform_node.GetID())
                    self.root_transform_nodes.pop(parent_name)

        # check all transform nodes in the scene
        all_transform_nodes = slicer.mrmlScene.GetNodesByClass("vtkMRMLTransformNode")
        self.transform_nodes.append(list(self.root_transform_nodes.values())[0])
        for i in range(all_transform_nodes.GetNumberOfItems()):
            transformNode = all_transform_nodes.GetItemAsObject(i)
            if self.transform_nodes[0].IsTransformNodeMyChild(transformNode):
                self.transform_nodes.append(transformNode)


        # convert the initial transform to the slicer transform
        for transformNode in self.transform_nodes:
            initial_matrix = vtk.vtkMatrix4x4()
            transformNode.GetMatrixTransformToParent(initial_matrix)
            converted_matrix = MathHelper.convert2SlicerTransform(initial_matrix)
            transformNode.SetMatrixTransformToParent(converted_matrix)
        # convert the stored initial matrix to the slicer transform
        for jointName, jointData in self.joint_mapping.items():
            initial_matrix = jointData["initial_transform"]
            converted_matrix = MathHelper.convert2SlicerTransform(initial_matrix)
            jointData["initial_transform"] = converted_matrix

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
            SP_data = self.state_parser.initializeWaypointData(lengths)
            was_modified = self.robot_state_node.StartModify()
            self.updateSegmentState(SP_data)
            self.robot_state_node.EndModify(was_modified)

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
        # print("__onStateUpdate called")
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
        # print(f"Time spent in __onStateUpdate joint part: {(time.time() - start_time)*1000:.2f} ms")
        # Optimize segment sample point updates
        old_segment_SPs = self.robot_state_node.old_segment_SPs
        segment_SPs = self.robot_state_node.segment_SPs

        if old_joint_positions != joint_positions or old_segment_SPs != segment_SPs:
            
            backbone_SPs = MathHelper.string2Array(segment_SPs)
            segment_end_transforms = MathHelper.string2Array(self.robot_state_node.segment_end_transforms)
            
            # Pre-compute common values
            segments = self.robot.segments
            segment_count = len(segments)
            try:
                # Batch process segments
                for i in range(segment_count):
                    segment = segments[i]
                    segment_data = self.segment_mapping[segment.name]
                    if len(segment_data["model_nodes"]) == 0:
                        for idx_unit,unit in enumerate(segment.continuum_body.continuum_units):
                            model_node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLModelNode", f"{segment.name}_continuum_unit_{idx_unit}")
                            model_node.SetAndObserveTransformNodeID(segment_data["transform_node(start)"].GetID())
                            model_node.CreateDefaultDisplayNodes()
                            model_node.GetDisplayNode().SetColor(unit.color.rgba[0], unit.color.rgba[1], unit.color.rgba[2])
                            model_node.GetDisplayNode().SetOpacity(unit.color.rgba[3])
                            segment_data["model_nodes"].append(model_node)
                    start_transform_node = segment_data["transform_node(start)"]
                    # Handle end transforms
                    
                    if  segment_end_transforms is None:
                        _, _, end_poses = self.SP_fitter.getIntermediatePoses(
                            self.default_segment_direction, self.Euler_ANGLE_ORDER, backbone_SPs[i], self.default_u_new
                        )
                        end_pose = MathHelper.npMatrixToVtkMatrix(end_poses[0])
                        self.segment_mapping[segment.name]["transform_node(end)"].SetMatrixTransformToParent(end_pose)
                    else:
                        end_pose = MathHelper.npMatrixToVtkMatrix(np.array(segment_end_transforms[i].squeeze()))
                        self.segment_mapping[segment.name]["transform_node(end)"].SetMatrixTransformToParent(end_pose)
                    
                    
                    if segment.vertebrae and segment.vertebrae.geometry.type == 'mesh':
                        self._updateVertebrae(segment, backbone_SPs[i], start_transform_node)
                    
                    self._updateContinuumUnits(segment, self.vtk_matrix_base, backbone_SPs[i])

            finally:
                
                self.rendering_manager.show(self.robot, self.segment_mapping)
            
            

        # print(f"Time spent in __onStateUpdate segment part: {(time.time() - start_time)*1000:.2f} ms")

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
        for transformNode in self.transform_nodes:
            if transformNode:
                slicer.mrmlScene.RemoveNode(transformNode)
        for vertebra_model_node in self.vertebra_model_nodes.values():
            if vertebra_model_node:
                slicer.mrmlScene.RemoveNode(vertebra_model_node)
        for segment_data in self.segment_mapping.values():
            for model_node in segment_data["model_nodes"]:
                if model_node:
                    slicer.mrmlScene.RemoveNode(model_node)
        self.joint_mapping.clear()
        self.link_model_nodes.clear()
        self.vertebra_model_nodes.clear()
        self.root_transform_nodes.clear()
        self.transform_nodes.clear()
        self.segment_mapping.clear()
        self.robot = None
        self.robot_description_node = None
        self.robot_state_node = None
        print("RobotVisualizer cleanup completed.")
    
