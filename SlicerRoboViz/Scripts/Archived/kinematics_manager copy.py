import vtk
from scipy.spatial.transform import Rotation as R
import slicer
from Scripts.Utils.math_helper import MathHelper

class KinematicsManager:
    CONVERSION_SCALE = 1000
    Euler_ANGLE_ORDER = 'xyz'
    def __init__(self,robot_manager):
        self.robot_manager = robot_manager
        self.joint_transform_container = {}
        self.segment_transform_container = {}
        self.root_transform_nodes = {}
        self.transform_nodes = []

    def setupTransformHierarchy(self):
        self.setup()
        self.buildJointTransforms()
        self.ensureJointParents()
        self.buildSegmentTransforms()
        self.ensureSegmentParents()
    
    def setup(self):
        """Setup the mapping for the joints and segments"""

        for index, joint in enumerate(self.robot_manager.robot.joints):
            self.joint_transform_container[joint.name] = {
                "index": index,
                "transform_node": None,
                "initial_transform": None
            }
        for index, segment in enumerate(self.robot_manager.robot.segments):
            self.segment_transform_container[segment.name] = {
                "index": index,
                "transform_node": None,
                "transform_node(start)": None,
                "transform_node(end)": None,
                "initial_transform": None
            }
    
    def buildJointTransforms(self):
        """Build the transforms for the joints"""

        for joint in self.robot_manager.robot.joints:
            initialTransform, transformNode = self.createJointTransform(joint)
            self.joint_transform_container[joint.name]["initial_transform"] = initialTransform
            self.joint_transform_container[joint.name]["transform_node"] = transformNode
            self.attachJointVisual(joint, transformNode)
            self.connectJointToParent(joint, transformNode)

    def attachJointVisual(self, joint, transformNode):
        """Attach the visual to the joint transform node"""

        child_model_node = self.robot_manager.link_model_nodes.get(joint.child)
        if not child_model_node:
            print(f"ERROR: Could not find model node for link: {joint.child}")
            return
        visual_transform_node = child_model_node.GetParentTransformNode()
        if visual_transform_node:
            visual_transform_node.SetAndObserveTransformNodeID(transformNode.GetID())
            print(f"Attached {joint.child} visual to {joint.name}_Transform")

    def connectJointToParent(self, joint, transformNode):
        """Connect the joint to the parent joint"""
        
        joint_parent_name, _ = self.findParentJointbyChildLink(joint.parent)
        if joint_parent_name: # if the joint has a parent, connect the joint to the parent
            parent_transform_node = self.joint_transform_container[joint_parent_name]["transform_node"]
            if parent_transform_node:
                transformNode.SetAndObserveTransformNodeID(parent_transform_node.GetID())
                print(f"Attached {joint.name}_Transform to {joint.parent}_Transform")
            return
        # If no parent joint, the link is the uppermost parent link,
        # we create a root transform node, attach the visual to it and child the joint transform to it
        root_node = self.getOrCreateRootTransform(joint.parent)
        self.attachLinkVisualToRoot(joint.parent, root_node)
        transformNode.SetAndObserveTransformNodeID(root_node.GetID())
        
        print(f"Set {joint.parent} as root")

    def findParentJointbyChildLink(self, child_name):
        """Find joint that has the queried child link"""

        for joint in self.robot_manager.robot.joints:
            if joint.child == child_name:
                return joint.name, joint
        return None, None    

    def ensureJointParents(self):
        """Ensure the parent joint transform is set for all joints.
           This is usually done in the buildJointTransforms function, but in case the joint is not defined at the beginning,
           we need to ensure the parent joint transform is set.
        """

        for joint in self.robot_manager.robot.joints:
            parent_joint_name, _ = self.findParentJointbyChildLink(joint.parent)
            if parent_joint_name:
                parent_transform_node = self.joint_transform_container[parent_joint_name]["transform_node"]
                if parent_transform_node:
                    self.joint_transform_container[joint.name]["transform_node"].SetAndObserveTransformNodeID(parent_transform_node.GetID())

    def buildSegmentTransforms(self):
        for segment in self.robot_manager.robot.segments:
            start_transform_node, end_transform_node = self.createSegmentTransforms(segment)
            self.connectSegmentToParent(segment, start_transform_node)

    def ensureSegmentParents(self):
        """Ensure the parent segment transform is set for all segments.
        """

        for segment in self.robot_manager.robot.segments:
            if segment.parent in self.segment_transform_container:
                parent_end_transform_node = self.segment_transform_container[segment.parent]["transform_node(end)"]
                start_transform_node = self.segment_transform_container[segment.name]["transform_node(start)"]
                if parent_end_transform_node and start_transform_node:
                    start_transform_node.SetAndObserveTransformNodeID(parent_end_transform_node.GetID())
                    print(f"Attached {segment.name}_Transform to {segment.parent}_Transform")


    def createJointTransform(self, joint):
        origin = joint.origin
        initialTransform = self.getOriginTransform(origin.xyz, origin.rpy)
        transformNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLTransformNode", f"{joint.name}_Transform")
        transformNode.SetMatrixTransformToParent(initialTransform)
        return initialTransform,transformNode
    
    def getOriginTransform(self, xyz, rpy):
        """Create VTK transform and apply rotation and translation"""

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


    def getOrCreateRootTransform(self, name,type="link"):
        """Get or create the root transform node for the uppermost parent link"""
        if type == "link":
           node_name = f"{self.robot_manager.robot_name}_{name}_Transform(root)"
        elif type == "segment":
            node_name = f"{name}_Transform(root)"
        root_node = slicer.mrmlScene.GetFirstNodeByName(node_name)
        if not root_node:
            root_node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLTransformNode", node_name)
            self.root_transform_nodes[name] = root_node
        return root_node

    def attachLinkVisualToRoot(self, linkName, rootNode):
        #TODO: check if this is correct.
        modelNode = self.robot_manager.link_model_nodes.get(linkName)
        if not modelNode:
            return
        visualTransformNode = modelNode.GetParentTransformNode()
        if visualTransformNode:
            visualTransformNode.SetAndObserveTransformNodeID(rootNode.GetID())

    def createSegmentTransforms(self, segment):
        """Create the transforms for the segment which has origin transform and endpoint transform"""

        if segment.origin:
            initialTransform = self.getOriginTransform(segment.origin.xyz, segment.origin.rpy)
        else:
            initialTransform = self.getOriginTransform([0, 0, 0], [0, 0, 0])
        self.segment_transform_container[segment.name]["initial_transform"] = initialTransform
        # Transform node for the start point of a segment
        start_transform_node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLTransformNode", f"{segment.name}_Transform(start)")
        start_transform_node.SetMatrixTransformToParent(initialTransform)
        self.segment_transform_container[segment.name]["transform_node(start)"] = start_transform_node
        # Transform node for the end point of a segment
        end_transform_node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLTransformNode", f"{segment.name}_Transform(end)")
        end_transform_node.SetAndObserveTransformNodeID(start_transform_node.GetID())
        self.segment_transform_container[segment.name]["transform_node(end)"] = end_transform_node
        return start_transform_node, end_transform_node

    def connectSegmentToParent(self, segment, start_transform_node):
        """Connect the segment to the parent segment"""

        segment_parent_name = segment.parent
        if segment_parent_name in self.segment_transform_container:
            parent_end_transform_node = self.segment_transform_container[segment_parent_name]["transform_node(end)"]
            if parent_end_transform_node:
                start_transform_node.SetAndObserveTransformNodeID(parent_end_transform_node.GetID())
                print(f"Attached {segment.name}_Transform to {segment_parent_name}_Transform")
            return
        # If no parent segment, the segment is the uppermost parent segment,
        root_transform_node = self.getOrCreateRootTransform(segment_parent_name,type="segment")
        self.root_transform_nodes[segment_parent_name] = root_transform_node
        start_transform_node.SetAndObserveTransformNodeID(root_transform_node.GetID())
        print(f"Set {segment_parent_name} as root")
    
    
    ######################################################
    def modifyTransformHierarchy(self):
        """
        Modify the transform hierarchy to connect the joint and segment hierarchy 
        and convert the initial transforms to Slicer transforms.
        """
        self.connectJointandSegment()
        self.collectTransformSubtree()
        self.convertInitialTransforms()

    def connectJointandSegment(self):
        """The Joint hierarchy and segment hierarchy are separated, we need to connect them.
            The self.root_transform_nodes store the parent name and the root transform node.
            In some cases, the parent name of a segment is in the joint_transform_container, and vice versa.
            We need to connect them together.
        """

        for parent_name, root_node in list(self.root_transform_nodes.items()):
            if parent_name in self.segment_transform_container:
                parent_end_transform_node = self.segment_transform_container[parent_name]["transform_node(end)"]
                if parent_end_transform_node:
                    root_node.SetAndObserveTransformNodeID(parent_end_transform_node.GetID())
                    self.root_transform_nodes.pop(parent_name)
                    continue
            if parent_name in self.robot_manager.link_model_nodes:
                joint_name, _ = self.findParentJointbyChildLink(parent_name)
                print(f"connect segment to joint: {joint_name}")
                
                if joint_name:
                    parent_joint_transform_node = self.joint_transform_container[joint_name]["transform_node"]
                    root_node.SetAndObserveTransformNodeID(parent_joint_transform_node.GetID())
                    self.root_transform_nodes.pop(parent_name)

    def collectTransformSubtree(self):
        """Collect the transform subtree"""

        if not self.root_transform_nodes:
            return
        root_transform_node = next(iter(self.root_transform_nodes.values()))
        self.transform_nodes.append(root_transform_node)
        all_transform_nodes = slicer.mrmlScene.GetNodesByClass("vtkMRMLTransformNode")
        for i in range(all_transform_nodes.GetNumberOfItems()):
            transform_node = all_transform_nodes.GetItemAsObject(i)
            if root_transform_node.IsTransformNodeMyChild(transform_node):
                self.transform_nodes.append(transform_node)

    def convertInitialTransforms(self):
        """Convert the initial transforms to Slicer transforms"""

        for node in self.transform_nodes:
            matrix = vtk.vtkMatrix4x4()
            node.GetMatrixTransformToParent(matrix)
            node.SetMatrixTransformToParent(MathHelper.convert2SlicerTransform(matrix))
        for mapping in self.joint_transform_container.values():
            mapping["initial_transform"] = MathHelper.convert2SlicerTransform(mapping["initial_transform"])
        for mapping in self.segment_transform_container.values(): #TODO: check if this is correct
            mapping["initial_transform"] = MathHelper.convert2SlicerTransform(mapping["initial_transform"])
    
    ######################################################
    def getTransformsHierarchy(self):
        """Get the transforms hierarchy"""

        transforms_hierarchy = {}
        transforms_hierarchy[list(self.root_transform_nodes.keys())[0]] = list(self.root_transform_nodes.values())[0]
        for joint_name, mapping in self.joint_transform_container.items():
            transforms_hierarchy[joint_name] = mapping["transform_node"]
        for segment_name, mapping in self.segment_transform_container.items():
            segment_transform_node_end = mapping["transform_node(end)"]
            segment_transform_node_start = mapping["transform_node(start)"]
            transforms_hierarchy[segment_name] = {"end": segment_transform_node_end, "start": segment_transform_node_start}
        return transforms_hierarchy
    
    ######################################################
    def cleanUp(self):
        """Clean up the transform hierarchy"""
        for transformNode in self.transform_nodes:
            if transformNode:
                slicer.mrmlScene.RemoveNode(transformNode)
        self.root_transform_nodes = {}
        self.transform_nodes = []
        self.joint_transform_container = {}
        self.segment_transform_container = {}
       