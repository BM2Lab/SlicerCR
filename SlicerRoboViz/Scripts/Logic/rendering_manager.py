import vtk
import slicer
import qt
import os
from datetime import datetime
import math
from scipy.spatial.transform import Rotation
import numpy as np
import time
class RenderingManager():
    CONVERSION_SCALE = 1000
    Euler_ANGLE_ORDER = 'xyz'  

    def __init__(self) -> None:
        """Called when the logic class is instantiated. Can be used for initializing member variables."""

        self.conversion_scale = 1000
        self.sides = 20
    
    def renderLinksInSlicer(self, robot_manager):

        robot_manager.link_model_nodes.clear()

        for link in robot_manager.robot.links:
            if not link.visual or not link.visual.geometry or not link.visual.geometry.filename:
                print(f"Skipping link {link.name}: No visual geometry defined.")
                continue

            meshFilePath = os.path.normpath(os.path.join(robot_manager.urdf_dir, link.visual.geometry.filename))
            if not os.path.exists(meshFilePath):
                qt.QMessageBox.critical(None, "Error", f"Mesh file not found: {meshFilePath}")
                continue

            position = link.visual.origin.xyz if link.visual.origin else [0, 0, 0]
            orientation = link.visual.origin.rpy if link.visual.origin else [0, 0, 0]
            color = link.visual.material.color.rgba if link.visual.material and link.visual.material.color else [1, 1, 1, 1]
            scale = link.visual.geometry.scale if link.visual.geometry.scale else [1,1,1]
            scale = [s*self.CONVERSION_SCALE for s in scale]
            modelNode,_ = self.renderMeshInSlicer(meshFilePath, link.name, position, orientation, color, scale= scale)
            
            robot_manager.link_model_nodes[link.name] = modelNode

    

    def renderMeshInSlicer(self, mesh_file_path,model_name, position, orientation, color, scale=None):
        modelNode = slicer.modules.models.logic().AddModel(mesh_file_path)
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
        if scale is not None and  len(scale) == 3:
            transform.Scale([scale[0],scale[1],scale[2]])
        transformNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLTransformNode",f"{model_name}_visual_Transform")
        transformNode.SetMatrixTransformToParent(transform.GetMatrix())
        modelNode.SetAndObserveTransformNodeID(transformNode.GetID())
        return modelNode, transformNode


    def updateModelNode(self,robot,segment_model_nodes):
        '''
        Initialize the model node for continuum units.
        '''
        for segment in robot.segments:
            if segment.continuum_body.continuum_units:
                for idx, unit in enumerate(segment.continuum_body.continuum_units):
                    model_node = segment_model_nodes[segment.name][idx]
                    polydata = self.createTubePolyData(unit.trajectory, 
                                            radius=unit.radius*self.conversion_scale, 
                                            sides=self.sides)
                                            
                    model_node.SetAndObservePolyData(polydata)


    def createTubePolyData(self,trajectory, radius=3, sides=50):
        '''
        Create a tube polydata from a trajectory.
        '''
        trajectory = trajectory.T
        points = vtk.vtkPoints()
        for i in range(trajectory.shape[1]):
            points.InsertNextPoint(trajectory[0,i], trajectory[1,i], trajectory[2,i])
        # Create a polyline to connect the points
        num_points = points.GetNumberOfPoints()
        polyLine = vtk.vtkPolyLine()
        polyLine.GetPointIds().SetNumberOfIds(num_points)
        for i in range(num_points):
            polyLine.GetPointIds().SetId(i, i)

        # Create a cell array to store the polyline
        cells = vtk.vtkCellArray()
        cells.InsertNextCell(polyLine)

        # Create a polydata object to store the points and polyline
        polyData = vtk.vtkPolyData()
        polyData.SetPoints(points)
        polyData.SetLines(cells)

        # Create a tube filter to turn the curve into a tube
        tubeFilter = vtk.vtkTubeFilter()
        tubeFilter.SetInputData(polyData)
        tubeFilter.SetRadius(radius)
        tubeFilter.SetNumberOfSides(sides)
        tubeFilter.Update()
    
        return tubeFilter.GetOutput()
