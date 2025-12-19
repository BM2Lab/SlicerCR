import vtk
import slicer
import qt
import ctk
from datetime import datetime
import math
from scipy.spatial.transform import Rotation
import numpy as np
import time
class RenderingManager():
        
    def __init__(self) -> None:
        """Called when the logic class is instantiated. Can be used for initializing member variables."""

        self.conversion_scale = 1000
        self.sides = 20
        

    def show(self,robot,segment_mapping):
        
        self._updateModelNode(robot,segment_mapping)
        



    def _updateModelNode(self,robot,segment_mapping):
        '''
        Initialize the model node for continuum units.
        '''
        for segment in robot.segments:
            if segment.continuum_body.continuum_units:
                for idx, unit in enumerate(segment.continuum_body.continuum_units):
                    model_node = segment_mapping[segment.name]["model_nodes"][idx]
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
