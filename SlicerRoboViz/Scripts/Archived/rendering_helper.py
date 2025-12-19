import vtk
import slicer
import qt
import ctk
from datetime import datetime
import math
from scipy.spatial.transform import Rotation
import numpy as np
import time
class RenderingHelper():
        
    def __init__(self) -> None:
        """Called when the logic class is instantiated. Can be used for initializing member variables."""
        self.renderer = slicer.app.layoutManager().threeDWidget(0).threeDView().renderWindow().GetRenderers().GetFirstRenderer()
        self.actors = []
        self.conversion_scale = 1000
        self.sides = 20
        self._actor_cache = {}
        self.is_actor_initialized = False
        pass
        

    def show(self,robot):
        
        # for actor in self.actors:
        #     if actor is not None:
        #         self.renderer.RemoveActor(actor)
        self._updateActor(robot)
        if not self.is_actor_initialized:
            for actor in self.actors:
                if actor is not None:
                    self.renderer.AddActor(actor)
            self.is_actor_initialized = True
        self.renderer.GetRenderWindow().Render()

    def _updateActor(self,robot):
       
        # Initialize actor cache if not exists
        if not self._actor_cache:
            self._createInitialActors(robot)
        
        # Update existing actors
        self._updateExistingActors(robot)

    def _createInitialActors(self, robot):
        """Create actors once and cache them"""
        
        self.actors = []
        for segment in robot.segments:
            segment_id = segment.name
            self._actor_cache[segment_id] = []
            
            # Create tube actors for continuum units
            if segment.continuum_body.continuum_units:
                for unit in segment.continuum_body.continuum_units:
                    unit_actor = self.addTube(unit.trajectory, 
                                            radius=unit.radius*self.conversion_scale, 
                                            sides=self.sides, 
                                            color=unit.color.rgba)
                    self._actor_cache[segment_id].append(unit_actor)
                    self.actors.append(unit_actor)
            
            # Create cylinder actors for disks
            if segment.disks and segment.disks.geometry.type == 'cylinder':
                for idx in range(segment.disks.count):
                    disk_actor = self.addCylinder(
                        radius=segment.disks.geometry.radius*self.conversion_scale, 
                        height=segment.disks.geometry.height*self.conversion_scale, 
                        center=segment.disks.centers[idx], 
                        direction=segment.disks.directions[idx], 
                        sides=self.sides, 
                        color=segment.disks.color.rgba
                    )
                    self._actor_cache[segment_id].append(disk_actor)
                    self.actors.append(disk_actor)

    def _updateExistingActors(self, robot):
        """Update existing actors instead of recreating them"""
        for segment in robot.segments:
            segment_id = segment.name
            actors = self._actor_cache.get(segment_id, [])
            
            actor_index = 0
            
            # Update tube actors for continuum units
            if segment.continuum_body.continuum_units:
                for unit in segment.continuum_body.continuum_units:
                    if actor_index < len(actors):
                        # Update existing tube actor
                        self.updateTubeActorEfficient(
                            actors[actor_index], 
                            unit.trajectory, 
                            radius=unit.radius*self.conversion_scale,
                            color=unit.color.rgba
                        )
                    else:
                        # Create new actor if needed
                        unit_actor = self.addTube(unit.trajectory, 
                                                radius=unit.radius*self.conversion_scale, 
                                                sides=self.sides, 
                                                color=unit.color.rgba)
                        self._actor_cache[segment_id].append(unit_actor)
                        self.actors.append(unit_actor)
                        print("created new actor", len(self.actors))
                    actor_index += 1
            
            # Update cylinder actors for disks
            if segment.disks and segment.disks.geometry.type == 'cylinder':
                for idx in range(segment.disks.count):
                    if actor_index < len(actors):
                        # Update existing cylinder actor
                        self.updateCylinderActor(
                            actors[actor_index],
                            center=segment.disks.centers[idx],
                            direction=segment.disks.directions[idx],
                            radius=segment.disks.geometry.radius*self.conversion_scale,
                            height=segment.disks.geometry.height*self.conversion_scale,
                            color=segment.disks.color.rgba
                        )
                        
                    else:
                        # Create new actor if needed
                        disk_actor = self.addCylinder(
                            radius=segment.disks.geometry.radius*self.conversion_scale, 
                            height=segment.disks.geometry.height*self.conversion_scale, 
                            center=segment.disks.centers[idx], 
                            direction=segment.disks.directions[idx], 
                            sides=self.sides, 
                            color=segment.disks.color.rgba
                        )
                        self._actor_cache[segment_id].append(disk_actor)
                        self.actors.append(disk_actor)
                    actor_index += 1
    
    def addTube(self,trajectory, radius=3, sides=50, color=[0, 1, 0, 1]):
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
        # Create a mapper for the tube
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(tubeFilter.GetOutputPort())
        # Create an actor to represent the tube in the 3D scene
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(color[0], color[1], color[2])
        actor.GetProperty().SetOpacity(color[3])
        return actor
    
    def updateTubeActorEfficient(self, actor, trajectory, radius=None, color=None):
        """More efficient update that reuses existing VTK objects"""
        trajectory = trajectory.T
        
        # Get existing objects
        mapper = actor.GetMapper()
        tube_filter = mapper.GetInputConnection(0, 0).GetProducer()
        poly_data = tube_filter.GetInput()
        points = poly_data.GetPoints()
        
        # Check if we need to resize points array
        current_points = points.GetNumberOfPoints()
        new_points = trajectory.shape[1]
        
        if current_points != new_points:
            # Resize points array
            points.SetNumberOfPoints(new_points)
        
        # Update points efficiently
        for i in range(new_points):
            points.SetPoint(i, trajectory[0,i], trajectory[1,i], trajectory[2,i])
        
        # Update polyline if needed
        if current_points != new_points:
            cells = poly_data.GetLines()
            cells.Reset()
            polyLine = vtk.vtkPolyLine()
            polyLine.GetPointIds().SetNumberOfIds(new_points)
            for i in range(new_points):
                polyLine.GetPointIds().SetId(i, i)
            cells.InsertNextCell(polyLine)
        
        # Update radius if provided
        if radius is not None:
            tube_filter.SetRadius(radius)
        
        # Update color if provided
        if color is not None:
            actor.GetProperty().SetColor(color[0], color[1], color[2])
            actor.GetProperty().SetOpacity(color[3])
        
        # Mark as modified and update
        points.Modified()
        poly_data.Modified()
        tube_filter.Update()
        mapper.Update()

    def addCylinder(self, radius=3, height=10, center=None, direction=None, sides=50, color=[0, 1, 0, 1]):
        if center is None:
            center = [0, 0, 0]
        if direction is None:
            direction = [0, 0, 0]
        center = center.tolist()
        direction = direction.tolist()
        
        cylinderSource = vtk.vtkCylinderSource()
        cylinderSource.SetRadius(radius)
        cylinderSource.SetHeight(height)
        cylinderSource.SetResolution(sides)
        cylinderSource.Update()
        cylinderMapper = vtk.vtkPolyDataMapper()
        cylinderMapper.SetInputConnection(cylinderSource.GetOutputPort())
        cylinderActor = vtk.vtkActor()
        cylinderActor.SetMapper(cylinderMapper)
        cylinderActor.GetProperty().SetColor(color[0], color[1], color[2])
        cylinderActor.GetProperty().SetOpacity(color[3])
        cylinderActor.SetOrientation(direction[1], direction[2], direction[0]) # intrinsic rotation ZXY
        # cylinderActor.RotateX(direction[0]) # intrinsic rotation XYZ, also works
        # cylinderActor.RotateY(direction[1])
        # cylinderActor.RotateZ(direction[2])
        cylinderActor.SetPosition(center)
        return cylinderActor

    def updateCylinderActor(self, actor, center=None, direction=None, radius=None, height=None, color=None):
        """Update existing cylinder actor with new parameters"""
        
        # Update position if provided
        if center is not None:
            center = center.tolist() if hasattr(center, 'tolist') else center
            actor.SetPosition(center)
        
        # Update orientation if provided
        if direction is not None:
            direction = direction.tolist() if hasattr(direction, 'tolist') else direction
            actor.SetOrientation(direction[1], direction[2], direction[0])  # ZXY rotation
        
        # Update color if provided
        if color is not None:
            actor.GetProperty().SetColor(color[0], color[1], color[2])
            actor.GetProperty().SetOpacity(color[3])
        
        # Note: Updating radius/height requires recreating the cylinder source
        # This is more expensive, so we only do it if necessary
        if radius is not None or height is not None:
            # Get current mapper and source
            mapper = actor.GetMapper()
            cylinder_source = mapper.GetInputConnection(0, 0).GetProducer()
            
            if radius is not None:
                cylinder_source.SetRadius(radius)
            if height is not None:
                cylinder_source.SetHeight(height)
            
            # Update the source and mapper
            cylinder_source.Update()
            mapper.Update()
        
    
    def addLine(self,trajectory, width=3, color=[0, 1, 0, 1]):
        trajectory = trajectory.T
        points = vtk.vtkPoints()
        for i in range(trajectory.shape[1]):
            points.InsertNextPoint(trajectory[0,i], trajectory[1,i], trajectory[2,i])
        num_points = points.GetNumberOfPoints()
        last_point = points.GetPoint(num_points - 1)
        second_last_point = points.GetPoint(num_points - 2)
        # Calculate the direction vector from the second last to the last point
        direction = [last_point[i] - second_last_point[i] for i in range(3)]
        # Normalize the direction vector
        direction_length = vtk.vtkMath.Normalize(direction)
        # Create a line source
        lineSource = vtk.vtkLineSource()
        lineSource.SetPoint1(last_point)
        lineSource.SetPoint2([last_point[i] + direction[i] * 10 for i in range(3)])
        lineSource.Update()
        # Create a mapper for the line
        lineMapper = vtk.vtkPolyDataMapper()
        lineMapper.SetInputConnection(lineSource.GetOutputPort())
        # Create an actor for the line
        lineActor = vtk.vtkActor()
        lineActor.SetMapper(lineMapper)
        lineActor.GetProperty().SetColor(0.65, 0.65, 0.65)
        lineActor.GetProperty().SetLineWidth(3)
        return lineActor
    
    def addCone(self,points):
        
        num_points = points.GetNumberOfPoints()
        last_point = points.GetPoint(num_points - 1)  # Get the last point coordinates
        second_last_point = points.GetPoint(num_points - 2)  # To calculate the direction

        # Calculate the direction vector from the second last to the last point
        direction = [last_point[i] - second_last_point[i] for i in range(3)]

        # Normalize the direction vector
        direction_length = vtk.vtkMath.Normalize(direction)

        # Create a cone source
        coneSource = vtk.vtkConeSource()
        coneSource.SetRadius(3)  # Set the radius of the cone base
        coneSource.SetHeight(10)  # Set the height of the cone
        coneSource.SetDirection(direction)  # Set the orientation of the cone
        coneSource.SetCenter(last_point)  # Set the position of the cone
        coneSource.Update()

        # Create a mapper for the cone
        coneMapper = vtk.vtkPolyDataMapper()
        coneMapper.SetInputConnection(coneSource.GetOutputPort())

        # Create an actor for the cone
        coneActor = vtk.vtkActor()
        coneActor.SetMapper(coneMapper)
        coneActor.GetProperty().SetColor(1, 0, 0)  # Set cone color (red)

        return coneActor
    
    def addRibbon(self, trajectory, color=[1, 0, 0, 1], width=10):
        trajectory = trajectory.T
        points = vtk.vtkPoints()
        for i in range(trajectory.shape[1]):
            points.InsertNextPoint(trajectory[0, i], trajectory[1, i], trajectory[2, i])
        num_points = points.GetNumberOfPoints()
        polyLine = vtk.vtkPolyLine()
        polyLine.GetPointIds().SetNumberOfIds(num_points)
        for i in range(num_points):
            polyLine.GetPointIds().SetId(i, i)
        cells = vtk.vtkCellArray()
        cells.InsertNextCell(polyLine)
        polyData = vtk.vtkPolyData()
        polyData.SetPoints(points)
        polyData.SetLines(cells)
        ribbonFilter = vtk.vtkRibbonFilter()
        ribbonFilter.SetInputData(polyData)
        ribbonFilter.SetWidth(width)
        ribbonFilter.Update()
        ribbonMapper = vtk.vtkPolyDataMapper()
        ribbonMapper.SetInputConnection(ribbonFilter.GetOutputPort())
        ribbonActor = vtk.vtkActor()
        ribbonActor.SetMapper(ribbonMapper)
        ribbonActor.GetProperty().SetColor(color[0], color[1], color[2])
        ribbonActor.GetProperty().SetOpacity(color[3])
        return ribbonActor
    
    def addNotchWall(self, outerRadius, thickness, height, angle, sides=8,center=[0, 0, 0],direction=[0, 0, 0], color=[1, 0, 0, 1]):
  
        innerRadius = outerRadius - thickness
        if innerRadius <= 0:
            raise ValueError("Inner radius must be positive. Check outerRadius and thickness.")
        if angle <= 0 or angle >= 360:
            raise ValueError("Notch angle must be between 0 and 360 degrees.")

        # The angle covered by the notch
        startAngle = -angle / 2.0
        endAngle = angle / 2.0

        n = sides + 1  # +1 to close the arc

        points = vtk.vtkPoints()
        top_z = height / 2.0
        bottom_z = -height / 2.0
         # Outer arc (top)
        for i in range(n):
            angle = math.radians(startAngle + (endAngle - startAngle) * i / (n - 1))
            x = outerRadius * math.cos(angle)
            y = outerRadius * math.sin(angle)
            points.InsertNextPoint(x, y, top_z)
        # Inner arc (top)
        for i in range(n):
            angle = math.radians(startAngle + (endAngle - startAngle) * i / (n - 1))
            x = innerRadius * math.cos(angle)
            y = innerRadius * math.sin(angle)
            points.InsertNextPoint(x, y, top_z)
        # Outer arc (bottom)
        for i in range(n):
            angle = math.radians(startAngle + (endAngle - startAngle) * i / (n - 1))
            x = outerRadius * math.cos(angle)
            y = outerRadius * math.sin(angle)
            points.InsertNextPoint(x, y, bottom_z)
        # Inner arc (bottom)
        for i in range(n):
            angle = math.radians(startAngle + (endAngle - startAngle) * i / (n - 1))
            x = innerRadius * math.cos(angle)
            y = innerRadius * math.sin(angle)
            points.InsertNextPoint(x, y, bottom_z)

        polys = vtk.vtkCellArray()
        offset = 2 * n
        # Top face
        for i in range(n - 1):
            polys.InsertNextCell(4)
            polys.InsertCellPoint(i)
            polys.InsertCellPoint(i + 1)
            polys.InsertCellPoint(n + i + 1)
            polys.InsertCellPoint(n + i)
        # Bottom face
        for i in range(n - 1):
            polys.InsertNextCell(4)
            polys.InsertCellPoint(offset + i)
            polys.InsertCellPoint(offset + i + 1)
            polys.InsertCellPoint(offset + n + i + 1)
            polys.InsertCellPoint(offset + n + i)
        # Outer wall
        for i in range(n - 1):
            polys.InsertNextCell(4)
            polys.InsertCellPoint(i)
            polys.InsertCellPoint(i + 1)
            polys.InsertCellPoint(offset + i + 1)
            polys.InsertCellPoint(offset + i)
        # Inner wall
        for i in range(n - 1):
            polys.InsertNextCell(4)
            polys.InsertCellPoint(n + i)
            polys.InsertCellPoint(n + i + 1)
            polys.InsertCellPoint(offset + n + i + 1)
            polys.InsertCellPoint(offset + n + i)

        # First side face
        polys.InsertNextCell(4)
        polys.InsertCellPoint(0)
        polys.InsertCellPoint(n)
        polys.InsertCellPoint(offset +n)
        polys.InsertCellPoint(offset)
        # Second side face
        polys.InsertNextCell(4)
        polys.InsertCellPoint(n - 1)
        polys.InsertCellPoint(2 * n - 1)
        polys.InsertCellPoint(offset + 2 * n - 1)
        polys.InsertCellPoint(offset + n - 1)


        polyData = vtk.vtkPolyData()
        polyData.SetPoints(points)
        polyData.SetPolys(polys)

        normals = vtk.vtkPolyDataNormals()
        normals.SetInputData(polyData)
        normals.ConsistencyOn()
        normals.Update()

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(normals.GetOutput())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(color[0], color[1], color[2])
        actor.GetProperty().SetOpacity(color[3])
        actor.SetOrientation(direction[1], direction[2], direction[0]) # intrinsic rotation ZXY
        actor.SetPosition(center)
        return actor


    def clear(self):
        for actor in self.actors:
            if actor is not None:
                self.renderer.RemoveActor(actor)
        self.actors = []
        self._actor_cache = {}
        self.is_actor_initialized = False
        self.renderer.GetRenderWindow().Render()