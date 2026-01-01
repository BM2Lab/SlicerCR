
import json
import numpy as np
from scipy.interpolate import splprep, splev
from scipy.spatial.transform import Rotation as R
import vtk
import time
from .math_helper import MathHelper
class StateParser:

    def __init__(self):
        pass

    def initializeJsonString(self,num_segments):
        segments = []
        for i in range(num_segments):
            segments.append({
                "segment_name": f"segment_{i}",
                "backbone": [],
                "tendons": {},
                "sheath": [],
                "disks": {}
            })
        return json.dumps({"segments": segments})
    
    def initializeDictofNumpyArrays(self,num_segments):
        segments = {}
        for i in range(num_segments):
            segments[f"segment_{i}"] = {
                "backbone": np.array([]),
                "tendons": {},
                "sheath": np.array([]),
                "disks": {}
            }
        return segments

    def waypointJsonToNumpy(self,json_string):
        """Parse the waypoint JSON and convert to numpy arrays."""
        try:
            data = json.loads(json_string)
            
            parsed_data = {}
            
            for segment in data["segments"]:
                segment_name = segment["segment_name"]
                
                parsed_data[segment_name] = {
                    "backbone": np.array(segment["backbone"]),  # is Nx3 numpy array
                    "tendons": {},
                    "sheath": np.array(segment["sheath"]) if "sheath" in segment else None,
                    "disks": {}
                }
                
                # Parse tendons
                for tendon_name, tendon_waypoints in segment["tendons"].items():
                    parsed_data[segment_name]["tendons"][tendon_name] = np.array(tendon_waypoints)
                parsed_data[segment_name]["disks"]["centers"] = np.array(segment["disks"]["centers"]) if "centers" in segment["disks"] else None
                parsed_data[segment_name]["disks"]["directions"] = np.array(segment["disks"]["directions"]) if "directions" in segment["disks"] else None
            return parsed_data
            
        except json.JSONDecodeError as e:
            print(f"JSON parsing error: {e}")
            return None
        except KeyError as e:
            print(f"Missing required key: {e}")
            return None

    def numpyToWaypointJson(self,waypoint_data):
        """Convert numpy arrays back to JSON format."""
        json_data = {"segments": []}
        
        for segment_name, segment_data in waypoint_data.items():
            segment_json = {
                "segment_name": segment_name,
                "backbone": segment_data["backbone"].tolist(),  # is Nx3 numpy array
                "tendons": {},
                "sheath": segment_data["sheath"].tolist() if segment_data["sheath"] is not None else [],
                "disks": {}
            }
            
            # Convert tendon data
            for tendon_name, tendon_waypoints in segment_data["tendons"].items():
                segment_json["tendons"][tendon_name] = tendon_waypoints.tolist()
            segment_json["disks"]["centers"] = segment_data["disks"]["centers"].tolist() if segment_data["disks"]["centers"] is not None else []
            segment_json["disks"]["directions"] = segment_data["disks"]["directions"].tolist() if segment_data["disks"]["directions"] is not None else []
            
            json_data["segments"].append(segment_json)
        
        return json.dumps(json_data, indent=2)

    def updateBackboneWaypoints(self,json_string,segment_name,waypoints):
        waypoint_data = self.waypointJsonToNumpy(json_string)
        waypoint_data[segment_name]["backbone"] = waypoints
        return self.numpyToWaypointJson(waypoint_data)

    def updateTendonWaypoints(self,json_string,segment_name,tendon_name,waypoints):
        waypoint_data = self.waypointJsonToNumpy(json_string)
        waypoint_data[segment_name]["tendons"][tendon_name] = waypoints
        return self.numpyToWaypointJson(waypoint_data)
    
    def updateSheathWaypoints(self,json_string,segment_name,waypoints):
        waypoint_data = self.waypointJsonToNumpy(json_string)
        waypoint_data[segment_name]["sheath"] = waypoints
        return self.numpyToWaypointJson(waypoint_data)
    
    def initializeWaypointData(self,lengths)->np.ndarray:
        num_points_each_segment = 10
        num_segments = len(lengths)
        waypoint_data = np.zeros((num_segments, num_points_each_segment, 3))
        for i in range(num_segments):
            waypoint_data[i][:,2] = np.linspace(0, lengths[i], num_points_each_segment)
        return waypoint_data

class WaypointFitter:
    def __init__(self):
        self.cached_waypoints = None
        self.cached_tck = None

        pass

    def fitCurveToWaypoints(self, waypoints, smoothing=0, num_points=20):
        """
        Fit a smooth curve to waypoints using spline interpolation.
        
        Args:
            waypoints: Nx3 numpy array of waypoints
            smoothing: smoothing parameter (0 = no smoothing)
            num_points: number of points to generate on the curve
        
        Returns:
            curve_points: Mx3 array of interpolated curve points
            curve_derivatives: Mx3 array of first derivatives (tangent direction)
        """
        # splprep expects 3xN, so transpose
        tck, u = splprep(waypoints.T, s=smoothing)
        u_new = np.linspace(0, 1, num_points)
        # splev returns 3xM, so transpose back to Mx3
        curve_points = np.array(splev(u_new, tck, der=0)).T
        curve_derivatives = np.array(splev(u_new, tck, der=1)).T
        return curve_points, curve_derivatives, tck

    def calculateFrenetFrame(self, curve_points, curve_derivatives):
        """
        Calculate Frenet frame (tangent, normal, binormal) for each point on curve.
        
        Args:
            curve_points: Mx3 array of curve points
            curve_derivatives: Mx3 array of first derivatives
        
        Returns:
            tangent: Mx3 array of unit tangent vectors
            normal: Mx3 array of unit normal vectors  
            binormal: Mx3 array of unit binormal vectors
        """
        # Normalize tangent vectors
        tangent = curve_derivatives / np.linalg.norm(curve_derivatives, axis=1, keepdims=True)
        # Calculate second derivatives numerically
        second_derivatives = np.gradient(curve_derivatives, axis=0)
        # Calculate normal vectors using Gram-Schmidt
        tangent_dot_second = np.sum(tangent * second_derivatives, axis=1, keepdims=True)
        normal_unnormalized = second_derivatives - tangent_dot_second * tangent
        normal_norms = np.linalg.norm(normal_unnormalized, axis=1, keepdims=True)
        normal = np.zeros_like(normal_unnormalized)
        for i in range(normal.shape[0]):
            if normal_norms[i, 0] > 1e-10:
                normal[i] = normal_unnormalized[i] / normal_norms[i, 0]
            else:
                t = tangent[i]
                if abs(t[0]) < 0.9:
                    perp = np.array([1, 0, 0])
                else:
                    perp = np.array([0, 1, 0])
                n = perp - np.dot(perp, t) * t
                normal[i] = n / np.linalg.norm(n)
        # Calculate binormal vectors
        binormal = np.cross(tangent, normal)
        return tangent, normal, binormal

    def calculateTendonWaypoints(self, curve_points, normal, binormal, tendon_config):
        """
        Calculate tendon waypoints based on centerline curve and tendon configuration.
        
        Args:
            curve_points: Mx3 array of centerline curve points
            normal: Mx3 array of normal vectors
            binormal: Mx3 array of binormal vectors
            tendon_config: (angle, offset) tuple for each tendon
        
        Returns:
            tendon_waypoints: Mx3 array of tendon waypoints
        """
        offset, angle = tendon_config
        local_x = offset * np.cos(angle)
        local_y = offset * np.sin(angle)
        # Transform to global frame (the origin coordinate frame is the start point of the centerline)
        global_offset = local_x * normal + local_y * binormal
        tendon_waypoints = curve_points + global_offset
        return tendon_waypoints
    
    def getFrameFromTransform(self, base_transform, waypoint_data):
        '''
        Args:
            base_transform: vtkMRMLTransformNode of base transform
            waypoint_data: Nx3 numpy array of waypoints
        Returns:
            frame: Mx3 numpy array of frame
        '''
        if isinstance(base_transform, vtk.vtkMatrix4x4):
            vtk_matrix = base_transform
        else:
            vtk_matrix = vtk.vtkMatrix4x4()
            base_transform.GetMatrixTransformToWorld(vtk_matrix)
        np_matrix = MathHelper.vtkMatrixToNpMatrix(vtk_matrix)
        u_new = np.linspace(0, 1, waypoint_data.shape[0])
        z_axis = np.array([vtk_matrix.GetElement(0, 2), vtk_matrix.GetElement(1, 2), vtk_matrix.GetElement(2, 2)])
        _, _, transform_matrices = self.getIntermediatePoses(z_axis, 'ZXY', waypoint_data, u_new)
        normal = np.zeros((waypoint_data.shape[0], 3))
        binormal = np.zeros((waypoint_data.shape[0], 3))
        for i in range(waypoint_data.shape[0]):
            result_matrix = np_matrix @ transform_matrices[i]
            normal[i] = result_matrix[:3,0]
            binormal[i] = result_matrix[:3,1]
        return normal, binormal
    
    def getContinuumUnitWaypoints(self, base_transform, waypoint_data, configs):
        '''
        Args:
            base_transform: vtkMRMLTransformNode of base transform
            waypoint_data: Nx3 numpy array of waypoints
            configs: list of [offset, angle] for each tendon
        Returns:
            waypoints: Mx3 numpy array of waypoints
        '''
        result = np.zeros((len(configs),waypoint_data.shape[0], 3))  # NxMx3
        normal = None
        binormal = None
        for i in range(len(configs)):
            if configs[i][0] == 0:
                result[i] = waypoint_data
            else:
                if normal is None and binormal is None:
                    normal, binormal = self.getFrameFromTransform(base_transform, waypoint_data)
                result[i] = self.calculateTendonWaypoints(waypoint_data, normal, binormal, configs[i])
        return result
        

    
    
    def vectors_to_euler_angles(self, v1, v2, convention='ZXY'):
        """
        Calculate Euler angles to transform vector v1 to vector v2.
        
        Args:
            v1: 3x1 numpy array, source vector (normalized)
            v2: 3x1 numpy array, target vector (normalized)
            convention: string, Euler angle convention ('XYZ', 'ZYX', 'ZXY', etc.)
        
        Returns:
            euler_angles: 3x1 numpy array of Euler angles [rad]
            rotation_matrix: 3x3 rotation matrix
        Note:
            Uppercase (e.g., 'XYZ'): Indicates intrinsic rotations, meaning the rotations 
            are applied about the axes of the rotating coordinate system (which change with each rotation).
            Lowercase (e.g., 'xyz'): Indicates extrinsic rotations, meaning the rotations 
            are applied about the axes of a fixed, global coordinate system. 
            vtk.actor use ZXY convention

        """
        # Ensure vectors are normalized
        v1 = v1.flatten() / np.linalg.norm(v1)
        v2 = v2.flatten() / np.linalg.norm(v2)
        
        # Check if vectors are already aligned
        if np.allclose(v1, v2):
            return np.array([0, 0, 0]), np.eye(3)
        
        # Check if vectors are opposite (180° rotation)
        if np.allclose(v1, -v2):
            # Find a perpendicular vector for 180° rotation
            if abs(v1[0]) < 0.9:
                perp = np.array([1, 0, 0])
            else:
                perp = np.array([0, 1, 0])
            
            # Make it perpendicular to v1
            perp = perp - np.dot(perp, v1) * v1
            axis = perp / np.linalg.norm(perp)
            angle = np.pi
            rotation_matrix = R.from_rotvec(axis * angle).as_matrix()
        else:
            # General case: use Rodrigues' rotation formula
            # Rotation axis (cross product)
            axis = np.cross(v1, v2)
            axis = axis / np.linalg.norm(axis)
            
            # Rotation angle
            cos_angle = np.dot(v1, v2)
            angle = np.arccos(np.clip(cos_angle, -1, 1))
            
            # Rodrigues' rotation formula
            K = np.array([[0, -axis[2], axis[1]],
                        [axis[2], 0, -axis[0]],
                        [-axis[1], axis[0], 0]])

            rotation_matrix = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K
            # print("rotation_matrix: \n", rotation_matrix)
        
        # Extract Euler angles using scipy
        rotation_obj = R.from_matrix(rotation_matrix)
        euler_angles = rotation_obj.as_euler(convention, degrees=True)
        # print("euler_angles: \n", euler_angles)
        return euler_angles, rotation_matrix
    
    def __isCached(self, waypoint_data):
        if self.cached_waypoints is None:
            return False
        if self.cached_waypoints.shape != waypoint_data.shape:
            return False
        if not np.allclose(self.cached_waypoints, waypoint_data):
            return False
        return True
        

    def getIntermediatePoses(self,default_direction, convention, waypoint_data, u_new):

        if self.__isCached(waypoint_data):
            tck = self.cached_tck
        else:
            self.cached_waypoints = waypoint_data.copy() # Remember to copy the value instead of reference
            _, _, tck = self.fitCurveToWaypoints(waypoint_data)
            self.cached_tck = tck
        # _, _, tck = self.fitCurveToWaypoints(waypoint_data)
        point_centers = np.array(splev(u_new, tck, der=0)).T
        point_derivatives = np.array(splev(u_new, tck, der=1)).T
        point_directions = point_derivatives / np.linalg.norm(point_derivatives, axis=1, keepdims=True)

        transform_matrices = []
        for i in range(point_directions.shape[0]):
            euler_angles, rotation_matrix = self.vectors_to_euler_angles( default_direction, point_directions[i],convention)
            point_directions[i] = euler_angles
            transform_matrix = np.eye(4)
            transform_matrix[:3,:3] = rotation_matrix
            transform_matrix[:3,3] = point_centers[i]
            transform_matrices.append(transform_matrix)

       
        return point_centers , point_directions, transform_matrices

    def getEndPose(self, waypoint_data):
        curve_points, curve_derivatives, tck = self.fitCurveToWaypoints(waypoint_data)
        tangent, normal, binormal = self.calculateFrenetFrame(curve_points, curve_derivatives)
        tangent = tangent[-1]
        normal = normal[-1]
        binormal = binormal[-1]
        # construct transform matrix, tangent is z axis, normal is x axis, binormal is y axis
        vtk_matrix = vtk.vtkMatrix4x4()
        
        vtk_matrix.SetElement(0, 0, normal[0])
        vtk_matrix.SetElement(1, 0, normal[1])
        vtk_matrix.SetElement(2, 0, normal[2])
        vtk_matrix.SetElement(0, 1, binormal[0])
        vtk_matrix.SetElement(1, 1, binormal[1])
        vtk_matrix.SetElement(2, 1, binormal[2])
        vtk_matrix.SetElement(0, 2, tangent[0])
        vtk_matrix.SetElement(1, 2, tangent[1])
        vtk_matrix.SetElement(2, 2, tangent[2])
        vtk_matrix.SetElement(0, 3, waypoint_data[-1,0])
        vtk_matrix.SetElement(1, 3, waypoint_data[-1,1])
        vtk_matrix.SetElement(2, 3, waypoint_data[-1,2])
        vtk_matrix.SetElement(3, 3, 1)
        return vtk_matrix
    
    

