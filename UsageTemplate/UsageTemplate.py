import logging
import os
from typing import Annotated, Optional
import vtk
import qt
import slicer
from slicer.i18n import tr as _
from slicer.i18n import translate
from slicer.ScriptedLoadableModule import *
from slicer.util import VTKObservationMixin
from slicer.parameterNodeWrapper import (
    parameterNodeWrapper,
    WithinRange,
)
import math
from slicer import vtkMRMLScalarVolumeNode
import numpy as np
import time
import csv
from datetime import datetime
import io
from scipy.linalg import expm
#
# UsageTemplate
#


class UsageTemplate(ScriptedLoadableModule):
    """Uses ScriptedLoadableModule base class, available at:
    https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self, parent):
        ScriptedLoadableModule.__init__(self, parent)
        self.parent.title = _("UsageTemplate")  
        self.parent.categories = ["SlicerCR"]
        self.parent.dependencies = []  
        self.parent.contributors = ["Letian Ai (BM2 Lab, Georgia Institute of Technology)"]  
        self.parent.helpText = _("""
                    This module is used to provide a template for SlicerRobot extension
                    """)
        self.parent.acknowledgementText = _("""
                                                Developed by Letian Ai (BM2 Lab, Georgia Institute of Technology)
                                                """)

#
# Register sample data sets in Sample Data module
#
@parameterNodeWrapper
class RobotNode:
    """Parameter node wrapper for robot."""
    robot_names: list[str] = []
    urdf_file_paths: list[str] = []

#
# UsageTemplateWidget
#


class UsageTemplateWidget(ScriptedLoadableModuleWidget, VTKObservationMixin):
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
        self.count = 0
        self.num_points = 20
        self.demo5_transforms_hierarchy = None
        self.demo_flag = 0
        self.demo8_index = 0
        self.demo8_full_path = None
        self.demo8_joint_value = 0

    def setup(self) -> None:
        """Called when the user opens the module the first time and the widget is initialized."""
        ScriptedLoadableModuleWidget.setup(self)

        uiWidget = slicer.util.loadUI(self.resourcePath("UI/UsageTemplate.ui"))
        self.layout.addWidget(uiWidget)
        self.ui = slicer.util.childWidgetVariables(uiWidget)
        uiWidget.setMRMLScene(slicer.mrmlScene)
        self.logic = UsageTemplateLogic()
        self.addObserver(slicer.mrmlScene, slicer.mrmlScene.StartCloseEvent, self.onSceneStartClose)
        self.addObserver(slicer.mrmlScene, slicer.mrmlScene.EndCloseEvent, self.onSceneEndClose)

        self.timer_1 = qt.QTimer()
        self.timer_1.timeout.connect(self.onTimer1Timeout)
        self.SRV_logic = slicer.util.getModuleLogic("SlicerRoboViz")
        self.ui.DemoSelectPushBtn.clicked.connect(self.onDemoSelectPushBtnClicked)

        

    def onDemoSelectPushBtnClicked(self):
        if self.ui.DemoSelectPushBtn.text == "Start":
            self.ui.DemoSelectPushBtn.setText("Stop")
            time_interval = int(1000 / int(self.ui.FrequencyInput.text))
            self.timer_1.start(time_interval)
        else:
            self.ui.DemoSelectPushBtn.setText("Start")
            self.timer_1.stop()
            

    def onTimer1Timeout(self):
        self.timer_1.stop()
        self.count += 1
        if self.ui.DemoNumberDropDown.currentText == "Demo 1":
            self.demo_flag = 1
            success = self.demo1()
        elif self.ui.DemoNumberDropDown.currentText == "Demo 2":
            self.demo_flag = 2
            success = self.demo2()
        elif self.ui.DemoNumberDropDown.currentText == "Demo 3":
            self.demo_flag = 3
            success = self.demo3()
        elif self.ui.DemoNumberDropDown.currentText == "Demo 4":
            self.demo_flag = 4
            success = self.demo4()
        elif self.ui.DemoNumberDropDown.currentText == "Demo 5":
            if self.demo_flag !=5:
                self.demo5_transforms_hierarchy = None
            self.demo_flag = 5
            success = self.demo5()
        elif self.ui.DemoNumberDropDown.currentText == "Demo 6":
            self.demo_flag = 6
            success = self.demo6()
        elif self.ui.DemoNumberDropDown.currentText == "Demo 7":
            self.demo_flag = 7
            success = self.demo7()
        elif self.ui.DemoNumberDropDown.currentText == "Demo 8":
            if self.demo_flag != 8:
                self.demo8_index = 0
            self.demo_flag = 8
            success = self.demo8()

        self.timer_1.start(int(1000 / int(self.ui.FrequencyInput.text)))
        
    def checkRobotExists(self, robot_name):
        SRVRobot_node = slicer.mrmlScene.GetFirstNodeByName("SRVRobotsNode")
        if SRVRobot_node:
            robot_wrapper = RobotNode(SRVRobot_node)
            if robot_name in robot_wrapper.robot_names:  # Direct list access
                self.ui.InformationWindow.setText("")
                return True
        self.ui.InformationWindow.setText("No such a robot in the scene: " + robot_name)
        return False

    def demo1(self):
        robot_name = "Demo_1_Robot"
        scale = math.sin(self.count / 10.0) * 0.5 + 0.5 
        joint_positions = [0, scale * math.radians(45), scale * math.radians(60),scale * math.radians(60),
                           -scale * math.radians(90), scale * math.radians(45), scale * math.radians(30),scale * math.radians(30),
                           scale * math.radians(90), scale * math.radians(45), scale * math.radians(30),scale * math.radians(30)]
        segment_length = 125
        backbone_SP0 = self.getStraightBackboneSPs(length=80, num_points=self.num_points)
        backbone_SP1 = self.getPeriodicSweepingTubeSPs(time=self.count/100, length=segment_length, num_points=self.num_points)
        backbone_SP2 = self.getPeriodicSweepingTubeSPs(time=self.count/100,length=segment_length, theta_max=-np.pi/4, num_points=self.num_points)
        backbone_SP3 = self.getPeriodicSweepingTubeSPs(time=self.count/100,length=segment_length, theta_max=np.pi/4, num_points=self.num_points)
        backbone_SPs = np.concatenate((backbone_SP0, backbone_SP1, backbone_SP2, backbone_SP3), axis=0)
        if self.checkRobotExists(robot_name):
            success, _= self.SRV_logic.updateRobotState(robot_name, joint_positions, backbone_SPs)
            return success
        else:
            return False

    def demo2(self):
        robot_name = "Demo_2_Robot"
        backbone_SP1 = self.getPeriodicSweepingTubeSPs(time=self.count/1000, length=200, num_points=self.num_points)
        if self.checkRobotExists(robot_name):
            success, _= self.SRV_logic.updateRobotState(robot_name, backbone_SPs=backbone_SP1)
            return success
        else:
            return False

    def demo3(self):
        robot_name = "Demo_3_Robot"
        # Animation parameters
        # max_angle = np.pi / 3  # Maximum angle span for each arc (60 degrees)
        max_angle = [np.pi/3, np.pi/4, np.pi/3]
        min_angle = 0.01       # Minimal angle span for non-extending segments
        radius = 200           # Fixed radius for all arcs
        phase_duration = 20   # Number of timer ticks per phase (adjust for speed)
        total_phases = 6       # 3 extend + 3 retract
        phase = (self.count // phase_duration) % total_phases
        t_in_phase = (self.count % phase_duration) / phase_duration

        # Compute angle_span for each segment based on phase
        if phase == 0:  # Segment 1 extends
            a1 = min_angle + (max_angle[0] - min_angle) * t_in_phase
            a2 = min_angle
            a3 = min_angle
        elif phase == 1:  # Segment 2 extends
            a1 = max_angle[0]
            a2 = min_angle + (max_angle[1] - min_angle) * t_in_phase
            a3 = min_angle
        elif phase == 2:  # Segment 3 extends
            a1 = max_angle[0]
            a2 = max_angle[1]
            a3 = min_angle + (max_angle[2] - min_angle) * t_in_phase
        elif phase == 3:  # Segment 3 retracts
            a1 = max_angle[0]
            a2 = max_angle[1]
            a3 = max_angle[2] - (max_angle[2] - min_angle) * t_in_phase
        elif phase == 4:  # Segment 2 retracts
            a1 = max_angle[0]
            a2 = max_angle[1] - (max_angle[1] - min_angle) * t_in_phase
            a3 = min_angle
        elif phase == 5:  # Segment 1 retracts
            a1 = max_angle[0] - (max_angle[0] - min_angle) * t_in_phase
            a2 = min_angle
            a3 = min_angle
        else:
            a1 = a2 = a3 = min_angle

        # Generate SPs for each segment with current angle_span
        wp1 = self.getPeriodicSweepingTubeSPsWithFixedRadius(radius=radius, theta=a1, num_points=self.num_points)
        wp2 = self.getPeriodicSweepingTubeSPsWithFixedRadius(radius=radius, theta=a2, num_points=self.num_points)
        wp3 = self.getPeriodicSweepingTubeSPsWithFixedRadius(radius=radius, theta=a3, num_points=self.num_points)
        # Concatenate all segments
        backbone_SPs = np.concatenate((wp1, wp2, wp3), axis=0)

        if self.checkRobotExists(robot_name):
            success, _= self.SRV_logic.updateRobotState(robot_name, backbone_SPs=backbone_SPs)
            return success
        else:
            return False

    def demo4(self):
        robot_name = "Demo_4_Robot"
        segment_base_SPs = self.getStraightBackboneSPs(length=1, num_points=self.num_points)
        segment_outer_base_SPs = self.getStraightBackboneSPs(length=20, num_points=self.num_points)
        segment_inner_base_SPs = self.getStraightBackboneSPs(length=20, num_points=self.num_points)
        segment_outer_SP1 = self.getPeriodicSweepingTubeSPs(time=self.count/1000, length=100, num_points=self.num_points)   
        segment_inner_front = self.getPeriodicSweepingTubeSPs(time=self.count/1000, length=100, num_points=self.num_points)
        segment_inner_back = self.getPeriodicSweepingTubeSPs(time=self.count/1000, length=60, num_points=self.num_points)

        backbone_SPs = np.concatenate((segment_base_SPs, segment_outer_base_SPs, 
                                             segment_inner_base_SPs, segment_outer_SP1,segment_inner_front, segment_inner_back), axis=0)
        if self.checkRobotExists(robot_name):
            success, _= self.SRV_logic.updateRobotState(robot_name, backbone_SPs=backbone_SPs)
            return success
        else:
            return False

    def demo5(self):
            robot_name = "Demo_5_Robot"
            scale = math.sin(self.count / 10.0) 
            # 15 joints
            angle = 10
            joint_positions = [scale * math.radians(angle)]*15
            joint_names = ["joint_unit_1_unit_2", "joint_unit_2_unit_3", "joint_unit_3_unit_4", 
                                    "joint_unit_4_unit_5", "joint_unit_5_unit_6", "joint_unit_6_unit_7", 
                                    "joint_unit_7_unit_8", "joint_unit_8_unit_9", "joint_unit_9_unit_10", 
                                    "joint_unit_10_unit_11", "joint_unit_11_unit_12", "joint_unit_12_unit_13", 
                                    "joint_unit_13_unit_14", "joint_unit_14_unit_15", "joint_unit_15_unit_16"]
            # specify the SPs of the backbone
            original_points_left = np.array([[10,0,5.1],[10,0,8.4]])
            original_points_right = np.array([[-10,0,5.1],[-10,0,8.4]])
            scales =[
                        0.8912, 0.7942, 0.7078, 0.6308, 0.5622,
                        0.5010, 0.4465, 0.3979, 0.3546, 0.3160,
                        0.2816, 0.2510, 0.2237, 0.1994, 0.1777
                    ]
            view = slicer.app.layoutManager().threeDWidget(0).threeDView()
            view.setRenderEnabled(False)
            if self.checkRobotExists(robot_name):
                logic = slicer.util.getModuleLogic("SlicerRoboViz")
                logic.updateRobotState(robot_name, joint_positions=joint_positions)
                if self.demo5_transforms_hierarchy is None:
                    self.demo5_transforms_hierarchy = logic.getTransformsHierarchy(robot_name)
                # get the transform node for each joint, get the tranform from the root to this transform and apply it to the scaled points
                root_transform_node = self.demo5_transforms_hierarchy[list(self.demo5_transforms_hierarchy.keys())[0]]
                
                SPs_left = np.zeros((1, (len(joint_names)+1)*2, 3))
                SPs_left[0,0,:] = original_points_left[0,0:3]
                SPs_left[0,1,:] = original_points_left[1,0:3]
                
                SPs_right = np.zeros((1, (len(joint_names)+1)*2, 3))
                SPs_right[0,0,:] = original_points_right[0,0:3]
                SPs_right[0,1,:] = original_points_right[1,0:3]
                for i in range(len(joint_names)):
                    transform_matrix = vtk.vtkMatrix4x4()
                    joint_transform_node = self.demo5_transforms_hierarchy[joint_names[i]]
                    joint_transform_node.GetMatrixTransformToNode(root_transform_node, transform_matrix)
                    scaled_points_left = original_points_left * scales[i]
                    # add a 1 to the end of the scaled_points_left
                    scaled_points_left = np.concatenate((scaled_points_left, np.ones((2,1))), axis=1)
                    scaled_points_right = original_points_right * scales[i]
                    # add a 1 to the end of the scaled_points_right
                    scaled_points_right = np.concatenate((scaled_points_right, np.ones((2,1))), axis=1)
                    transform_matrix_np = np.zeros((4,4))
                    for m in range(4):
                        for n in range(4):
                            transform_matrix_np[m,n] = transform_matrix.GetElement(m,n)
                    scaled_points_left = (transform_matrix_np @ scaled_points_left.T).T
                    scaled_points_right = (transform_matrix_np @ scaled_points_right.T).T
                    SPs_left[0,i*2+2:i*2+4,:] = scaled_points_left[0:2,0:3]
                    SPs_right[0,i*2+2:i*2+4,:] = scaled_points_right[0:2,0:3]
                SPs_left = np.insert(SPs_left, 0, np.array([[10,0,0]]), axis=1)
                SPs_right = np.insert(SPs_right, 0, np.array([[-10,0,0]]), axis=1)
                
                backbone_SPs = np.concatenate((SPs_left, SPs_right), axis=0)
                success, execution_time = logic.updateRobotState(robot_name, backbone_SPs=backbone_SPs)
                view.setRenderEnabled(True)
                view.forceRender()
                return success
            else:
                view.setRenderEnabled(True)
                view.forceRender()
                return False


    def demo6(self):
        robot_name = "Demo_6_Robot"
        scale = math.sin(self.count / 20.0) * 0.5 + 0.5
        L1 = L2 = L3 = 156
        xi_1 = xi_2 = xi_3 = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1]
        # q_1 = q_2 = q_3 = [30*scale, -30*scale, 40*scale]
        q_1 = q_2 = q_3 = [30*scale, 30, 30]
        r_1 = 46.32
        r_2 = 32
        r_3 = 24
        R_1, p_1 = self.softArmForwardKinematics(L1, q_1, r_1, xi_1)
        R_2, p_2 = self.softArmForwardKinematics(L2, q_2, r_2, xi_2)
        R_3, p_3 = self.softArmForwardKinematics(L3, q_3, r_3, xi_3)
        T_1 = np.zeros((1,4,4))
        T_2 = np.zeros((1,4,4))
        T_3 = np.zeros((1,4,4))
        T_1[0,0:3,0:3] = R_1[-1].reshape(3,3)
        T_2[0,0:3,0:3] = R_2[-1].reshape(3,3)
        T_3[0,0:3,0:3] = R_3[-1].reshape(3,3)
        T_1[0,0:3,3] = p_1[-1].squeeze()
        T_2[0,0:3,3] = p_2[-1].squeeze()
        T_3[0,0:3,3] = p_3[-1].squeeze()
        T_1[0,3,3] = 1
        T_2[0,3,3] = 1
        T_3[0,3,3] = 1
        backbone_SPs = np.concatenate((p_1.reshape(1, -1, 3)  , p_2.reshape(1, -1, 3), p_3.reshape(1, -1, 3)), axis=0)
        end_transforms = np.concatenate((T_1, T_2, T_3), axis=0)
        
        if self.checkRobotExists(robot_name):
            success, _= self.SRV_logic.updateRobotState(robot_name, backbone_SPs=backbone_SPs, segment_end_transforms=end_transforms)
            return success
        else:
            return False
    def demo7(self):
        """
        Demo 7: Tendon-driven catheter with multiple Tendons
        """
        robot_name = "Demo_7_Robot"
        d = 2
        
        # Define parameters for each SP set: [L0, amp]
        SP_params = [
            {'L0': 200, 'amp': 0.5, 'steps': 100},   # SPs_1
            {'L0': 150, 'amp': 2, 'steps': 80},   # SPs_2
            {'L0': 100, 'amp': 5, 'steps': 60}    # SPs_3
        ]
        
        SPs_list = []
        for params in SP_params:
            L0 = params['L0']
            amp = params['amp']
            steps = params['steps']
            l_base = L0 * np.ones(4)
            # Zero-sum actuation trajectory (sum(l_i) stays constant) - periodic using sin/cos
            # dl1 = 2*amp * np.sin(self.count/steps)  # Periodic sine wave
            dl1 = 0
            dl3 = -dl1  # Opposite of dl1
            dl2 = amp * np.cos(self.count/steps)  # Periodic cosine wave (90° phase shift from sin)
            dl4 = -dl2  # Opposite of dl2
            # Combine base lengths with periodic variations
            L = np.array([
                l_base[0] + dl1,
                l_base[1] + dl2,
                l_base[2] + dl3,
                l_base[3] + dl4
            ])
            # Get SPs for this segment
            SPs_list.append(self.ccArc(L, d, self.num_points))
        
        # Concatenate all SP sets
        SPs = np.concatenate(SPs_list, axis=0)
        if self.checkRobotExists(robot_name):
            success, _= self.SRV_logic.updateRobotState(robot_name, backbone_SPs=SPs)
            return success
        else:
            return False

    def demo8(self):
        """
        Demo 8: Bevel-tip needle steering using Lie group integration.
        Generates a single-segment backbone from a kinematic needle model.
        """
        robot_name = "Demo_8_Robot"
        kappa = 2.5
        spin_events = [(10.2, 11.2)]
        total_time = 10
        dt = 0.05
        spin_rate = np.deg2rad(180)
        insertion_speed = 0.02
        total_steps = int(total_time/dt)
        if self.demo8_index % total_steps == 0:
            self.demo8_full_path = [None]*self.num_points
            z_s = np.linspace(0, 0.001, self.num_points)
            for i in range(self.num_points):
                self.demo8_full_path[i] = np.eye(4)
                self.demo8_full_path[i][2, 3] = z_s[i]
            self.demo8_joint_value = 0
        else:
            spinning = any(start <= self.demo8_index*dt <= end for (start, end) in spin_events)
            if spinning:
                u1 = 0.0          # no insertion during spin
                u2 = spin_rate    # rotate needle
            else:
                u1 = insertion_speed
                u2 = 0.0
            g_last = self.demo8_full_path[-1]
            g_new = self.propagate_needle(g_last, kappa, [u1, u2], dt)
            self.demo8_joint_value = self.demo8_joint_value + u2*dt
            self.demo8_full_path.append(g_new)
        # Extract shape
        indices = np.linspace(0, len(self.demo8_full_path)-1, self.num_points).astype(int)
        current_pts = np.array([self.demo8_full_path[i][:3, 3]*1000 for i in indices])
        # make current_pts
        resampled_pts = self.resample_polyline(current_pts, self.num_points)
        self.demo8_index = (self.demo8_index + 1) % len(self.demo8_full_path)
        backbone_SPs = resampled_pts[np.newaxis, :, :]  # (1, window, 3)
        
        if self.checkRobotExists(robot_name):
            success, _ = self.SRV_logic.updateRobotState(robot_name, joint_positions=[self.demo8_joint_value], backbone_SPs=backbone_SPs)
            return success
        else:
            return False

    def softArmForwardKinematics(self, L0, q, r, xi):
        """L0 : float
            Initial length of the actuator
        q : numpy.ndarray
            Actuator elongations of shape (3,)
        r : float
            Radius of the placement of actuators with respect to the center of the disk
        xi : float or list of float
            Value(s) between 0 and 1, representing the location(s) of moving frame(s)
            (0 being the start and 1 is the end)
        
        Returns:
        --------
        R : numpy.ndarray
            Rotation matrix SO(3) of shape (3, 3) if xi is scalar, or (n, 3, 3) if xi is list
        p : numpy.ndarray
            Position vector in R^3 of shape (3, 1) if xi is scalar, or (n, 3, 1) if xi is list
        
        Citation: Azizkhani, M., Kousik, S., & Chen, Y. (2025). Dynamic Task Space Control of 
        Redundant Pneumatically Actuated Soft Robot. IEEE Robotics and Automation Letters.
        """
        
        # Convert xi to numpy array if it's a list
        if isinstance(xi, (list, tuple)):
            xi_array = np.array(xi)
            single_xi = False
        else:
            xi_array = np.array([xi])
            single_xi = True
        
        # Calculate intermediate variables
        A1 = q[0]**2 + q[1]**2 + q[2]**2 - q[0]*q[2] - q[1]*q[2] - q[0]*q[1]
        A2 = 2*q[0] - q[1] - q[2]
        A3 = q[1] - q[2]
        A4 = 3*L0 + q[0] + q[1] + q[2]
        
        # Initialize arrays for multiple xi values
        n_xi = len(xi_array)
        R = np.zeros((n_xi, 3, 3))
        p = np.zeros((n_xi, 3, 1))
        
        # Compute for each xi value
        for i, xi_val in enumerate(xi_array):
            # Calculate R(1,1)
            R[i, 0, 0] = (1 - (A2**2 * A1**4 * xi_val**10) / (837019575 * r**10) + 
                        (A2**2 * A1**3 * xi_val**8) / (4133430 * r**8) - 
                        (A2**2 * A1**2 * xi_val**6) / (32805 * r**6) + 
                        (A1 * A2**2 * xi_val**4) / (486 * r**4) - 
                        (A2**2 * xi_val**2) / (18 * r**2))
            
            # Calculate R(1,2)
            R[i, 0, 1] = ((np.sqrt(3) * A2 * A3 * A1**4 * xi_val**10) / (837019575 * r**10) + 
                        (np.sqrt(3) * A2 * A3 * A1**3 * xi_val**8) / (4133430 * r**8) - 
                        (np.sqrt(3) * A2 * A3 * A1**2 * xi_val**6) / (32805 * r**6) + 
                        (np.sqrt(3) * A2 * A3 * A1 * xi_val**4) / (486 * r**4) - 
                        (np.sqrt(3) * A2 * A3 * xi_val**2) / (18 * r**2))
            
            # Calculate R(1,3)
            R[i, 0, 2] = (-(2 * A2 * A1**4 * xi_val**9) / (55801305 * r**9) + 
                        (4 * A2 * A1**3 * xi_val**7) / (688905 * r**7) - 
                        (2 * A2 * A1**2 * xi_val**5) / (3645 * r**5) + 
                        (2 * A2 * A1 * xi_val**3) / (81 * r**3) - 
                        (A2 * xi_val) / (3 * r))
            
            # Use symmetries for other elements
            R[i, 1, 0] = R[i, 0, 1]  # R(2,1) = R(1,2)
            R[i, 2, 0] = -R[i, 0, 2]  # R(3,1) = -R(1,3)
            
            # Calculate R(2,2)
            R[i, 1, 1] = (1 - (A3**2 * A1**4 * xi_val**10) / (279006525 * r**10) + 
                        (A3**2 * A1**3 * xi_val**8) / (1377810 * r**8) - 
                        (A3**2 * A1**2 * xi_val**6) / (10935 * r**6) + 
                        (A3**2 * A1 * xi_val**4) / (162 * r**4) - 
                        (A3**2 * xi_val**2) / (6 * r**2))
            
            # Calculate R(2,3)
            R[i, 1, 2] = (-(2 * np.sqrt(3) * A3 * A1**4 * xi_val**9) / (55801305 * r**9) + 
                        (4 * np.sqrt(3) * A3 * A1**3 * xi_val**7) / (688905 * r**7) - 
                        (2 * np.sqrt(3) * A3 * A1**2 * xi_val**5) / (3645 * r**5) + 
                        (2 * np.sqrt(3) * A3 * A1 * xi_val**3) / (81 * r**3) - 
                        (np.sqrt(3) * A3 * xi_val) / (3 * r))
            
            # Use symmetry for R(3,2)
            R[i, 2, 1] = -R[i, 1, 2]  # R(3,2) = -R(2,3)
            
            # Calculate R(3,3)
            R[i, 2, 2] = (1 - (2 * xi_val**2 * A1) / (9 * r**2) + 
                        (2 * xi_val**4 * A1**2) / (243 * r**4) - 
                        (4 * xi_val**6 * A1**3) / (32805 * r**6) + 
                        (2 * xi_val**8 * A1**4) / (2066715 * r**8) - 
                        (4 * xi_val**10 * A1**5) / (837019575 * r**10))
            
            # Calculate p(1)
            p[i, 0, 0] = (-(A2 * A1**4 * A4 * xi_val**10) / (837019575 * r**9) + 
                        (A2 * A1**3 * A4 * xi_val**8) / (4133430 * r**7) - 
                        (A2 * A1**2 * A4 * xi_val**6) / (32805 * r**5) + 
                        (A2 * A1 * A4 * xi_val**4) / (486 * r**3) - 
                        (A2 * A4 * xi_val**2) / (18 * r))
            
            # Calculate p(2)
            p[i, 1, 0] = (-(np.sqrt(3) * A3 * A1**4 * A4 * xi_val**10) / (837019575 * r**9) + 
                        (np.sqrt(3) * A3 * A1**3 * A4 * xi_val**8) / (4133430 * r**7) - 
                        (np.sqrt(3) * A3 * A1**2 * A4 * xi_val**6) / (32805 * r**5) + 
                        (np.sqrt(3) * A3 * A1 * A4 * xi_val**4) / (486 * r**3) - 
                        (np.sqrt(3) * A3 * A4 * xi_val**2) / (18 * r))
            
            # Calculate p(3)
            p[i, 2, 0] = ((2 * A1**4 * A4 * xi_val**9) / (55801305 * r**8) - 
                        (4 * A1**3 * A4 * xi_val**7) / (688905 * r**6) + 
                        (2 * A1**2 * A4 * xi_val**5) / (3645 * r**4) - 
                        (2 * A1 * A4 * xi_val**3) / (81 * r**2) + 
                        (A4 * xi_val) / 3)
        
        # Return appropriate shape based on input
        if single_xi:
            return R[0], p[0]  # Return (3,3) and (3,1) for single xi
        else:
            return R, p 
        

    def getBackboneSPs(self):
        t = np.linspace(0, 2*np.pi*100, 20) # mm
        SPs = np.stack([
            t,
            np.sin(t)*1.2,
            t * 0.3
        ], axis=-1)  # shape (20, 3)
        # For one segment, shape should be (1, 20, 3)
        return SPs[np.newaxis, ...]  # shape (1, 20, 3)
    
    def getStraightBackboneSPs(self, length=200, num_points=20):
        z = np.linspace(0, length, num_points)
        SPs = np.stack([
            np.zeros_like(z),
            np.zeros_like(z),
            z
        ], axis=-1)  # shape (num_points, 3)
        return SPs[np.newaxis, ...]  # shape (1, num_points, 3)
    
    def getArcBackboneSPs(self, radius=100, angle_span=np.pi/3, num_points=20, z_height=0):
        """
        Generate SPs for an arc in the XY-plane.
        Args:
            radius: radius of the arc
            angle_span: total angle of the arc (radians)
            num_points: number of SPs along the arc
            z_height: constant z value for the arc
        Returns:
            SPs: shape (1, num_points, 3)
        """
        theta = np.linspace(0, angle_span, num_points)
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)
        z = np.full_like(theta, z_height)
        SPs = np.stack([x, y, z], axis=-1)  # (num_points, 3)
        return SPs[np.newaxis, ...]         # (1, num_points, 3)
    def getPeriodicSweepingTubeSPsWithFixedRadius(self, radius=100, theta=np.pi/2, num_points=20):

 
        if theta == 0:
            x_arc = np.zeros(num_points)
            y_arc = np.zeros(num_points)
            z_arc = np.linspace(0, 0.1, num_points) # very small length
        else:
            
            theta = np.linspace(0, theta, num_points)
            x_arc = radius * (1 - np.cos(theta))
            y_arc = np.zeros_like(theta)
            z_arc = radius * np.sin(theta)
        
        # Rotate the arc around the Z-axis by sweep_angle
        # rotate_angle = 0
        rotate_angle = theta
        x = x_arc * np.cos(rotate_angle) - y_arc * np.sin(rotate_angle)
        y = x_arc * np.sin(rotate_angle) + y_arc * np.cos(rotate_angle)
        z = z_arc
        
        SPs = np.stack([x, y, z], axis=-1)  # (num_points, 3)
        return SPs[np.newaxis, ...] 
    
    def getPeriodicSweepingTubeSPs(self,time, length=200, theta_max=np.pi/2, num_points=5):
        """
        Generate SPs for an elastic tube that sweeps back and forth periodically.
        Args:
            time: current time (seconds)
            length: arc length of the tube
            max_radius: maximum radius of curvature (controls sweep amplitude)
            sweep_frequency: frequency of the sweep (Hz)
            num_points: number of SPs along the tube
        Returns:
            SPs: shape (1, num_points, 3)
        """
        # Calculate the current sweep angle (oscillates between -max_angle and +max_angle)
        sweep_angle = theta_max * np.sin(2 * np.pi * time)  # ±45 degrees
        if sweep_angle == 0:
            x_arc = np.zeros(num_points)
            y_arc = np.zeros(num_points)
            z_arc = np.linspace(0, length, num_points)
        else:
            radius = length / sweep_angle
            theta = np.linspace(0, sweep_angle, num_points)
            x_arc = radius * (1 - np.cos(theta))
            y_arc = np.zeros_like(theta)
            z_arc = radius * np.sin(theta)
        
        # Rotate the arc around the Z-axis by sweep_angle
        # rotate_angle = 0
        rotate_angle = sweep_angle
        x = x_arc * np.cos(rotate_angle) - y_arc * np.sin(rotate_angle)
        y = x_arc * np.sin(rotate_angle) + y_arc * np.cos(rotate_angle)
        z = z_arc
        
        SPs = np.stack([x, y, z], axis=-1)  # (num_points, 3)
        return SPs[np.newaxis, ...]         # (1, num_points, 3)
    
    def lengths2arc4(self, l, d):
        """
        Args:
            l: array-like of shape (4,)
                Tendon lengths [l1, l2, l3, l4]
            d: float
                Radial distance to tendon
        Returns:
            kappa, phi, ell: float
                Curvature, plane angle, arc length
        """
        l = np.asarray(l).flatten()
        ell = np.mean(l)  # inextensible center rod
        phi = np.arctan2(l[3] - l[1], l[2] - l[0])
        
        num = (l[0] - 3*l[1] + l[2] + l[3]) * np.sqrt((l[3] - l[1])**2 + (l[2] - l[0])**2)
        den = d * np.sum(l) * (l[3] - l[1])
        
        if ell <= 1e-12:
            kappa = 0.0
        elif den == 0:
            kappa = 0.0
        else:
            kappa = num / den
        
        return kappa, phi, ell
    

    def T_cc(self,kappa, phi, s):
        """
        Args:
                kappa: float
                    Curvature
                phi: float
                    Plane angle
                s: float
                    Arc length
        Returns:
                T: numpy.ndarray
                    Transformation matrix
        """
        epsk = 1e-9
        cphi = np.cos(phi)
        sphi = np.sin(phi)

        if abs(kappa) < epsk:
            # Straight: rotate about z by phi, then translate along +z
            R = np.array([[cphi, -sphi, 0],
                        [sphi,  cphi, 0],
                        [0,     0,    1]])
            p = np.array([0, 0, s])
            T = np.block([[R, p.reshape(3,1)],
                        [np.zeros((1,3)), np.array([[1]])]])
            return T

        th = kappa * s        # bend angle θ = κ s
        c = np.cos(th)
        sn = np.sin(th)

        # In-plane (y-bend) transform
        Ry = np.array([[ c, 0,  sn],
                    [ 0, 1,   0],
                    [-sn, 0,  c]])
        p_in = np.array([(1 - c)/kappa, 0, sn/kappa])

        # Rotate the whole arc about z by phi
        Rz = np.array([[cphi, -sphi, 0],
                    [sphi,  cphi, 0],
                    [0,     0,    1]])

        R = Rz @ Ry
        p = Rz @ p_in

        T = np.block([[R, p.reshape(3,1)],
                    [np.zeros((1,3)), np.array([[1]])]])
        return T
    
    def ccArc(self,l, d, N):
        """
        Args:
            l: array-like of shape (4,)
                Tendon lengths [l1, l2, l3, l4]
            d: float
                Radial distance to tendon
            N: int
                Number of discretization points
        Returns:
            xyz: numpy.ndarray of shape (N, 3)
                Cartesian coordinates along the arc
        """
        # Step 1: compute CC parameters
        kappa, phi, ell = self.lengths2arc4(l, d)

        # Step 2: discretize along the arc
        s = np.linspace(0, ell, N)
        xyz = np.zeros((N, 3))

        for i in range(N):
            T = self.T_cc(kappa, phi, s[i])   # 4x4 matrix
            xyz[i, :] = T[0:3, 3]        # take translation part (column 4 in MATLAB)
        xyz = xyz[np.newaxis, :, :]
        return xyz
    # ============================================================
# Lie algebra utilities for Demo 8 (needle steering example)
# ============================================================


    def hat(self, v):
        """so(3) hat operator for a 3-vector."""
        return np.array(
            [
                [0.0, -v[2], v[1]],
                [v[2], 0.0, -v[0]],
                [-v[1], v[0], 0.0],
            ]
        )


    def se3_hat(self,v6):
        """Convert 6×1 twist vector [v; ω] to 4×4 se(3) matrix."""
        v = v6[:3]
        w = v6[3:]
        m = np.zeros((4, 4))
        m[:3, :3] = self.hat(w)
        m[:3, 3] = v
        return m


    def needle_twists(self,kappa):
        """
        Returns the two left-invariant control vector fields V1 and V2.
        V1 = [e3; κ e1], V2 = [0; e3]
        """
        e1 = np.array([1.0, 0.0, 0.0])
        e3 = np.array([0.0, 0.0, 1.0])
        V1 = np.hstack([e3, kappa * e1])          # forward insertion
        V2 = np.hstack([np.zeros(3), e3])         # axial rotation
        return V1, V2


    def generate_trajectory_time(self,total_time, insertion_speed, spin_events, spin_rate, dt=0.1):
        """
        Generate (u1, u2, dt) commands over time.

        spin_events: list of (t_start, t_end) intervals in seconds.
        insertion_speed: constant insertion velocity when not spinning.
        spin_rate: constant angular velocity during spins.
        dt: time discretization.

        Returns: list of (u1, u2, dt)
        """
        traj = []
        joint_traj= []
        t = 0.0
        while t < total_time:
            spinning = any(start <= t <= end for (start, end) in spin_events)
            if spinning:
                u1 = 0.0          # no insertion during spin
                u2 = spin_rate    # rotate needle
            else:
                u1 = insertion_speed
                u2 = 0.0
            traj.append((u1, u2, dt))
            if len(joint_traj) == 0:
                joint_traj.append(0)
            else:
                joint_traj.append(joint_traj[-1] + u2*dt)
            t += dt
        return traj, joint_traj


    def resample_polyline(self,pts, N):
        """Resample a 3D polyline to have N uniformly spaced points in arc length."""
        if len(pts) == 1:
            return np.repeat(pts, N, axis=0)

        diffs = np.diff(pts, axis=0)
        seg_len = np.linalg.norm(diffs, axis=1)
        s = np.hstack([[0], np.cumsum(seg_len)])
        total_s = s[-1]
        if total_s == 0:
            return np.repeat(pts[:1], N, axis=0)

        s_query = np.linspace(0, total_s, N)
        new_pts = np.zeros((N, 3))
        for i in range(3):
            new_pts[:, i] = np.interp(s_query, s, pts[:, i])
        return new_pts

    def propagate_needle(self,g0, kappa, actuation, dt):
        """
        Propagate the needle using the Lie group integration
        """
        g = g0
        V1, V2 = self.needle_twists(kappa)
        xi = actuation[0] * V1 + actuation[1] * V2
        g = g @ expm(self.se3_hat(xi) * dt)
        
        return g
        
    def simulate_needle(self,kappa, trajectory):
        """
        kappa: curvature parameter (1/m)
        trajectory: list of (u1, u2, dt)
        N: number of resampled SPs

        Returns:
            Nx3 array of needle SPs
        """
        gs = []
        g = np.eye(4)
        gs.append(g)
        V1, V2 = self.needle_twists(kappa)
        for (u1, u2, dt) in trajectory:
            xi = u1 * V1 + u2 * V2
            g = g @ expm(self.se3_hat(xi) * dt)
            gs.append(g)
        return  gs

    ###################################################################
    ##########################Built in Functions#######################
    ###################################################################
    def cleanup(self) -> None:
        """Called when the application closes and the module widget is destroyed."""
        self.removeObservers()
        



    def enter(self) -> None:
        """Called each time the user opens this module."""
        # Make sure parameter node exists and observed
        pass

    def exit(self) -> None:
        """Called each time the user opens a different module."""
        # Do not react to parameter node changes (GUI will be updated when the user enters into the module)
        pass

    def onSceneStartClose(self, caller, event) -> None:
        """Called just before the scene is closed."""
        # Parameter node will be reset, do not use it anymore
        pass

    def onSceneEndClose(self, caller, event) -> None:
        """Called just after the scene is closed."""
        # If this module is shown while the scene is closed then recreate a new parameter node immediately
        pass

#
# UsageTemplateLogic
#


class UsageTemplateLogic(ScriptedLoadableModuleLogic):
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



#
# UsageTemplateTest
#


class UsageTemplateTest(ScriptedLoadableModuleTest):
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
        pass
