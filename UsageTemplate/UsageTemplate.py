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
from Scripts.sample_point_generator import SamplePointGenerator
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
        self.SPG = SamplePointGenerator() # sample point generator

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
        backbone_SP0 = self.SPG.getStraightBackboneSPs(length=80, num_points=self.num_points)
        backbone_SP1 = self.SPG.getPeriodicSweepingTubeSPs(time=self.count/100, length=segment_length, num_points=self.num_points)
        backbone_SP2 = self.SPG.getPeriodicSweepingTubeSPs(time=self.count/100,length=segment_length, theta_max=-np.pi/4, num_points=self.num_points)
        backbone_SP3 = self.SPG.getPeriodicSweepingTubeSPs(time=self.count/100,length=segment_length, theta_max=np.pi/4, num_points=self.num_points)
        backbone_SPs = np.concatenate((backbone_SP0, backbone_SP1, backbone_SP2, backbone_SP3), axis=0)
        if self.checkRobotExists(robot_name):
            success, _= self.SRV_logic.updateRobotState(robot_name, joint_positions, backbone_SPs)
            return success
        else:
            return False

    def demo2(self):
        robot_name = "Demo_2_Robot"
        backbone_SP1 = self.SPG.getPeriodicSweepingTubeSPs(time=self.count/1000, length=200, num_points=self.num_points)
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
        wp1 = self.SPG.getPeriodicSweepingTubeSPsWithFixedRadius(radius=radius, theta=a1, num_points=self.num_points)
        wp2 = self.SPG.getPeriodicSweepingTubeSPsWithFixedRadius(radius=radius, theta=a2, num_points=self.num_points)
        wp3 = self.SPG.getPeriodicSweepingTubeSPsWithFixedRadius(radius=radius, theta=a3, num_points=self.num_points)
        # Concatenate all segments
        backbone_SPs = np.concatenate((wp1, wp2, wp3), axis=0)

        if self.checkRobotExists(robot_name):
            success, _= self.SRV_logic.updateRobotState(robot_name, backbone_SPs=backbone_SPs)
            return success
        else:
            return False

    def demo4(self):
        robot_name = "Demo_4_Robot"
        segment_base_SPs = self.SPG.getStraightBackboneSPs(length=1, num_points=self.num_points)
        segment_outer_base_SPs = self.SPG.getStraightBackboneSPs(length=20, num_points=self.num_points)
        segment_inner_base_SPs = self.SPG.getStraightBackboneSPs(length=20, num_points=self.num_points)
        segment_outer_SP1 = self.SPG.getPeriodicSweepingTubeSPs(time=self.count/1000, length=100, num_points=self.num_points)   
        segment_inner_front = self.SPG.getPeriodicSweepingTubeSPs(time=self.count/1000, length=100, num_points=self.num_points)
        segment_inner_back = self.SPG.getPeriodicSweepingTubeSPs(time=self.count/1000, length=60, num_points=self.num_points)

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
        R_1, p_1 = self.SPG.softArmForwardKinematics(L1, q_1, r_1, xi_1)
        R_2, p_2 = self.SPG.softArmForwardKinematics(L2, q_2, r_2, xi_2)
        R_3, p_3 = self.SPG.softArmForwardKinematics(L3, q_3, r_3, xi_3)
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
            SPs_list.append(self.SPG.ccArc(L, d, self.num_points))
        
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
            g_new = self.SPG.propagate_needle(g_last, kappa, [u1, u2], dt)
            self.demo8_joint_value = self.demo8_joint_value + u2*dt
            self.demo8_full_path.append(g_new)
        # Extract shape
        indices = np.linspace(0, len(self.demo8_full_path)-1, self.num_points).astype(int)
        current_pts = np.array([self.demo8_full_path[i][:3, 3]*1000 for i in indices])
        # make current_pts
        resampled_pts = self.SPG.resample_polyline(current_pts, self.num_points)
        self.demo8_index = (self.demo8_index + 1) % len(self.demo8_full_path)
        backbone_SPs = resampled_pts[np.newaxis, :, :]  # (1, window, 3)
        
        if self.checkRobotExists(robot_name):
            success, _ = self.SRV_logic.updateRobotState(robot_name, joint_positions=[self.demo8_joint_value], backbone_SPs=backbone_SPs)
            return success
        else:
            return False

   
        

    

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
