import numpy as np
from scipy.linalg import expm

class SamplePointGenerator:
    def __init__(self):
        pass
    
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
    
    def getStraightBackboneSPs(self, length=200, num_points=20):
        z = np.linspace(0, length, num_points)
        SPs = np.stack([
            np.zeros_like(z),
            np.zeros_like(z),
            z
        ], axis=-1)  # shape (num_points, 3)
        return SPs[np.newaxis, ...]  # shape (1, num_points, 3)
    
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
    
    #########################################################
    #################Tendon-driven catheter#################
    #########################################################
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
