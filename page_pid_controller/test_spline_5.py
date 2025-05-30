import rospy
from geometry_msgs.msg import Pose, Twist, PoseWithCovarianceStamped
import numpy as np
import tkinter as tk
import tkinter.ttk as ttk
import threading
import tf.transformations
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
# Scipy splrep/splev are no longer used for the main path
from scipy.signal import butter, filtfilt

# --- Helper functions for Catmull-Rom to Bezier conversion ---
def get_catmull_rom_intermediate_point(points_list, index):
    """
    Helper to get points for Catmull-Rom, duplicating ends.
    points_list is a list of np.array([x,y]).
    """
    if not points_list:
        # This case should be handled before calling, but as a fallback:
        rospy.logwarn_throttle(5.0, "get_catmull_rom_intermediate_point called with empty points_list.")
        return np.array([0.0, 0.0])
    if index < 0:
        return points_list[0]
    if index >= len(points_list):
        return points_list[-1]
    return points_list[index]

def cubic_bezier_point(p0, p1, p2, p3, t):
    """Calculates a point on a cubic Bezier curve (p0,p1,p2,p3 are start, cp1, cp2, end)."""
    t = float(t)
    return (1-t)**3 * p0 + 3*(1-t)**2 * t * p1 + 3*(1-t) * t**2 * p2 + t**3 * p3

def cubic_bezier_derivative(p0, p1, p2, p3, t):
    """Calculates the derivative (dp/dt) of a cubic Bezier curve."""
    t = float(t)
    # Derivative of the Bezier formula w.r.t. t
    return 3*(1-t)**2 * (p1-p0) + 6*(1-t)*t * (p2-p1) + 3*t**2 * (p3-p2)
# --- End Helper functions ---

class MPCController:
    def __init__(self, dt, v_max, v_min, omega_max, omega_min, lookahead_distance=0.1, filter_order=4, cutoff_frequency=4.0): # spline_degree removed
        self.dt = dt
        self.v_max = v_max
        self.v_min = v_min
        self.omega_max = omega_max
        self.omega_min = omega_min
        self.current_pose = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.trajectory_x = []
        self.trajectory_y = []

        # Waypoints will be a list of np.array([x,y])
        self.waypoints = []
        
        # Path representation based on Catmull-Rom to Bezier
        self.bezier_segments = [] # List of [P_start, CP1, CP2, P_end] np.arrays
        self.segment_arclengths_approx = []
        self.cumulative_arclengths_approx = []
        self.total_arclength_approx = 0.0
        self.num_arclength_samples_per_segment = 30 # For approximating arclength

        self.distance_threshold = 0.2
        self.lookahead_distance = lookahead_distance
        self.reached_goal = False
        self.last_v = 0.0
        self.last_omega = 0.0
        self.velocity_data = []
        self.angular_velocity_data = []
        self.acceleration_data = []
        self.angular_acceleration_data = []
        self.time_data = []
        self.start_time = None
        
        self.filter_order = filter_order
        self.cutoff_frequency = cutoff_frequency
        
        if self.dt <= 0:
            rospy.logerr("dt must be positive for filter design. Disabling filter.")
            self.b, self.a = ([1], [1])
        else:
            nyquist_freq = 0.5 / self.dt
            normalized_cutoff = self.cutoff_frequency / nyquist_freq
            if normalized_cutoff >= 1.0:
                rospy.logwarn(f"Cutoff frequency ({self.cutoff_frequency} Hz) is at or above Nyquist frequency ({nyquist_freq} Hz). Clamping.")
                normalized_cutoff = 0.99
            elif normalized_cutoff <= 0:
                rospy.logwarn(f"Cutoff frequency ({self.cutoff_frequency} Hz) must be positive. Disabling filter.")
                self.b, self.a = ([1], [1])
            else:
                self.b, self.a = butter(self.filter_order, normalized_cutoff, btype='low', analog=False)

        self.ramp_up_duration = 2.0
        self.ramp_down_distance = 0.5
        self.velocity_smoothing_alpha = 0.3

    def lowpass_filter(self, data):
        if len(self.b) == 1 and self.b[0] == 1 and len(self.a) == 1 and self.a[0] == 1:
            return data
        padlen = 3 * max(len(self.a), len(self.b), self.filter_order)
        if len(data) <= padlen:
            return data
        try:
            y = filtfilt(self.b, self.a, data, padlen=padlen)
            return y
        except ValueError as e:
            rospy.logwarn(f"Error during filtering: {e}. Returning unfiltered data. Data length: {len(data)}, padlen: {padlen}")
            return data

    def smooth_velocity(self, new_v, new_omega):
        smoothed_v = (1 - self.velocity_smoothing_alpha) * self.last_v + self.velocity_smoothing_alpha * new_v
        smoothed_omega = (1 - self.velocity_smoothing_alpha) * self.last_omega + self.velocity_smoothing_alpha * new_omega
        return smoothed_v, smoothed_omega

    def smooth_ramp_factor(self, factor):
        return np.sin(factor * np.pi / 2)

    def pose_callback(self, msg):
        pose_msg = msg.pose.pose
        quat = pose_msg.orientation
        norm_sq = quat.x**2 + quat.y**2 + quat.z**2 + quat.w**2
        if norm_sq < 1e-9:
            rospy.logwarn_throttle(5.0, "Invalid (zero) quaternion received, skipping pose update.")
            return

        try:
            euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            _, _, yaw = euler
        except Exception as e:
            rospy.logerr(f"Error converting quaternion to Euler: {e}")
            return

        self.x = pose_msg.position.x
        self.y = pose_msg.position.y
        self.theta = yaw
        self.current_pose = np.array([self.x, self.y, self.theta])
        self.trajectory_x.append(self.x)
        self.trajectory_y.append(self.y)

        if not self.waypoints: # If no waypoints exist, set the first one to current pose
            self.waypoints.append(np.array([self.x, self.y]))
            rospy.loginfo(f"Initial waypoint set to robot's current pose: ({self.x:.2f}, {self.y:.2f})")
        else: 
            # If waypoints exist, update the first one if it's meant to be the robot's dynamic start
            # This is consistent with the previous logic of waypoints_x[0] = self.x
            self.waypoints[0] = np.array([self.x, self.y])


    def calculate_path(self): # Renamed from calculate_spline
        self.bezier_segments = []
        self.segment_arclengths_approx = []
        self.cumulative_arclengths_approx = []
        self.total_arclength_approx = 0.0

        if len(self.waypoints) < 2:
            rospy.logwarn("Need at least two waypoints to create a path.")
            return False

        # Filter out consecutive duplicate waypoints to prevent zero-length segments / issues
        unique_wps = []
        if self.waypoints:
            unique_wps.append(self.waypoints[0])
            for i in range(1, len(self.waypoints)):
                if np.linalg.norm(self.waypoints[i] - self.waypoints[i-1]) > 1e-6:
                    unique_wps.append(self.waypoints[i])
        
        if len(unique_wps) < 2:
            rospy.logwarn("After filtering duplicates, less than two unique waypoints remain. Cannot create path.")
            self.waypoints = [] # Clear waypoints if they are all duplicates
            return False
        
        effective_waypoints = unique_wps

        for i in range(len(effective_waypoints) - 1):
            p0_catmull = get_catmull_rom_intermediate_point(effective_waypoints, i - 1)
            p1_catmull = get_catmull_rom_intermediate_point(effective_waypoints, i)     # Start of Bezier segment
            p2_catmull = get_catmull_rom_intermediate_point(effective_waypoints, i + 1) # End of Bezier segment
            p3_catmull = get_catmull_rom_intermediate_point(effective_waypoints, i + 2)
            
            # Bezier control points
            cp1 = p1_catmull + (p2_catmull - p0_catmull) / 6.0
            cp2 = p2_catmull - (p3_catmull - p1_catmull) / 6.0
            
            segment_points = [p1_catmull, cp1, cp2, p2_catmull] # P_start, CP1, CP2, P_end
            self.bezier_segments.append(segment_points)

            # Approximate arclength for this segment
            length = 0.0
            prev_p = cubic_bezier_point(*segment_points, 0.0)
            for k_sample in range(1, self.num_arclength_samples_per_segment + 1):
                t_sample = float(k_sample) / self.num_arclength_samples_per_segment
                curr_p = cubic_bezier_point(*segment_points, t_sample)
                length += np.linalg.norm(curr_p - prev_p)
                prev_p = curr_p
            self.segment_arclengths_approx.append(length)

        if self.segment_arclengths_approx:
            self.cumulative_arclengths_approx = np.concatenate(([0.0], np.cumsum(self.segment_arclengths_approx)))
            self.total_arclength_approx = self.cumulative_arclengths_approx[-1]
        else: # Should not happen if len(effective_waypoints) >= 2
            self.cumulative_arclengths_approx = [0.0]
            self.total_arclength_approx = 0.0


        if self.total_arclength_approx < 1e-6 and len(effective_waypoints) >=2 :
             rospy.logwarn("Total path arclength is very small. Waypoints might be too close or identical.")
             # Still return True if segments were formed, path might just be tiny.
        
        rospy.loginfo(f"Successfully created Catmull-Rom path with {len(self.bezier_segments)} Bezier segments. "
                      f"Total approx arclength: {self.total_arclength_approx:.2f}m from {len(effective_waypoints)} unique waypoints.")
        return True

    def _get_segment_and_t(self, global_s):
        """Helper to find segment index and local t for a global arclength s."""
        if not self.bezier_segments or self.total_arclength_approx < 1e-9:
            return 0, 0.0 # Default to start of first segment if path is invalid

        target_s = np.clip(global_s, 0, self.total_arclength_approx)
        
        segment_idx = np.searchsorted(self.cumulative_arclengths_approx, target_s, side='right') - 1
        segment_idx = np.clip(segment_idx, 0, len(self.bezier_segments) - 1)

        s_at_segment_start = self.cumulative_arclengths_approx[segment_idx]
        s_into_segment = target_s - s_at_segment_start
        current_segment_length = self.segment_arclengths_approx[segment_idx]

        if current_segment_length < 1e-6:
            t_local = 0.0 # Or 1.0 if s_into_segment is also tiny, indicating end of zero-length segment
        else:
            t_local = s_into_segment / current_segment_length
        
        t_local = np.clip(t_local, 0.0, 1.0)
        return segment_idx, t_local

    def get_point_on_path(self, global_s):
        if not self.bezier_segments: return np.array([self.x, self.y]) # Return current if no path

        segment_idx, t_local = self._get_segment_and_t(global_s)
        segment_points = self.bezier_segments[segment_idx]
        return cubic_bezier_point(*segment_points, t_local)

    def get_derivative_on_path(self, global_s): # Returns (dx/ds, dy/ds)
        if not self.bezier_segments: return np.array([1.0, 0.0]) # Default derivative

        segment_idx, t_local = self._get_segment_and_t(global_s)
        segment_points = self.bezier_segments[segment_idx]
        
        deriv_wrt_t_local = cubic_bezier_derivative(*segment_points, t_local) # This is (dx/dt_local, dy/dt_local)
        
        # Magnitude of derivative is ds/dt_local (speed along curve w.r.t. t_local)
        ds_dt_local = np.linalg.norm(deriv_wrt_t_local)
        
        if ds_dt_local < 1e-6: # Avoid division by zero; e.g. at a cusp or tiny segment
            # Try to get derivative from a slightly different t or return a default
            if t_local < 0.5: # Try t_local + epsilon
                deriv_wrt_t_local_eps = cubic_bezier_derivative(*segment_points, min(1.0, t_local + 0.01))
                ds_dt_local_eps = np.linalg.norm(deriv_wrt_t_local_eps)
                if ds_dt_local_eps > 1e-6: return deriv_wrt_t_local_eps / ds_dt_local_eps
            else: # Try t_local - epsilon
                deriv_wrt_t_local_eps = cubic_bezier_derivative(*segment_points, max(0.0, t_local - 0.01))
                ds_dt_local_eps = np.linalg.norm(deriv_wrt_t_local_eps)
                if ds_dt_local_eps > 1e-6: return deriv_wrt_t_local_eps / ds_dt_local_eps
            # If still zero, use a heuristic like aiming at next waypoint if possible or default
            rospy.logwarn_throttle(1.0, f"ds/dt_local is near zero at s={global_s:.2f}, t={t_local:.2f} on segment {segment_idx}. Using default derivative.")
            if segment_idx + 1 < len(self.bezier_segments) : # Aim for start of next segment
                 next_pt_dir = self.bezier_segments[segment_idx+1][0] - cubic_bezier_point(*segment_points, t_local)
                 norm_next_pt_dir = np.linalg.norm(next_pt_dir)
                 if norm_next_pt_dir > 1e-6: return next_pt_dir / norm_next_pt_dir
            return np.array([1.0, 0.0]) # Default if all else fails
            
        return deriv_wrt_t_local / ds_dt_local # (dx/ds, dy/ds)

    def find_closest_point_on_path(self, x, y): # Renamed
        if not self.bezier_segments or self.total_arclength_approx < 1e-9:
            return 0.0

        robot_pos = np.array([x,y])
        min_dist_sq_overall = float('inf')
        closest_s_overall = 0.0
        
        # Number of samples per segment for coarse search.
        # For better accuracy, a numerical optimization (e.g., scipy.optimize) per segment
        # would be needed, minimizing distance from (x,y) to Bezier(t).
        # This discrete search is a simpler first step.
        num_search_samples_per_segment = 50 

        for i, segment_def_points in enumerate(self.bezier_segments):
            # P_start, CP1, CP2, P_end = segment_def_points
            
            # Search for the t that minimizes distance on this segment
            current_min_dist_sq_segment = float('inf')
            best_t_on_segment = 0.0

            for k_sample in range(num_search_samples_per_segment + 1):
                t = float(k_sample) / num_search_samples_per_segment
                pt_on_curve = cubic_bezier_point(*segment_def_points, t)
                dist_sq = np.sum((pt_on_curve - robot_pos)**2) # sum of squared differences
                
                if dist_sq < current_min_dist_sq_segment:
                    current_min_dist_sq_segment = dist_sq
                    best_t_on_segment = t
            
            # If this segment contains a point closer than any found so far
            if current_min_dist_sq_segment < min_dist_sq_overall:
                min_dist_sq_overall = current_min_dist_sq_segment
                # Convert local t on segment to global s, using approximated arclengths
                s_on_segment = best_t_on_segment * self.segment_arclengths_approx[i]
                closest_s_overall = self.cumulative_arclengths_approx[i] + s_on_segment
        
        return closest_s_overall


    def mpc_control(self, x, y, theta):
        if not self.bezier_segments or self.total_arclength_approx < 1e-9 :
            rospy.logwarn_throttle(1.0, "Path is not defined or too short. Cannot perform MPC control.")
            return 0.0, 0.0

        closest_s = self.find_closest_point_on_path(x, y)

        lookahead_s = closest_s + self.lookahead_distance
        lookahead_s = np.clip(lookahead_s, 0, self.total_arclength_approx) # Clamp

        lookahead_pt = self.get_point_on_path(lookahead_s)
        lookahead_x, lookahead_y = lookahead_pt[0], lookahead_pt[1]

        final_goal_pt = self.get_point_on_path(self.total_arclength_approx)
        final_goal_x, final_goal_y = final_goal_pt[0], final_goal_pt[1]
        distance_to_final_goal = np.sqrt((x - final_goal_x)**2 + (y - final_goal_y)**2)
        
        # Refined goal check:
        # Consider robot "at goal" if it's close to the final point AND
        # its projection (closest_s) is also near the end of the total path length.
        is_near_end_point = distance_to_final_goal < self.distance_threshold
        is_near_end_of_s = abs(self.total_arclength_approx - closest_s) < self.distance_threshold * 1.5 # Allow slightly larger s-error
        
        if is_near_end_point and is_near_end_of_s:
            rospy.loginfo("Reached the final goal!")
            self.reached_goal = True
            return 0.0, 0.0

        # Derivative (dx/ds, dy/ds) at the lookahead point for path tangent
        path_derivative_at_lookahead = self.get_derivative_on_path(lookahead_s)
        dx_ds, dy_ds = path_derivative_at_lookahead[0], path_derivative_at_lookahead[1]
        
        angle_to_goal = np.arctan2(dy_ds, dx_ds) # Path tangent angle
            
        heading_error = angle_to_goal - theta
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

        x_error_lookahead = lookahead_x - x
        y_error_lookahead = lookahead_y - y
        # This distance_error is to the lookahead point, not necessarily cross-track error.
        distance_error = np.sqrt(x_error_lookahead**2 + y_error_lookahead**2)

        heading_threshold = 0.1 
        linear_speed_kp = 1.8 
        angular_speed_kp = 1.8

        if abs(heading_error) > heading_threshold:
            v = self.v_max * 0.2 # Reduced speed for turning
            if distance_error < self.lookahead_distance * 0.3 and abs(heading_error) > np.pi/4 : # Very close but bad angle
                 v = 0.0 # Stop and turn
            omega = angular_speed_kp * heading_error
        else:
            v = linear_speed_kp * distance_error 
            omega = angular_speed_kp * heading_error

        v, omega = self.smooth_velocity(v, omega)

        if self.start_time is not None:
            current_time = rospy.Time.now().to_sec()
            elapsed_time = current_time - self.start_time
            if elapsed_time < self.ramp_up_duration:
                ramp_factor = self.smooth_ramp_factor(elapsed_time / self.ramp_up_duration)
                v *= ramp_factor
                omega *= ramp_factor

        if distance_to_final_goal < self.ramp_down_distance: # Using distance to geometric end point for ramp down
            ramp_down_factor = self.smooth_ramp_factor(distance_to_final_goal / self.ramp_down_distance)
            v *= ramp_down_factor
            omega *= ramp_down_factor 

        v = np.clip(v, self.v_min, self.v_max)
        omega = np.clip(omega, self.omega_min, self.omega_max)

        if self.start_time is not None:
            current_time_log = rospy.Time.now().to_sec()
            time_elapsed_log = current_time_log - self.start_time
            
            dt_accel = time_elapsed_log - self.time_data[-1] if self.time_data else time_elapsed_log
            if dt_accel > 1e-6:
                linear_acceleration = (v - self.last_v) / dt_accel
                angular_acceleration = (omega - self.last_omega) / dt_accel
            else:
                linear_acceleration = self.acceleration_data[-1] if self.acceleration_data else 0.0
                angular_acceleration = self.angular_acceleration_data[-1] if self.angular_acceleration_data else 0.0
            
            self.acceleration_data.append(linear_acceleration)
            self.angular_acceleration_data.append(angular_acceleration)
            self.velocity_data.append(v)
            self.angular_velocity_data.append(omega)
            self.time_data.append(time_elapsed_log)

        self.last_v = v
        self.last_omega = omega
        return v, omega

class GUI:
    def __init__(self, master, mpc_controller):
        self.master = master
        master.title("MiR100 Catmull-Rom Spline Tracking (Parametric + Map)") # Title updated

        self.mpc_controller = mpc_controller

        self.start_button = tk.Button(master, text="Start Tracking", command=self.start_control_loop)
        self.start_button.grid(row=0, column=0)
        self.stop_button = tk.Button(master, text="Stop Tracking", command=self.stop_control_loop, state=tk.DISABLED)
        self.stop_button.grid(row=0, column=1)
        self.clear_button = tk.Button(master, text="Clear Waypoints", command=self.clear_waypoints)
        self.clear_button.grid(row=0, column=2)

        self.result_label = tk.Label(master, text="")
        self.result_label.grid(row=1, column=0, columnspan=3)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.control_loop_running = False

        self.tabControl = ttk.Notebook(master)
        self.trajectory_tab = tk.Frame(self.tabControl)
        self.velocity_tab = tk.Frame(self.tabControl)
        self.acceleration_tab = tk.Frame(self.tabControl)
        self.tabControl.add(self.trajectory_tab, text='Trajectory')
        self.tabControl.add(self.velocity_tab, text='Velocity')
        self.tabControl.add(self.acceleration_tab, text='Acceleration')
        self.tabControl.grid(row=2, column=0, columnspan=3)

        self.fig_trajectory, self.ax_trajectory = plt.subplots()
        try:
            map_img = plt.imread("static/map_image.png")
            map_metadata = {"width": 400, "height": 400, "resolution": 0.05000000074505806,
                            "origin_x": 0.0, "origin_y": 0.0} 
            extent = [
                map_metadata["origin_x"],
                map_metadata["origin_x"] + map_metadata["width"] * map_metadata["resolution"],
                map_metadata["origin_y"],
                map_metadata["origin_y"] + map_metadata["height"] * map_metadata["resolution"]
            ]
            self.ax_trajectory.imshow(map_img, extent=extent, origin='lower', aspect='equal', zorder=0, cmap='gray')
            self.ax_trajectory.set_xlim([extent[0], extent[1]])
            self.ax_trajectory.set_ylim([extent[2], extent[3]])
        except Exception as e:
            rospy.logwarn(f"Map file 'static/map_image.png' not found or error: {e}. Displaying without map background.")
            self.ax_trajectory.set_xlim([-5, 25]); self.ax_trajectory.set_ylim([-5, 25])

        self.ax_trajectory.set_xlabel("X (m)"); self.ax_trajectory.set_ylabel("Y (m)")
        self.ax_trajectory.set_title("Robot Trajectory")
        self.line, = self.ax_trajectory.plot([], [], label="Robot Path", zorder=2)
        # Waypoints are now list of np.arrays. Need to unpack for plotting.
        self.waypoints_plot, = self.ax_trajectory.plot([], [], 'bo-', label="Waypoints", zorder=3, markersize=5, linewidth=1)
        self.robot_plot, = self.ax_trajectory.plot([], [], 'go', label="Robot", markersize=10, zorder=4)
        self.spline_plot, = self.ax_trajectory.plot([], [], 'm-', label="Path (Catmull-Rom)", linewidth=2, zorder=2) 
        self.lookahead_plot, = self.ax_trajectory.plot([], [], 'yo', label="Lookahead Point", markersize=5, zorder=3)
        self.ax_trajectory.legend(); self.ax_trajectory.axis('equal')

        self.canvas_trajectory = FigureCanvasTkAgg(self.fig_trajectory, master=self.trajectory_tab)
        self.canvas_trajectory.draw(); self.canvas_trajectory.get_tk_widget().grid(row=0, column=0, columnspan=3)
        self.fig_trajectory.canvas.mpl_connect('button_press_event', self.onclick)

        # Velocity and Acceleration plots remain the same structure
        self.fig_velocity, self.ax_velocity = plt.subplots()
        self.ax_velocity.set_xlabel("Time (s)"); self.ax_velocity.set_ylabel("Velocity (m/s, rad/s)")
        self.ax_velocity.set_title("Robot Velocity")
        self.line_linear_velocity, = self.ax_velocity.plot([], [], label="Linear Velocity (Filtered)")
        self.line_angular_velocity, = self.ax_velocity.plot([], [], label="Angular Velocity (Filtered)")
        self.ax_velocity.legend()
        self.canvas_velocity = FigureCanvasTkAgg(self.fig_velocity, master=self.velocity_tab)
        self.canvas_velocity.draw(); self.canvas_velocity.get_tk_widget().grid(row=0, column=0, columnspan=3)

        self.fig_acceleration, self.ax_acceleration = plt.subplots()
        self.ax_acceleration.set_xlabel("Time (s)"); self.ax_acceleration.set_ylabel("Acceleration (m/s^2, rad/s^2)")
        self.ax_acceleration.set_title("Robot Acceleration")
        self.line_linear_acceleration, = self.ax_acceleration.plot([], [], label="Linear Acceleration")
        self.line_angular_acceleration, = self.ax_acceleration.plot([], [], label="Angular Acceleration")
        self.ax_acceleration.legend()
        self.canvas_acceleration = FigureCanvasTkAgg(self.fig_acceleration, master=self.acceleration_tab)
        self.canvas_acceleration.draw(); self.canvas_acceleration.get_tk_widget().grid(row=0, column=0, columnspan=3)


    def onclick(self, event):
        if event.inaxes == self.ax_trajectory and not self.control_loop_running:
            if event.xdata is not None and event.ydata is not None:
                x, y = event.xdata, event.ydata
                
                if not self.mpc_controller.waypoints and self.mpc_controller.current_pose is None:
                    print("Waiting for initial robot pose to set the first waypoint via amcl_pose.")
                    self.result_label.config(text="Wait for initial pose before adding waypoints.")
                    return

                # If waypoints list is empty AND current_pose is available, it means pose_callback
                # already added the first waypoint. A click means adding a *second* waypoint.
                # If waypoints list is NOT empty, this click adds another waypoint.
                self.mpc_controller.waypoints.append(np.array([x, y]))
                self.update_waypoint_plot() # This will also try to draw the path preview
                self.result_label.config(text=f"Added waypoint {len(self.mpc_controller.waypoints)} at ({x:.2f}, {y:.2f})")
                print(f"Added waypoint at x={x:.2f}, y={y:.2f}. Total: {len(self.mpc_controller.waypoints)}")

    def start_control_loop(self):
        if not self.mpc_controller.calculate_path(): # Use new method
            self.result_label.config(text="Error: Could not create path. Need at least 2 unique waypoints.")
            return

        self.start_button.config(state=tk.DISABLED); self.stop_button.config(state=tk.NORMAL)
        self.clear_button.config(state=tk.DISABLED)
        self.result_label.config(text="Tracking started...")

        self.mpc_controller.reached_goal = False
        self.mpc_controller.velocity_data = []; self.mpc_controller.angular_velocity_data = []
        self.mpc_controller.acceleration_data = []; self.mpc_controller.angular_acceleration_data = []
        self.mpc_controller.time_data = []
        self.mpc_controller.last_v = 0.0; self.mpc_controller.last_omega = 0.0
        self.mpc_controller.start_time = rospy.Time.now().to_sec()

        self.control_loop_running = True
        self.update_plot()

    def stop_control_loop(self):
        self.control_loop_running = False
        self.start_button.config(state=tk.NORMAL); self.stop_button.config(state=tk.DISABLED)
        self.clear_button.config(state=tk.NORMAL)
        self.send_command(0.0, 0.0)
        self.result_label.config(text="Tracking stopped.")

    def clear_waypoints(self):
        if self.control_loop_running: self.stop_control_loop()

        self.mpc_controller.waypoints = [] # Clear waypoints list
        if self.mpc_controller.current_pose is not None: # Add current pose as first waypoint if available
            self.mpc_controller.waypoints.append(np.array([self.mpc_controller.x, self.mpc_controller.y]))
            self.result_label.config(text=f"Waypoints cleared. Robot start: ({self.mpc_controller.x:.2f}, {self.mpc_controller.y:.2f})")
        else:
            self.result_label.config(text="Waypoints cleared. Waiting for initial pose.")

        self.mpc_controller.bezier_segments = [] # Clear path data
        self.mpc_controller.segment_arclengths_approx = []
        self.mpc_controller.cumulative_arclengths_approx = []
        self.mpc_controller.total_arclength_approx = 0.0
        self.mpc_controller.trajectory_x = []; self.mpc_controller.trajectory_y = []

        self.update_waypoint_plot() # Will clear spline plot as bezier_segments is empty
        self.line.set_data([], [])
        if self.mpc_controller.current_pose is not None:
            self.robot_plot.set_data([self.mpc_controller.x], [self.mpc_controller.y])
        else:
            self.robot_plot.set_data([], [])
        self.lookahead_plot.set_data([], [])
        self.canvas_trajectory.draw_idle()

        # Clear velocity and acceleration plots
        self.mpc_controller.velocity_data = []; self.mpc_controller.angular_velocity_data = []
        self.mpc_controller.acceleration_data = []; self.mpc_controller.angular_acceleration_data = []
        self.mpc_controller.time_data = []
        for line_obj in [self.line_linear_velocity, self.line_angular_velocity, 
                         self.line_linear_acceleration, self.line_angular_acceleration]:
            line_obj.set_data([], [])
        for ax, canvas in [(self.ax_velocity, self.canvas_velocity), 
                           (self.ax_acceleration, self.canvas_acceleration)]:
            ax.relim(); ax.autoscale_view(); canvas.draw_idle()

    def update_waypoint_plot(self):
        # Unpack waypoints for plotting
        if self.mpc_controller.waypoints:
            wp_x = [wp[0] for wp in self.mpc_controller.waypoints]
            wp_y = [wp[1] for wp in self.mpc_controller.waypoints]
            self.waypoints_plot.set_data(wp_x, wp_y)
        else:
            self.waypoints_plot.set_data([], [])

        # Path preview if not tracking
        if not self.control_loop_running:
            if self.mpc_controller.calculate_path() and self.mpc_controller.bezier_segments:
                all_spline_x, all_spline_y = [], []
                for seg_points in self.mpc_controller.bezier_segments:
                    num_samples = 20 # Samples per Bezier segment for plotting
                    for k_sample in range(num_samples + 1):
                        t = float(k_sample) / num_samples
                        pt = cubic_bezier_point(*seg_points, t) # Pass all 4 points of segment
                        all_spline_x.append(pt[0])
                        all_spline_y.append(pt[1])
                self.spline_plot.set_data(all_spline_x, all_spline_y)
            else:
                 self.spline_plot.set_data([], []) # Clear if not enough points or error
        self.canvas_trajectory.draw_idle()

    def update_plot(self):
        if not self.control_loop_running: return

        if self.mpc_controller.current_pose is None:
            self.result_label.config(text="Waiting for initial pose from /amcl_pose...")
            self.master.after(100, self.update_plot); return

        x, y, theta = self.mpc_controller.x, self.mpc_controller.y, self.mpc_controller.theta
        v, omega = self.mpc_controller.mpc_control(x, y, theta)
        self.send_command(v, omega)

        self.line.set_data(self.mpc_controller.trajectory_x, self.mpc_controller.trajectory_y)
        self.robot_plot.set_data([x], [y])

        if self.mpc_controller.bezier_segments: # Path exists
            # Spline path is plotted once at start_control_loop or by update_waypoint_plot
            # Update lookahead point
            closest_s = self.mpc_controller.find_closest_point_on_path(x, y)
            lookahead_s = np.clip(closest_s + self.mpc_controller.lookahead_distance, 0, self.mpc_controller.total_arclength_approx)
            lookahead_pt = self.mpc_controller.get_point_on_path(lookahead_s)
            self.lookahead_plot.set_data([lookahead_pt[0]], [lookahead_pt[1]])
        
        self.canvas_trajectory.draw_idle()

        # Velocity and Acceleration plots (same logic as before)
        if len(self.mpc_controller.time_data) > 1:
            filtered_v = self.mpc_controller.lowpass_filter(np.array(self.mpc_controller.velocity_data))
            filtered_w = self.mpc_controller.lowpass_filter(np.array(self.mpc_controller.angular_velocity_data))
            self.line_linear_velocity.set_data(self.mpc_controller.time_data, filtered_v)
            self.line_angular_velocity.set_data(self.mpc_controller.time_data, filtered_w)
            self.ax_velocity.relim(); self.ax_velocity.autoscale_view(); self.canvas_velocity.draw_idle()

        if len(self.mpc_controller.time_data) > 1:
            self.line_linear_acceleration.set_data(self.mpc_controller.time_data, self.mpc_controller.acceleration_data)
            self.line_angular_acceleration.set_data(self.mpc_controller.time_data, self.mpc_controller.angular_acceleration_data)
            self.ax_acceleration.relim(); self.ax_acceleration.autoscale_view(); self.canvas_acceleration.draw_idle()

        self.master.update_idletasks()

        if not self.mpc_controller.reached_goal:
            update_interval_ms = max(10, int(self.mpc_controller.dt * 1000 * 0.8))
            self.master.after(update_interval_ms, self.update_plot)
        else:
            self.result_label.config(text="Reached final goal!")
            self.stop_control_loop()

    def send_command(self, v, omega):
        twist_msg = Twist(); twist_msg.linear.x = v; twist_msg.angular.z = omega
        self.cmd_vel_pub.publish(twist_msg)

def ros_spin():
    try: rospy.spin()
    except rospy.ROSInterruptException: print("ROS spin thread interrupted.")
    except Exception as e: print(f"Exception in ROS spin thread: {e}")

if __name__ == '__main__':
    gui_instance = None
    try:
        rospy.init_node('spline_tracking_node_catmull_rom') # Node name updated
        
        dt = 0.05; v_max = 0.3; v_min = -0.3; omega_max = 0.5; omega_min = -0.5
        lookahead_distance = 0.2; filter_order = 3; cutoff_freq = 1.5

        controller = MPCController(dt, v_max, v_min, omega_max, omega_min,
                                   lookahead_distance, filter_order, cutoff_freq)
        
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, controller.pose_callback, queue_size=1)

        root = tk.Tk()
        gui_instance = GUI(root, controller)
        
        ros_thread = threading.Thread(target=ros_spin, daemon=True); ros_thread.start()
        root.mainloop()

    except rospy.ROSInterruptException: print("ROS node interrupted by ROS.")
    except KeyboardInterrupt: print("Script terminated by user (Ctrl+C).")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        import traceback; traceback.print_exc()
    finally:
        print("Shutting down...")
        if gui_instance and hasattr(gui_instance, 'cmd_vel_pub') and gui_instance.cmd_vel_pub:
            try:
                gui_instance.send_command(0.0, 0.0)
                rospy.sleep(0.2)
                print("Zero velocity command sent via GUI publisher.")
            except Exception as e_final_cmd: print(f"Error sending final zero cmd via GUI: {e_final_cmd}")
        elif not rospy.is_shutdown(): # If GUI publisher failed or not available, try a new one
            final_pub = None
            try:
                final_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
                rospy.sleep(0.5) # Allow connection
                final_pub.publish(Twist())
                rospy.sleep(0.2)
                print("Zero velocity command sent via temporary publisher.")
            except Exception as e_final_cmd_temp: print(f"Error sending final zero cmd via temp pub: {e_final_cmd_temp}")
        
        if not rospy.is_shutdown(): rospy.signal_shutdown("Application exit")
        print("Exiting application.")