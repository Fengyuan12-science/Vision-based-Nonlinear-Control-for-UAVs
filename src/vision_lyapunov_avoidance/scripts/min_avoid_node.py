#!/usr/bin/env python3
import rospy, numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped, PoseStamped, Point
from std_msgs.msg import Float32
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import euler_from_quaternion

class MinimalAvoidAllInOne:
    def __init__(self):
        rospy.init_node('min_avoid_node')
                
        self.mav_frame = rospy.get_param('/mavros/setpoint_velocity/mav_frame', 'BODY_NED')
        self._vz_sign  = -1.0 if 'BODY_NED' in self.mav_frame.upper() else 1.0
        rospy.loginfo("cmd_vel frame = %s, vz_sign = %+.1f", self.mav_frame, self._vz_sign)

        #Range/breaking symmetry
        self.range_cap      = float(rospy.get_param('~range_cap', 8.0))
        self.asym_bias_deg  = float(rospy.get_param('~asym_bias_deg', 4.0))

        # Basic parameters
        
        self.depth_topic    = rospy.get_param('~depth_topic', '/iris/depth_camera/depth/image_raw')
        self.safe_distance  = float(rospy.get_param('~safe_distance', 1.8))
        self.release_margin = float(rospy.get_param('~release_margin', 0.4))
        self.avoid_speed    = float(rospy.get_param('~avoid_speed', 0.7))
        self.forward_speed  = float(rospy.get_param('~forward_speed', 0.30))
        self.window         = int(rospy.get_param('~window', 120))
        self.alpha          = float(rospy.get_param('~alpha', 0.85))
        self.rate_hz        = float(rospy.get_param('~rate', 30.0))
        self.vmax_xy        = float(rospy.get_param('~vmax_xy', 1.0))

        #(Lyapunov/CBF) Reaction formula
        self.react_tau      = float(rospy.get_param('~react_tau', 0.8))
        self.k_side         = float(rospy.get_param('~k_side', 1.1))
        self.k_forward      = float(rospy.get_param('~k_forward', 0.6))
        self.flip_side      = bool(rospy.get_param('~flip_side', False))
        self.use_7_sectors  = bool(rospy.get_param('~use_7_sectors', True))
        self.ttc_thresh     = float(rospy.get_param('~ttc_thresh', 3.0))
        self.cbf_relax_eps  = float(rospy.get_param('~cbf_relax_eps', 0.05))

        #Predictive Planning (Short Time Domain)
        self.pred_enabled   = bool(rospy.get_param('~pred_enabled', True))
        self.mpc_horizon    = float(rospy.get_param('~mpc_horizon', 1.2))
        self.mpc_num_angles = int(rospy.get_param('~mpc_num_angles', 11))
        self.mpc_max_deg    = float(rospy.get_param('~mpc_max_deg', 40.0))
        self.mpc_w_clear    = float(rospy.get_param('~mpc_w_clear', 1.0))
        self.mpc_w_prog     = float(rospy.get_param('~mpc_w_prog', 0.8))
        self.mpc_v_scale    = float(rospy.get_param('~mpc_v_scale', 1.0))

        # Takeoff/Mission
        self.alt_target     = float(rospy.get_param('~target_alt', 1.5))
        self.climb_vz       = float(rospy.get_param('~climb_vz', 0.8))
        self.loop_mission   = bool(rospy.get_param('~loop_mission', True))
        self.mission_time   = float(rospy.get_param('~mission_time', 40.0))

        # Communication
        self.bridge   = CvBridge()
        self.v_pub    = rospy.Publisher('/lyapunov_value', Float32, queue_size=10)
        self.near_pub = rospy.Publisher('/nearest_obstacle', Float32, queue_size=10)
        self.jerk_pub = rospy.Publisher('/cmd_jerk', Float32, queue_size=10)
        self.m_pub    = rospy.Publisher('/avoid_vis', MarkerArray, queue_size=1)
        self.plan_pub = rospy.Publisher('/avoid_plan', MarkerArray, queue_size=1)
        self.cmd_pub  = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=20)
        self.pos_pub  = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)  # ★NEW: 位置setpoint

        rospy.Subscriber(self.depth_topic, Image, self.depth_cb, queue_size=1)

        self.state = State()
        self.alt   = 0.0
        self.yaw   = 0.0
        self.home  = None

        rospy.Subscriber('/mavros/state', State, self.state_cb)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)
        self.arm_srv  = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        # State quantity
        self.d_filt = None
        self.d_use  = None
        self.phase  = 'INIT'
        self.last_cmd_xy = (0.0, 0.0)
        self.u_filt = None
        self.u_prev = None

        self.rate = rospy.Rate(self.rate_hz)
        if self.depth_topic.endswith('/image_raw') and '/depth/' not in self.depth_topic:
            rospy.logwarn("depth_topic seems like RGB not depth：%s", self.depth_topic)
        rospy.loginfo('MinimalAvoidAllInOne subscribing %s', self.depth_topic)
        self.main_loop()

    # Callback
    def state_cb(self, msg): self.state = msg
    def pose_cb(self, msg):
        self.alt = msg.pose.position.z
        if self.home is None:
            self.home = (msg.pose.position.x, msg.pose.position.y)
        q = msg.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    # Deep sector
    def _sector_depths(self, depth):
        h, w = depth.shape[:2]; win = self.window; cx = w // 2
        y0, y1 = max(0, h//2 - win//2), min(h, h//2 + win//2)
        def mean_box(xc):
            x0, x1 = max(0, xc - win//2), min(w, xc + win//2)
            roi = depth[y0:y1, x0:x1]
            return float(np.nanmean(roi))
        if self.use_7_sectors:
            offsets = [-3, -2, -1, 0, 1, 2, 3]
            xs = [cx + k*(win//2) for k in offsets]
            d_list = [mean_box(int(x)) for x in xs]
            thetas = np.deg2rad([-30.0, -18.0, -9.0, 0.0, 9.0, 18.0, 30.0])
        else:
            xs = [cx - win, cx, cx + win]
            d_list = [mean_box(int(x)) for x in xs]
            thetas = np.deg2rad([-30.0, 0.0, 30.0])
        return d_list, thetas

    def _depth_at_angle(self, d_list, thetas, theta_query):
        vals = np.array([d if (np.isfinite(d) and d > 0) else self.range_cap for d in d_list], dtype=np.float32)
        ths  = np.array(thetas, dtype=np.float32)
        theta_q = np.clip(theta_query, ths.min(), ths.max())
        return float(np.interp(theta_q, ths, vals))

    # Image callback
    def depth_cb(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1'); scale = 1.0
        except Exception:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            scale = 0.001 if hasattr(depth, 'dtype') and 'uint16' in str(depth.dtype) else 1.0

        d_list, thetas = self._sector_depths(depth)
        def vld(x): return np.isfinite(x) and x > 0
        has_any = any(vld(d) for d in d_list)

        if has_any:
            d_list = [(d*scale if vld(d) else 0.0) for d in d_list]
            dC = d_list[len(d_list)//2]
            self.d_filt = dC if self.d_filt is None else (self.alpha*self.d_filt + (1.0-self.alpha)*dC)
            self.d_use  = self.d_filt
        else:
            self.d_use  = None

        valids  = [d for d in d_list if np.isfinite(d) and d > 0]
        nearest = min(valids) if valids else self.range_cap * 0.98
        nearest = float(np.clip(nearest, 0.05, self.range_cap))
        self.near_pub.publish(Float32(nearest))

        d_for_V = min(self.d_use, self.range_cap) if (self.d_use is not None and np.isfinite(self.d_use) and self.d_use > 0) else nearest
        V = 1.0 / (d_for_V + 0.1)
        self.v_pub.publish(Float32(V))

        if self.phase == 'NAV':
            # Reaction formula
            safe_eff = self.safe_distance + self.react_tau * self.forward_speed
            side_gain = -self.k_side if self.flip_side else self.k_side

            bias = np.deg2rad(self.asym_bias_deg)
            vy_des = 0.0
            for d, th in zip(d_list, thetas):
                th_biased = th + np.sign(th) * bias
                val = d if (np.isfinite(d) and d > 0) else self.range_cap
                vy_des += side_gain * (val * np.sin(th_biased))
            vy_des /= max(1.0, float(len(d_list)))
            if self.d_use is not None and self.d_use < (self.safe_distance + 0.4):
                vy_des *= 1.4

            if self.d_use is None:
                vx_des = self.forward_speed
            else:
                vx_des = self.k_forward * max(0.0, self.d_use - safe_eff)
                vx_des = min(vx_des, self.forward_speed)
                if vx_des > 0.05:
                    ttc = self.d_use / max(vx_des, 1e-3)
                    if ttc < self.ttc_thresh:
                        vx_des = min(vx_des, 0.10)

            if self.d_use is not None and self.d_use < self.safe_distance:
                vx_des = min(vx_des, 0.10)
                if abs(vy_des) < self.avoid_speed * 0.8:
                    vy_des = np.sign(vy_des if vy_des != 0 else 1.0) * (self.avoid_speed * 0.8)

            u_react = np.array([vx_des, vy_des], dtype=np.float32)

            # Predictive
            if self.pred_enabled:
                u_pred, plan_markers = self._predictive_sample(d_list, thetas)
            else:
                u_pred, plan_markers = u_react.copy(), None

            # merge
            gamma = 0.8 if (self.d_use is not None and self.d_use < self.safe_distance + 0.6) else 0.35
            u_des = (1.0 - gamma) * u_react + gamma * u_pred

            # CBF 
            if self.d_use is not None:
                idx_min = int(np.argmin([d if d > 0 else 1e9 for d in d_list]))
                theta   = thetas[idx_min]
                n       = np.array([np.cos(theta), np.sin(theta)], dtype=np.float32)
                a       = n
                b       = self.alpha * (self.d_use - self.safe_distance) - self.cbf_relax_eps
                viol    = a.dot(u_des) - b
                u       = u_des - (viol / (a.dot(a) + 1e-6)) * a if viol > 0.0 else u_des
            else:
                u = u_des

            # Limiting + smoothing + jerk 
            u[0] = np.clip(u[0], -self.vmax_xy, self.vmax_xy)
            u[1] = np.clip(u[1], -self.vmax_xy, self.vmax_xy)
            if self.u_filt is None: self.u_filt = u.copy()
            beta = 0.75
            self.u_filt = beta*self.u_filt + (1.0-beta)*u

            if self.u_prev is None: self.u_prev = self.u_filt.copy()
            du = self.u_filt - self.u_prev
            du_max = 0.10
            du = np.clip(du, -du_max, du_max)
            u_cmd = self.u_prev + du
            self.jerk_pub.publish(float(np.linalg.norm(u_cmd - self.u_prev) * self.rate_hz))
            self.u_prev = u_cmd

            self.last_cmd_xy = (float(u_cmd[0]), float(u_cmd[1]))

            # Visualization
            self._publish_markers(u_cmd, u_des, n if self.d_use is not None else None)
            self._publish_sectors(d_list, thetas)
            if plan_markers is not None:
                self.plan_pub.publish(plan_markers)

    # Predictability: Candidate sampling + scoring
    def _predictive_sample(self, d_list, thetas):
        angs = np.deg2rad(np.linspace(-self.mpc_max_deg, self.mpc_max_deg, self.mpc_num_angles))
        v_nom = self.forward_speed * self.mpc_v_scale
        horizon = self.mpc_horizon
        w_clear, w_prog = self.mpc_w_clear, self.mpc_w_prog

        best_cost, best_vx, best_vy, best_idx = 1e9, 0.0, 0.0, -1
        mk = MarkerArray(); base_frame = "base_link"

        for i, th in enumerate(angs):
            d0 = self._depth_at_angle(d_list, thetas, th)
            d_min_pred = d0 - v_nom * horizon
            penal = 0.0
            if d_min_pred < (self.safe_distance + 0.2):
                penal = 100.0 * (self.safe_distance + 0.2 - d_min_pred)
            cost = w_clear * (1.0 / (d0 + 0.1)) - w_prog * np.cos(th) + penal

            if cost < best_cost:
                best_cost, best_vx, best_vy, best_idx = cost, v_nom*np.cos(th), v_nom*np.sin(th), i

            m = Marker(); m.header.frame_id = base_frame; m.id = 100+i
            m.type = Marker.ARROW; m.action = Marker.ADD
            m.scale.x, m.scale.y, m.scale.z = 0.02, 0.04, 0.04
            c = float(np.tanh(cost / 3.0))
            m.color.r, m.color.g, m.color.b, m.color.a = min(1.0, c), max(0.0, 1.0 - c), 0.2, 0.6
            p0 = Point(0,0,0); L = min(d0, self.range_cap)
            p1 = Point(L*np.cos(th), L*np.sin(th), 0)
            m.points = [p0, p1]; mk.markers.append(m)

        if best_idx >= 0:
            th = angs[best_idx]
            m = Marker(); m.header.frame_id = base_frame; m.id = 1000
            m.type = Marker.ARROW; m.action = Marker.ADD
            m.scale.x, m.scale.y, m.scale.z = 0.05, 0.08, 0.08
            m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.9, 0.1, 0.9
            m.points = [Point(0,0,0), Point(0.8*np.cos(th), 0.8*np.sin(th), 0)]
            mk.markers.append(m)

        return np.array([best_vx, best_vy], dtype=np.float32), mk

    # Visualization
    def _make_arrow(self, mid, frame, p0, p1, rgba):
        m = Marker(); m.header.frame_id = frame; m.id = mid
        m.type = Marker.ARROW; m.action = Marker.ADD
        m.scale.x, m.scale.y, m.scale.z = 0.05, 0.08, 0.08
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        m.points = [p0, p1]; return m

    def _publish_markers(self, u_cmd, u_des, n_vec):
        arr = MarkerArray(); frame = "base_link"; p0 = Point(0,0,0)
        arr.markers.append(self._make_arrow(1, frame, p0, Point(u_cmd[0], u_cmd[1], 0), (0.1,0.8,0.1,1.0)))
        arr.markers.append(self._make_arrow(2, frame, p0, Point(u_des[0], u_des[1], 0), (0.1,0.4,0.9,0.6)))
        if n_vec is not None:
            L = 0.8
            arr.markers.append(self._make_arrow(3, frame, p0, Point(n_vec[0]*L, n_vec[1]*L, 0), (0.9,0.2,0.2,0.8)))
        self.m_pub.publish(arr)

    def _publish_sectors(self, d_list, thetas):
        arr = MarkerArray(); frame = "base_link"
        for i, (d, th) in enumerate(zip(d_list, thetas), start=10):
            if not np.isfinite(d) or d <= 0:  continue
            p0 = Point(0,0,0); p1 = Point(d*np.cos(th), d*np.sin(th), 0)
            m = Marker(); m.header.frame_id=frame; m.id=i
            m.type=Marker.ARROW; m.action=Marker.ADD
            m.scale.x, m.scale.y, m.scale.z = 0.03, 0.06, 0.06
            c = max(0.0, min(1.0, (self.safe_distance+1.0 - d)/(self.safe_distance+1.0)))
            m.color.r, m.color.g, m.color.b, m.color.a = 0.8*c+0.2, 0.8*(1-c)+0.2, 0.2, 0.8
            m.points=[p0,p1]; arr.markers.append(m)
        self.m_pub.publish(arr)

    # main loop (Position takeoff -> Speed navigation)
    def main_loop(self):
        while not rospy.is_shutdown() and not self.state.connected:
            self._send_cmd(0, 0, 0)
            rospy.loginfo_throttle(2.0, "Waiting for MAVROS connection...")
            self.rate.sleep()

        # When this position and posture are available
        rospy.loginfo("Waiting for /mavros/local_position/pose ...")
        pose0 = rospy.wait_for_message('/mavros/local_position/pose', PoseStamped, timeout=5.0)
        x0, y0 = pose0.pose.position.x, pose0.pose.position.y

        # Preheating (2 seconds): Both belong to the setpoint for sending speed and position, ensuring the safety of switching OFFBOARD
        self.phase = 'WARM'
        for _ in range(int(self.rate_hz*2.0)):
            self._send_cmd(0, 0, 0)
            p = PoseStamped(); p.header.stamp = rospy.Time.now()
            p.pose.position.x = x0; p.pose.position.y = y0; p.pose.position.z = max(0.5, self.alt)
            p.pose.orientation = pose0.pose.orientation
            self.pos_pub.publish(p)
            self.rate.sleep()
        try:
            self.mode_srv(0, 'OFFBOARD'); rospy.sleep(0.2)
            self.arm_srv(True)
            rospy.loginfo('OFFBOARD set, armed.')
        except Exception as e:
            rospy.logerr('Arm/Mode failed: %s', e); return

        self.phase = 'TAKEOFF'
        tol = 0.10
        z_cmd = max(self.alt, 0.2)
        while not rospy.is_shutdown() and self.alt < self.alt_target - tol:
            z_cmd = min(self.alt_target, z_cmd + 0.05)  
            p = PoseStamped(); p.header.stamp = rospy.Time.now()
            p.pose.position.x = x0; p.pose.position.y = y0; p.pose.position.z = z_cmd
            p.pose.orientation = pose0.pose.orientation
            self.pos_pub.publish(p)
            self.rate.sleep()
            
        for _ in range(int(self.rate_hz*1.0)):
            p = PoseStamped(); p.header.stamp = rospy.Time.now()
            p.pose.position.x = x0; p.pose.position.y = y0; p.pose.position.z = self.alt_target
            p.pose.orientation = pose0.pose.orientation
            self.pos_pub.publish(p)
            self.rate.sleep()

        rospy.loginfo('Reached altitude %.2fm', self.alt)

        self.phase = 'NAV'
        start_nav = rospy.Time.now()
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_nav).to_sec()
            if self.phase == 'NAV' and elapsed > self.mission_time:
                rospy.loginfo("Switching to RETURN mode")
                self.phase = 'RETURN'

            if self.phase == 'NAV':
                vx, vy = self.last_cmd_xy
                self._send_cmd(vx, vy, 0.0)

            elif self.phase == 'RETURN' and self.home is not None:
                try:
                    cur = rospy.wait_for_message('/mavros/local_position/pose', PoseStamped, timeout=0.2)
                    cx, cy = cur.pose.position.x, cur.pose.position.y
                    hx, hy = self.home
                    dx, dy = hx - cx, hy - cy
                    dist = max(1e-3, np.hypot(dx, dy))
                    self._send_cmd(0.4*dx/dist, 0.4*dy/dist, 0.0)
                except rospy.ROSException:
                    self._send_cmd(0,0,0)
            else:
                self._send_cmd(0,0,0)

            self.rate.sleep()
            
    def _send_cmd(self, vx_body, vy_body, vz):
        cy, sy = np.cos(self.yaw), np.sin(self.yaw)
        vx_enu =  cy*vx_body - sy*vy_body
        vy_enu =  sy*vx_body + cy*vy_body
        cmd = TwistStamped() 
        cmd.header.stamp = rospy.Time.now()
        cmd.twist.linear.x = float(vx_enu)
        cmd.twist.linear.y = float(vy_enu)
        cmd.twist.linear.z = float(self._vz_sign * vz)
        self.cmd_pub.publish(cmd)

if __name__ == '__main__':
    try:
        MinimalAvoidAllInOne()
    except Exception as e:
        rospy.logfatal("min_avoid_node crashed: %s", e); raise

