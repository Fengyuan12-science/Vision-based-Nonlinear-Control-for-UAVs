#!/usr/bin/env python3
import rospy, math
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State

class OffboardTakeoffVel:
    def __init__(self):
        rospy.init_node('takeoff_offboard_vel')
        self.state = State()
        self.alt = 0.0

        self.alt_target = float(rospy.get_param('~target_alt', 1.5))
        self.rate_hz    = float(rospy.get_param('~rate', 30.0))
        self.climb_vz   = float(rospy.get_param('~climb_vz', 0.8))  # m/s
        self.vmax_xy    = float(rospy.get_param('~vmax_xy', 1.0))

        rospy.Subscriber('/mavros/state', State, self.state_cb)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=20)
        self.arm_srv  = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        rate = rospy.Rate(self.rate_hz)

        # 预热：先连续发 2-3 秒 0 速度，建立 OFFBOARD 流
        warm = TwistStamped()
        warm.header.stamp = rospy.Time.now()
        for _ in range(int(self.rate_hz*3)):
            self.vel_pub.publish(warm)
            rate.sleep()

        # ARM + OFFBOARD
        self.arm_srv(True)
        self.mode_srv(0, 'OFFBOARD')
        rospy.loginfo("Armed & OFFBOARD. Taking off by velocity to %.2fm", self.alt_target)

        # 爬升阶段：仅给 z 速度，其它由避障节点负责
        tol = 0.10
        while not rospy.is_shutdown() and self.alt < self.alt_target - tol:
            cmd = TwistStamped()
            cmd.header.stamp = rospy.Time.now()
            cmd.twist.linear.z = self.climb_vz
            self.vel_pub.publish(cmd)
            rate.sleep()

        rospy.loginfo("Reached altitude %.2fm, holding z", self.alt)

        # 保持阶段：持续发 0 的 z 速度（不间断流）；x/y 让避障节点来写
        while not rospy.is_shutdown():
            cmd = TwistStamped()
            cmd.header.stamp = rospy.Time.now()
            cmd.twist.linear.z = 0.0
            self.vel_pub.publish(cmd)
            rate.sleep()

    def state_cb(self, msg): self.state = msg
    def pose_cb(self, msg):  self.alt = msg.pose.position.z

if __name__ == '__main__':
    OffboardTakeoffVel()

