#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import AccelWithCovarianceStamped
from mocap4r2_msgs.msg import RigidBodies
from ackermann_msgs.msg import AckermannDrive   # wheel speed only


def yaw_from_quat(x, y, z, w):
    """Extract planar yaw from quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_from_yaw(yaw):
    """Create quaternion from planar yaw."""
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return (0.0, 0.0, qz, qw)


def angle_diff(a, b):
    """Shortest difference between two angles."""
    d = a - b
    return (d + math.pi) % (2.0 * math.pi) - math.pi


class EKF:
    """
    EKF state: [x, y, yaw, v, a, yaw_rate]
    Uses a CTRA-style transition (straight approximation when yaw_rate ~ 0).
    """
    def __init__(self, qa, qw, rx, ry, rpsi, P0):
        self.x = np.zeros((6, 1))     # [x, y, yaw, v, a, yaw_rate]
        self.P = np.diag(P0)
        self.R = np.diag([rx**2, ry**2, rpsi**2])
        self.qa, self.qw = qa, qw
        self.I = np.eye(6)

    def _Q(self, dt):
        """Process noise covariance (simple diagonal)."""
        return np.diag([0, 0, 0, 0, (self.qa**2) * dt, (self.qw**2) * dt])

    def _Fx(self, dt):
        """Nonlinear motion model and Jacobian."""
        X, Y, psi, v, a, w = self.x.flatten()
        dt = max(dt, 1e-3)

        # Straight motion approximation
        if abs(w) < 1e-6:
            c, s = math.cos(psi), math.sin(psi)
            Xp = X + (v * dt + 0.5 * a * dt * dt) * c
            Yp = Y + (v * dt + 0.5 * a * dt * dt) * s
            fx = np.array([[Xp], [Yp], [psi], [v + a * dt], [a], [w]])

            F = np.eye(6)
            F[0,3] = c * dt
            F[1,3] = s * dt
            F[0, 4] = 0.5 * c * dt * dt
            F[1, 4] = 0.5 * s * dt * dt
            F[3,4] = dt
            F[2,5] = dt
            return F, fx

        # Turning motion
        s0, c0 = math.sin(psi), math.cos(psi)
        s1, c1 = math.sin(psi + w * dt), math.cos(psi + w * dt)
        w2 = w * w

        Xp = X + (v / w) * (s1 - s0) + (a / w2) * (c1 - c0)
        Yp = Y + (v / w) * (-c1 + c0) + (a / w2) * (s1 - s0)
        fx = np.array([[Xp], [Yp], [psi + w * dt], [v + a * dt], [a], [w]])

        return np.eye(6), fx

    def predict(self, dt):
        """EKF prediction step."""
        F, fx = self._Fx(dt)
        self.x = fx
        self.P = F @ self.P @ F.T + self._Q(dt)

    def update(self, z):
        """EKF measurement update for [x, y, yaw]."""
        H = np.array([[1, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0, 0]])
        z = z.reshape((3, 1))
        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (self.I - K @ H) @ self.P


class Localization(Node):
    def __init__(self):
        super().__init__('localization')

        # parameters (kept similar names)
        self.id = str(self.declare_parameter('target_body_id', '6').value).strip()
        self.pass_pos = bool(self.declare_parameter('use_measurement_position', True).value)
        self.twist_child = bool(self.declare_parameter('twist_in_child_frame', True).value)
        self.no_neg_vx = bool(self.declare_parameter('non_negative_vx', True).value)
        self.deadband = float(self.declare_parameter('deadband', 0.01).value)

        rx = float(self.declare_parameter('meas_noise.x', 0.005).value)
        ry = float(self.declare_parameter('meas_noise.y', 0.005).value)
        rpsi = float(self.declare_parameter('meas_noise.yaw', 0.01).value)
        qa = float(self.declare_parameter('proc_noise.qa', 0.8).value)
        qw = float(self.declare_parameter('proc_noise.qw', 0.6).value)
        P0 = list(self.declare_parameter('P0_diag', [0.1, 0.1, 0.05, 0.5, 0.5, 0.2]).value)

        # algorithm switches
        # use a fixed nominal dt for predictions when needed (no wall-clock math)
        self.nominal_dt = float(self.declare_parameter('nominal_dt', 0.02).value)
        # epsilon to detect whether pose changed between *consecutive* RigidBodies msgs
        self._EPS_CHANGE = float(self.declare_parameter('pose_change_eps', 1e-6).value)
        # speed threshold to decide moving
        self._SPEED_THRESHOLD = float(self.declare_parameter('speed_threshold', 0.1).value)

        # frames
        self.frame_id = 'map'
        self.child_id = 'base_link'

        # EKF
        self.ekf = EKF(qa, qw, rx, ry, rpsi, P0)

        # stored previous mocap pose (concurrent message comparison)
        self.prev_rb_x = None
        self.prev_rb_y = None
        self.prev_rb_yaw = None
        self.prev_z = 0.0

        # last published/known pose used for dead-reckon integration when frozen+moving
        # initialize from EKF state on first message
        self.last_pose_x = None
        self.last_pose_y = None
        self.last_pose_yaw = None

        # wheel speed
        self.last_speed = 0.0

        # CHANGE 1: filtered longitudinal acceleration (low-pass on dv/dt)
        # This reduces noise in acceleration estimates derived from wheel speed.
        self.prev_speed = 0.0
        self.filtered_accel = 0.0
        self.alpha = 0.2  # 0..1 (higher = less smoothing)

        # topics
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(RigidBodies, '/pose_modelcars', self.cb_mocap, qos)
        self.create_subscription(AckermannDrive, '/ackermann_drive_feedback', self.cb_ackermann_feedback, 10)

        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_accel = self.create_publisher(AccelWithCovarianceStamped, '/odom_accel', 10)

        self.get_logger().info("Robust localization started.")

    def cb_ackermann_feedback(self, msg):
        """Store wheel speed feedback (used for velocity + fallback)."""
        self.last_speed = float(msg.speed)

    def _select(self, msg):
        """Pick the target rigid body by name/id, otherwise fallback to first."""
        for rb in msg.rigidbodies:
            if (rb.rigid_body_name or '').strip() == self.id:
                return rb
        if msg.rigidbodies:
            return msg.rigidbodies[0]
        return None

    def _clean(self, x):
        """Deadband small values to reduce jitter in outputs."""
        return 0.0 if abs(x) < self.deadband else x

    def _get_last_yaw(self):
        """Use last published yaw during freeze; otherwise EKF yaw."""
        if self.last_pose_yaw is not None:
            return float(self.last_pose_yaw)
        return float(self.ekf.x[2, 0])

    def cb_mocap(self, msg):
        """Main callback: uses mocap pose updates and handles freeze fallback."""
        rb = self._select(msg)
        if not rb:
            return

        p = rb.pose.position
        q = rb.pose.orientation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

        # First message: initialize all stored values and publish zero-velocity odom
        if self.prev_rb_x is None:
            # initialize EKF pose to mocap
            self.ekf.x[0, 0] = float(p.x)
            self.ekf.x[1, 0] = float(p.y)
            self.ekf.x[2, 0] = float(yaw)

            # store previous mocap reading
            self.prev_rb_x = float(p.x)
            self.prev_rb_y = float(p.y)
            self.prev_rb_yaw = float(yaw)
            self.prev_z = float(p.z)

            # initialize last published pose to current mocap pose
            self.last_pose_x = float(p.x)
            self.last_pose_y = float(p.y)
            self.last_pose_yaw = float(yaw)

            # publish initial odom (no velocities)
            self._publish_pose(msg, rb, 0.0, 0.0, 0.0, 0.0)
            return

        # Compute pose difference
        # Check whether pose changed between *this* message and *previous* message
        dx = float(p.x) - float(self.prev_rb_x)
        dy = float(p.y) - float(self.prev_rb_y)
        dyaw = angle_diff(float(yaw), float(self.prev_rb_yaw))
        pose_changed = (abs(dx) > self._EPS_CHANGE) or (abs(dy) > self._EPS_CHANGE) or (abs(dyaw) > self._EPS_CHANGE)

        v = float(self.last_speed) if self.last_speed is not None else 0.0

        # use nominal dt for derived quantities (no wall-time)
        dt_nom = float(self.nominal_dt)

        # CHANGE 1: filtered acceleration from wheel speed
        raw_accel = (v - float(self.prev_speed)) / dt_nom if dt_nom > 1e-6 else 0.0
        self.filtered_accel = self.alpha * raw_accel + (1.0 - self.alpha) * self.filtered_accel
        self.prev_speed = v
        a_filt = float(self.filtered_accel)

        # CHANGE 2: Velocity consistency blending
        # Blend wheel-speed velocity with mocap displacement velocity (from dx, dy).
        # This stabilizes v when either sensor is noisy or temporarily inconsistent.
        v_meas = math.sqrt(dx * dx + dy * dy) / dt_nom if dt_nom > 1e-6 else 0.0
        beta = 0.7  # 0..1 (higher = trust mocap displacement more)
        v = beta * v_meas + (1.0 - beta) * v

        # CASE 1: pose changed -> normal EKF predict+update and publish from mocap
        if pose_changed:
            # use nominal dt for EKF predict (no wall-time)
            dt = float(self.nominal_dt)
            try:
                self.ekf.predict(dt)
                self.ekf.update(np.array([float(p.x), float(p.y), float(yaw)]))
            except Exception as e:
                self.get_logger().warn(f"EKF predict/update error: {e}")

            # finite-difference velocities from mocap and publish (using dt)
            vxm = dx / dt
            vym = dy / dt
            axm = (vxm - float(self.ekf.x[3, 0])) / dt if dt > 1e-6 else 0.0
            aym = (vym - float(self.ekf.x[4, 0])) / dt if dt > 1e-6 else 0.0

            # update last published pose to mocap (we trust mocap when it changes)
            self.last_pose_x = float(p.x)
            self.last_pose_y = float(p.y)
            self.last_pose_yaw = float(yaw)
            self.prev_rb_x = float(p.x)
            self.prev_rb_y = float(p.y)
            self.prev_rb_yaw = float(yaw)
            self.prev_z = float(p.z)

            # compute twist in child frame if requested
            psi = float(self.ekf.x[2, 0])
            c, s = math.cos(psi), math.sin(psi)
            if self.twist_child:
                vx = c * vxm + s * vym
                vy = -s * vxm + c * vym
                ax = c * axm + s * aym
                ay = -s * axm + c * aym
            else:
                vx, vy, ax, ay = vxm, vym, axm, aym

            vx, vy, ax, ay = map(self._clean, (vx, vy, ax, ay))
            if self.twist_child and self.no_neg_vx and vx < 0:
                vx = 0.0

            self._publish_pose(msg, rb, vx, vy, ax, ay)
            return

        # CASE 2: pose did NOT change between consecutive messages -> frozen
        # If frozen but speed > threshold -> dead-reckon using speed only
        if abs(v) > self._SPEED_THRESHOLD:
            # integrate last pose using speed along last yaw
            dt = float(self.nominal_dt)
            self.get_logger().info(f"last speed{v}")
            # v = 0.4
            yaw_use = self._get_last_yaw()
            # integrate forward using forward speed v
            dx_int = v * dt * math.cos(yaw_use)
            dy_int = v * dt * math.sin(yaw_use)

            # update last_pose
            self.last_pose_x = float(self.last_pose_x + dx_int)
            self.last_pose_y = float(self.last_pose_y + dy_int)
            # keep yaw unchanged (mocap frozen) -- do not modify EKF yaw
            self.last_pose_yaw = float(yaw_use)

            # Also update EKF velocity/acc and predict forward for internal filter state,
            # but do NOT use EKF position as authoritative for publishing in this frozen+moving case.
            try:
                # set EKF velocity/acc/yaw_rate and predict
                v_prev = float(self.ekf.x[3, 0])
                a = (v - v_prev) / dt if dt > 1e-6 else 0.0
                # CHANGE 1: use filtered acceleration for robustness
                a = a_filt
                self.ekf.x[3, 0] = v
                self.ekf.x[4, 0] = a
                self.ekf.x[5, 0] = 0.0
                self.ekf.predict(dt)
            except Exception as e:
                self.get_logger().warn(f"EKF predict error: {e}")

            # publish odom using integrated last_pose (NOT EKF pose)
            od = Odometry()
            od.header.frame_id = self.frame_id
            od.child_frame_id = self.child_id
            od.header.stamp = msg.header.stamp

            od.pose.pose.position.x = float(self.last_pose_x)
            od.pose.pose.position.y = float(self.last_pose_y)
            od.pose.pose.position.z = float(self.prev_z)
            qx, qy, qz, qw = quat_from_yaw(self.last_pose_yaw)
            od.pose.pose.orientation.x = qx
            od.pose.pose.orientation.y = qy
            od.pose.pose.orientation.z = qz
            od.pose.pose.orientation.w = qw

            # publish twist
            od.twist.twist.linear.x = float(v)
            od.twist.twist.linear.y = 0.0
            od.twist.twist.angular.z = 0.0
            self.pub_odom.publish(od)

            # publish approx accel message
            ac = AccelWithCovarianceStamped()
            ac.header = msg.header
            # CHANGE 1: publish filtered acceleration
            ac.accel.accel.linear.x = float(a_filt)
            ac.accel.accel.linear.y = 0.0
            self.pub_accel.publish(ac)
            return

        # CASE 3: frozen and stopped -> keep publishing last_pose with zero twist/accel
        od = Odometry()
        od.header.frame_id = self.frame_id
        od.child_frame_id = self.child_id
        od.header.stamp = msg.header.stamp

        od.pose.pose.position.x = float(self.last_pose_x)
        od.pose.pose.position.y = float(self.last_pose_y)
        od.pose.pose.position.z = float(self.prev_z)
        qx, qy, qz, qw = quat_from_yaw(self._get_last_yaw())
        od.pose.pose.orientation.x = qx
        od.pose.pose.orientation.y = qy
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw

        od.twist.twist.linear.x = 0.0
        od.twist.twist.linear.y = 0.0
        od.twist.twist.angular.z = 0.0
        self.pub_odom.publish(od)

        ac = AccelWithCovarianceStamped()
        ac.header = msg.header
        ac.accel.accel.linear.x = 0.0
        ac.accel.accel.linear.y = 0.0
        self.pub_accel.publish(ac)

    def _publish_pose(self, msg, rb, vx, vy, ax, ay):
        """Publish odom and accel using EKF."""
        X, Y, psi, _, _, w = self.ekf.x.flatten()
        od = Odometry()
        od.header.frame_id = self.frame_id
        od.child_frame_id = self.child_id
        od.header.stamp = msg.header.stamp

        if self.pass_pos:
            od.pose.pose = rb.pose
        else:
            od.pose.pose.position.x = float(X)
            od.pose.pose.position.y = float(Y)
            od.pose.pose.position.z = rb.pose.position.z
            qx, qy, qz, qw = quat_from_yaw(psi)
            od.pose.pose.orientation.x = qx
            od.pose.pose.orientation.y = qy
            od.pose.pose.orientation.z = qz
            od.pose.pose.orientation.w = qw

        od.twist.twist.linear.x = float(vx)
        od.twist.twist.linear.y = float(vy)
        od.twist.twist.angular.z = float(w)

        self.pub_odom.publish(od)

        ac = AccelWithCovarianceStamped()
        ac.header = msg.header
        ac.accel.accel.linear.x = float(ax)
        ac.accel.accel.linear.y = float(ay)
        self.pub_accel.publish(ac)


def main():
    rclpy.init()
    node = Localization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
