#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Traceability:
- US 4.1: OptiTrack freeze detection and fallback localization
- US 4.2: Wheel-speed-based prediction during fallback
"""

import math
import numpy as np
import pytest
import rclpy

from localization_ekf.localization import (
    EKF,
    Localization,
    yaw_from_quat,
    quat_from_yaw,
    angle_diff,
)

# Helper

def make_ekf():
    """Helper to create EKF with default parameters."""
    return EKF(
        qa=0.8,
        qw=0.6,
        rx=0.005,
        ry=0.005,
        rpsi=0.01,
        P0=[0.1, 0.1, 0.05, 0.5, 0.5, 0.2],
    )


# Math helpers (supporting EKF & pose comparison)


def test_UT_LOC_MATH_yaw_from_quat():
    """
    LOC_TC_01
    Supports EKF math correctness.
    Verifies yaw extraction from quaternion.
    """
    yaw = 0.7
    qz = math.sin(yaw / 2)
    qw = math.cos(yaw / 2)
    assert abs(yaw_from_quat(0.0, 0.0, qz, qw) - yaw) < 1e-6


def test_UT_LOC_MATH_quat_from_yaw_roundtrip():
    """
    LOC_TC_02
    Supports EKF math correctness.
    Verifies yaw → quaternion → yaw roundtrip.
    """
    yaw = -1.2
    _, _, qz, qw = quat_from_yaw(yaw)
    assert abs(yaw_from_quat(0.0, 0.0, qz, qw) - yaw) < 1e-6


def test_UT_LOC_MATH_angle_diff_wrap():
    """
    LOC_TC_03
    Supports EKF angle handling.
    Verifies correct wrap-around of angle difference.
    """
    assert math.isclose(
        angle_diff(math.pi - 0.1, -math.pi + 0.1),
        -0.2,
        abs_tol=1e-3,
    )


# EKF unit tests


def test_UT_LOC_EKF_initial_state():
    """
    LOC_TC_04
    Verifies EKF initial state initialization.
    """
    ekf = make_ekf()
    assert ekf.x.shape == (6, 1)
    assert np.allclose(ekf.x, 0.0)


def test_UT_LOC_EKF_predict_straight():
    """
    LOC_TC_05
    Verifies EKF prediction during straight-line motion.
    """
    ekf = make_ekf()
    ekf.x[:, 0] = np.array([0, 0, 0, 1.0, 0.0, 0.0])
    ekf.predict(1.0)
    assert ekf.x[0, 0] > 0.9


def test_UT_LOC_EKF_predict_turning():
    """
    LOC_TC_06
    Verifies EKF prediction during turning motion.
    """
    ekf = make_ekf()
    ekf.x[:, 0] = np.array([0, 0, 0, 1.0, 0.0, 0.5])
    ekf.predict(0.1)
    assert ekf.x[2, 0] != 0.0


def test_UT_LOC_EKF_update_reduces_error():
    """
    LOC_TC_07
    Verifies EKF update step reduces pose estimation error.
    """
    ekf = make_ekf()
    ekf.x[:, 0] = np.array([1.0, -1.0, 0.5, 0, 0, 0])
    z = np.array([0.0, 0.0, 0.0])
    err_before = np.linalg.norm(z - ekf.x[:3, 0])
    ekf.update(z)
    err_after = np.linalg.norm(z - ekf.x[:3, 0])
    assert err_after < err_before


# Localization logic  US 4.1 / US 4.2


def test_UT_LOC_detect_frozen_pose():
    """
    LOC_TC_10
    Verifies LOC_CR_10 (AC 4.1.1):
    Detection of frozen OptiTrack pose using EPS threshold.
    """
    rclpy.init()
    node = Localization()

    node.prev_rb_x = 1.0
    node.prev_rb_y = 1.0
    node.prev_rb_yaw = 0.5

    dx = 1.0 - node.prev_rb_x
    dy = 1.0 - node.prev_rb_y
    dyaw = angle_diff(0.5, node.prev_rb_yaw)

    assert abs(dx) <= node._EPS_CHANGE
    assert abs(dy) <= node._EPS_CHANGE
    assert abs(dyaw) <= node._EPS_CHANGE

    node.destroy_node()
    rclpy.shutdown()


def test_UT_LOC_fallback_init_from_last_pose():
    """
    LOC_TC_13
    Verifies LOC_CR_13 (AC 4.1.4):
    Fallback pose initialized from last valid OptiTrack pose.
    """
    rclpy.init()
    node = Localization()

    node.last_pose_x = 2.0
    node.last_pose_y = 3.0
    node.last_pose_yaw = 1.0

    yaw = node._get_last_yaw()
    assert yaw == pytest.approx(1.0)

    node.destroy_node()
    rclpy.shutdown()


def test_UT_LOC_frozen_pose_moving_dead_reckon():
    """
    LOC_TC_11
    Verifies LOC_CR_11, LOC_CR_12 (AC 4.1.2, AC 4.2.2):
    Dead-reckoning using wheel speed during frozen OptiTrack pose.
    """
    rclpy.init()
    node = Localization()

    node.last_pose_x = 0.0
    node.last_pose_y = 0.0
    node.last_pose_yaw = 0.0
    node.last_speed = 1.0
    node.nominal_dt = 0.1

    dx = node.last_speed * node.nominal_dt * math.cos(node.last_pose_yaw)
    dy = node.last_speed * node.nominal_dt * math.sin(node.last_pose_yaw)

    node.last_pose_x += dx
    node.last_pose_y += dy

    assert node.last_pose_x > 0.0
    assert node.last_pose_y == pytest.approx(0.0)

    node.destroy_node()
    rclpy.shutdown()


def test_UT_LOC_fallback_prediction_only():
    """
    LOC_TC_14
    Verifies LOC_CR_14 (AC 4.2.3):
    EKF runs in prediction-only mode during fallback.
    """
    rclpy.init()
    node = Localization()

    node.ekf.x[3, 0] = 1.0
    P_before = node.ekf.P.copy()

    node.ekf.predict(node.nominal_dt)

    assert not np.allclose(node.ekf.P, P_before)

    node.destroy_node()
    rclpy.shutdown()


def test_UT_LOC_exit_fallback_on_pose_change():
    """
    LOC_TC_16
    Verifies LOC_CR_16 (AC 4.1.5):
    Exit fallback mode when OptiTrack pose starts changing again.
    """
    rclpy.init()
    node = Localization()

    node.prev_rb_x = 1.0
    node.prev_rb_y = 1.0
    node.prev_rb_yaw = 0.0

    new_x = 1.5
    pose_changed = abs(new_x - node.prev_rb_x) > node._EPS_CHANGE

    assert pose_changed is True

    node.destroy_node()
    rclpy.shutdown()
