#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author: Yuki Furuta (ROS2 port)

import unittest


class TestInstalled(unittest.TestCase):
    def test_ros2_and_std_msgs(self):
        """Check that ROS2 runtime dependencies are importable."""
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import Bool, Int32, ColorRGBA, String, UInt8MultiArray
        self.assertTrue(True, 'ROS2 runtime dependencies are installed')

    def test_respeaker_params_defined(self):
        """Check that respeaker_node exposes PARAMETERS for gencfg (skip if hardware deps missing)."""
        try:
            from respeaker_ros import respeaker_node as m
        except (ImportError, RuntimeError, OSError):
            self.skipTest('respeaker_node requires hardware-specific dependencies; skipping')
        self.assertIn('DOAANGLE', m.PARAMETERS)
        self.assertIn('STATNOISEONOFF', m.RESPEAKER_RW_PARAMS)


if __name__ == '__main__':
    unittest.main()
