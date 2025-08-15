#!/usr/bin/env python

# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Unit test for robot_model_renderer."""

import cv2  # has to be here because of cv_bridge bug on arm64
import os
import unittest

import rospy
import rostest
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo

from robot_model_renderer import RenderingMode, RobotModelRenderer, RobotModelRendererConfig


cv_bridge = CvBridge()


def align(x, a):
    return (x + a - 1) & ~(a - 1)


def aligned_step(width, channels, planes=1):
    step = width * channels
    return [
        planes * step,
        planes * align(step, 8),
        planes * align(step, 16),
        planes * align(step, 32),
        planes * align(step, 64),
    ]


with open(os.path.join(os.path.dirname(__file__), "robot.urdf"), 'r') as f:
    model = f.read()


cam_info = CameraInfo()
cam_info.header.frame_id = "link1"
cam_info.header.stamp = rospy.Time(1732502880, 585000000)
cam_info.height = 1616
cam_info.width = 1212
cam_info.distortion_model = "plumb_bob"
cam_info.D = [1.0, 0, 0, 0, 0, 0.5, 0, 0]
cam_info.K = [800.0, 0.0, 600.0, 0.0, 800.0, 800.0, 0.0, 0.0, 1.0]
cam_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
cam_info.P = [800.0, 0.0, 600.0, 0.0, 0.0, 800.0, 800.0, 0.0, 0.0, 0.0, 1.0, 0.0]

tf = TransformStamped()
tf.header = cam_info.header
tf.child_frame_id = "link2"
tf.transform.translation.x = 0.3
tf.transform.translation.z = 1.0
tf.transform.rotation.w = 1.0


class RobotModelRendererTest(unittest.TestCase):

    def test_default(self):
        config = RobotModelRendererConfig()
        r = RobotModelRenderer('<robot name="test"><link name="link1"/></robot>', config, "rgba8")

        success, errors = r.setModel(model)
        self.assertTrue(success)

        success = r.updateCameraInfo(cam_info)
        self.assertTrue(success)

        success = r.setTransform(tf, "me", True)
        self.assertTrue(success)

        image, errors, link_errors = r.render(cam_info.header.stamp)
        self.assertIsNotNone(image)

        self.assertEqual(image.header.frame_id, "link1")
        self.assertEqual(image.header.stamp, rospy.Time(1732502880, 585000000))
        self.assertEqual(image.width, 1212)
        self.assertEqual(image.height, 1616)
        self.assertEqual(image.encoding, "rgba8")
        self.assertIn(image.step, aligned_step(1212, 4))
        self.assertEqual(image.is_bigendian, 0)

        cv_image = cv_bridge.imgmsg_to_cv2(image)
        self.assertEqual(cv_image.shape, (1616, 1212, 4))
        self.assertSequenceEqual(cv_image[0, 0].tolist(), (0, 0, 0, 0))
        self.assertSequenceEqual(cv_image[1615, 0].tolist(), (0, 0, 0, 0))
        self.assertSequenceEqual(cv_image[1615, 1211].tolist(), (0, 0, 0, 0))
        self.assertSequenceEqual(cv_image[0, 1211].tolist(), (0, 0, 0, 0))
        self.assertSequenceEqual(cv_image[1616 // 2, 1212 // 2].tolist(), (250, 0, 0, 255))
        self.assertSequenceEqual(cv_image[1616 // 2, 1211].tolist(),  (250, 0, 0, 255))
        self.assertSequenceEqual(cv_image[1616 // 2, 390].tolist(),  (0, 0, 0, 0))

    def test_cpu_distortion(self):
        config = RobotModelRendererConfig()
        config.gpuDistortion = False
        r = RobotModelRenderer('<robot name="test"><link name="link1"/></robot>', config, "rgba8")

        success, errors = r.setModel(model)
        self.assertTrue(success)

        success = r.updateCameraInfo(cam_info)
        self.assertTrue(success)

        success = r.setTransform(tf, "me", True)
        self.assertTrue(success)

        image, errors, link_errors = r.render(cam_info.header.stamp)
        self.assertIsNotNone(image)

        self.assertEqual(image.header.frame_id, "link1")
        self.assertEqual(image.header.stamp, rospy.Time(1732502880, 585000000))
        self.assertEqual(image.width, 1212)
        self.assertEqual(image.height, 1616)
        self.assertEqual(image.encoding, "rgba8")
        self.assertIn(image.step, aligned_step(1212, 4))
        self.assertEqual(image.is_bigendian, 0)

        cv_image = cv_bridge.imgmsg_to_cv2(image)
        self.assertEqual(cv_image.shape, (1616, 1212, 4))
        self.assertSequenceEqual(cv_image[0, 0].tolist(), (0, 0, 0, 0))
        self.assertSequenceEqual(cv_image[1615, 0].tolist(), (0, 0, 0, 0))
        self.assertSequenceEqual(cv_image[1615, 1211].tolist(), (0, 0, 0, 0))
        self.assertSequenceEqual(cv_image[0, 1211].tolist(), (0, 0, 0, 0))
        self.assertSequenceEqual(cv_image[1616 // 2, 1212 // 2].tolist(), (250, 0, 0, 255))
        self.assertSequenceEqual(cv_image[1616 // 2, 1211].tolist(),  (250, 0, 0, 255))
        self.assertSequenceEqual(cv_image[1616 // 2, 390].tolist(),  (0, 0, 0, 0))

    def test_default_bgra(self):
        config = RobotModelRendererConfig()
        r = RobotModelRenderer(model, config, "bgra8")

        success = r.updateCameraInfo(cam_info)
        self.assertTrue(success)

        success = r.setTransform(tf, "me", True)
        self.assertTrue(success)

        image, errors, link_errors = r.render(cam_info.header.stamp)
        self.assertIsNotNone(image)

        self.assertEqual(image.header.frame_id, "link1")
        self.assertEqual(image.header.stamp, rospy.Time(1732502880, 585000000))
        self.assertEqual(image.width, 1212)
        self.assertEqual(image.height, 1616)
        self.assertEqual(image.encoding, "bgra8")
        self.assertIn(image.step, aligned_step(1212, 4))
        self.assertEqual(image.is_bigendian, 0)

        cv_image = cv_bridge.imgmsg_to_cv2(image)
        self.assertEqual(cv_image.shape, (1616, 1212, 4))
        self.assertSequenceEqual(cv_image[0, 0].tolist(), (0, 0, 0, 0))
        self.assertSequenceEqual(cv_image[1615, 0].tolist(), (0, 0, 0, 0))
        self.assertSequenceEqual(cv_image[1615, 1211].tolist(), (0, 0, 0, 0))
        self.assertSequenceEqual(cv_image[0, 1211].tolist(), (0, 0, 0, 0))
        self.assertSequenceEqual(cv_image[1616 // 2, 1212 // 2].tolist(), (0, 0, 250, 255))
        self.assertSequenceEqual(cv_image[1616 // 2, 1211].tolist(), (0, 0, 250, 255))
        self.assertSequenceEqual(cv_image[1616 // 2, 390].tolist(), (0, 0, 0, 0))

    def test_default_mono(self):
        config = RobotModelRendererConfig()
        r = RobotModelRenderer(model, config, "mono8")

        success = r.updateCameraInfo(cam_info)
        self.assertTrue(success)

        success = r.setTransform(tf, "me", True)
        self.assertTrue(success)

        image, errors, link_errors = r.render(cam_info.header.stamp)
        self.assertIsNotNone(image)

        self.assertEqual(image.header.frame_id, "link1")
        self.assertEqual(image.header.stamp, rospy.Time(1732502880, 585000000))
        self.assertEqual(image.width, 1212)
        self.assertEqual(image.height, 1616)
        self.assertEqual(image.encoding, "mono8")
        self.assertIn(image.step, aligned_step(1212, 1))
        self.assertEqual(image.is_bigendian, 0)

        cv_image = cv_bridge.imgmsg_to_cv2(image)
        self.assertEqual(cv_image.shape, (1616, 1212))
        self.assertEqual(cv_image[0, 0].tolist(), 0)
        self.assertEqual(cv_image[1615, 0].tolist(), 0)
        self.assertEqual(cv_image[1615, 1211].tolist(), 0)
        self.assertEqual(cv_image[0, 1211].tolist(), 0)
        self.assertEqual(cv_image[1616 // 2, 1212 // 2].tolist(), 250)
        self.assertEqual(cv_image[1616 // 2, 1211].tolist(), 250)
        self.assertEqual(cv_image[1616 // 2, 390].tolist(), 0)

    def test_color_mode_with_outline(self):
        config = RobotModelRendererConfig()
        config.renderingMode = RenderingMode.COLOR
        config.colorModeColor = [0.0, 1.0, 0.0, 1.0]
        config.drawOutline = True
        config.outlineWidth = 20.0
        config.outlineColor = [0.0, 0.0, 1.0, 1.0]

        r = RobotModelRenderer(model, config, "rgba8")

        success = r.updateCameraInfo(cam_info)
        self.assertTrue(success)

        success = r.setTransform(tf, "me", True)
        self.assertTrue(success)

        image, errors, link_errors = r.render(cam_info.header.stamp)
        self.assertIsNotNone(image)

        self.assertEqual(image.header.frame_id, "link1")
        self.assertEqual(image.header.stamp, rospy.Time(1732502880, 585000000))
        self.assertEqual(image.width, 1212)
        self.assertEqual(image.height, 1616)
        self.assertEqual(image.encoding, "rgba8")
        self.assertIn(image.step, aligned_step(1212, 4))
        self.assertEqual(image.is_bigendian, 0)

        cv_image = cv_bridge.imgmsg_to_cv2(image)
        self.assertEqual(cv_image.shape, (1616, 1212, 4))
        self.assertSequenceEqual(cv_image[0, 0].tolist(), (0, 0, 0, 0))
        self.assertSequenceEqual(cv_image[1615, 0].tolist(), (0, 0, 0, 0))
        self.assertSequenceEqual(cv_image[1615, 1211].tolist(), (0, 0, 0, 0))
        self.assertSequenceEqual(cv_image[0, 1211].tolist(), (0, 0, 0, 0))
        self.assertSequenceEqual(cv_image[1616 // 2, 1212 // 2].tolist(), (0, 250, 0, 255))
        self.assertSequenceEqual(cv_image[1616 // 2, 1211].tolist(),  (0, 250, 0, 255))
        self.assertSequenceEqual(cv_image[1616 // 2, 390].tolist(),  (0, 0, 255, 255))

    def test_default_mask_mode_with_outline_mono(self):
        config = RobotModelRendererConfig()
        config.renderingMode = RenderingMode.MASK
        config.drawOutline = True
        config.outlineWidth = 20.0
        config.outlineColor = [1.0, 1.0, 1.0, 1.0]

        r = RobotModelRenderer(model, config, "mono8")

        success = r.updateCameraInfo(cam_info)
        self.assertTrue(success)

        success = r.setTransform(tf, "me", True)
        self.assertTrue(success)

        image, errors, link_errors = r.render(cam_info.header.stamp)
        self.assertIsNotNone(image)

        self.assertEqual(image.header.frame_id, "link1")
        self.assertEqual(image.header.stamp, rospy.Time(1732502880, 585000000))
        self.assertEqual(image.width, 1212)
        self.assertEqual(image.height, 1616)
        self.assertEqual(image.encoding, "mono8")
        self.assertIn(image.step, aligned_step(1212, 1))
        self.assertEqual(image.is_bigendian, 0)

        cv_image = cv_bridge.imgmsg_to_cv2(image)
        self.assertEqual(cv_image.shape, (1616, 1212))
        self.assertEqual(cv_image[0, 0].tolist(), 0)
        self.assertEqual(cv_image[1615, 0].tolist(), 0)
        self.assertEqual(cv_image[1615, 1211].tolist(), 0)
        self.assertEqual(cv_image[0, 1211].tolist(), 0)
        self.assertEqual(cv_image[1616 // 2, 1212 // 2].tolist(), 255)
        self.assertEqual(cv_image[1616 // 2, 1211].tolist(), 255)
        self.assertEqual(cv_image[1616 // 2, 390].tolist(), 255)

    def test_lowres(self):
        config = RobotModelRendererConfig()
        config.maxRenderImageSize = 256
        r = RobotModelRenderer('<robot name="test"><link name="link1"/></robot>', config, "rgba8")

        success, errors = r.setModel(model)
        self.assertTrue(success)

        success = r.updateCameraInfo(cam_info)
        self.assertTrue(success)

        success = r.setTransform(tf, "me", True)
        self.assertTrue(success)

        image, errors, link_errors = r.render(cam_info.header.stamp)
        self.assertIsNotNone(image)

        self.assertEqual(image.header.frame_id, "link1")
        self.assertEqual(image.header.stamp, rospy.Time(1732502880, 585000000))
        self.assertEqual(image.width, 1212)
        self.assertEqual(image.height, 1616)
        self.assertEqual(image.encoding, "rgba8")
        self.assertIn(image.step, aligned_step(1212, 4))
        self.assertEqual(image.is_bigendian, 0)

        cv_image = cv_bridge.imgmsg_to_cv2(image)
        import cv2
        cv2.imwrite("/tmp/a.png", cv_image)
        self.assertEqual(cv_image.shape, (1616, 1212, 4))
        self.assertSequenceEqual(cv_image[0, 0].tolist(), (0, 0, 0, 0))
        self.assertSequenceEqual(cv_image[1615, 0].tolist(), (0, 0, 0, 0))
        self.assertSequenceEqual(cv_image[1615, 1211].tolist(), (0, 0, 0, 0))
        self.assertSequenceEqual(cv_image[0, 1211].tolist(), (0, 0, 0, 0))
        self.assertSequenceEqual(cv_image[1616 // 2, 1212 // 2].tolist(), (250, 0, 0, 255))
        self.assertSequenceEqual(cv_image[1616 // 2, 1211].tolist(),  (250, 0, 0, 255))
        self.assertSequenceEqual(cv_image[1616 // 2, 390].tolist(),  (0, 0, 0, 0))


if __name__ == '__main__':
    rostest.rosrun('robot_model_renderer', 'test_robot_model_renderer_py', RobotModelRendererTest)
