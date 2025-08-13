# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

from __future__ import print_function

from ctypes import c_bool, c_char_p, c_int, c_size_t
from ctypes import byref, POINTER, RTLD_GLOBAL
import time
import sys

import rospy
from cras.ctypes_utils import load_library, Allocator, StringAllocator, BytesAllocator, LogMessagesAllocator
from robot_model_renderer.msg import RosTime, geometry_msgs_TransformStamped, sensor_msgs_CameraInfo
from robot_model_renderer.types import robot_model_renderer_RobotModelRendererHandle, \
    LinkError, RenderingMode, RobotModelRendererConfig
from sensor_msgs.msg import CameraInfo, Image


__lib = None


def _get_library():
    global __lib
    if __lib is None:
        __lib = load_library('robot_model_renderer_c_api', mode=RTLD_GLOBAL)
        if __lib is None:
            return None

        # Add function signatures

        __lib.robot_model_renderer_sensorMsgsEncodingToOgrePixelFormat.restype = c_int
        __lib.robot_model_renderer_sensorMsgsEncodingToOgrePixelFormat.argtypes = [c_char_p, Allocator.ALLOCATOR]

        __lib.robot_model_renderer_createRobotModelRenderer.restype = robot_model_renderer_RobotModelRendererHandle
        __lib.robot_model_renderer_createRobotModelRenderer.argtypes = [
            Allocator.ALLOCATOR, c_char_p, Allocator.ALLOCATOR, RobotModelRendererConfig,
        ]

        __lib.robot_model_renderer_deleteRobotModelRenderer.restype = None
        __lib.robot_model_renderer_deleteRobotModelRenderer.argtypes = [
            robot_model_renderer_RobotModelRendererHandle,
        ]

        __lib.robot_model_renderer_RobotModelRenderer_setModel.restype = c_bool
        __lib.robot_model_renderer_RobotModelRenderer_setModel.argtypes = [
            robot_model_renderer_RobotModelRendererHandle, c_char_p, Allocator.ALLOCATOR,
        ]

        __lib.robot_model_renderer_RobotModelRenderer_updateCameraInfo.restype = c_bool
        __lib.robot_model_renderer_RobotModelRenderer_updateCameraInfo.argtypes = [
            robot_model_renderer_RobotModelRendererHandle, sensor_msgs_CameraInfo,
        ]

        __lib.robot_model_renderer_RobotModelRenderer_render.restype = c_bool
        __lib.robot_model_renderer_RobotModelRenderer_render.argtypes = [
            robot_model_renderer_RobotModelRendererHandle, RosTime, POINTER(c_size_t),
            Allocator.ALLOCATOR, Allocator.ALLOCATOR, Allocator.ALLOCATOR,
        ]

        __lib.robot_model_renderer_LinkUpdater_set_transform.restype = c_bool
        __lib.robot_model_renderer_LinkUpdater_set_transform.argtypes = [
            robot_model_renderer_RobotModelRendererHandle, geometry_msgs_TransformStamped, c_char_p, c_bool,
        ]

    return __lib


def _decode(s, enc='utf-8'):
    if sys.version_info[0] == 2:
        return s
    return s.decode(enc)


def sensorMsgsEncodingToOgrePixelFormat(encoding):
    error_alloc = StringAllocator()
    pixelFormat = _get_library().robot_model_renderer_sensorMsgsEncodingToOgrePixelFormat(
        encoding.encode("utf-8"), error_alloc.get_cfunc())

    if error_alloc.value is not None:
        raise RuntimeError(error_alloc.value)

    return pixelFormat


class RobotModelRenderer(object):
    def __init__(self, model, config, imageEncoding=None):
        self._log_alloc = LogMessagesAllocator()
        self._log_alloc_func = self._log_alloc.get_cfunc()

        self._config = config
        self._imageEncoding = imageEncoding
        if imageEncoding is not None:
            self._config.pixelFormat = sensorMsgsEncodingToOgrePixelFormat(imageEncoding)

        error_alloc = StringAllocator()
        self._handle = _get_library().robot_model_renderer_createRobotModelRenderer(
            self._log_alloc_func, model.encode("utf-8"), error_alloc.get_cfunc(), config)

        self._camera_info = None
        self._flush_log_messages()

        if self._handle is None:
            raise RuntimeError("\n".join(error_alloc.values))

    def __del__(self):
        if self._handle is not None:
            _get_library().robot_model_renderer_deleteRobotModelRenderer(self._handle)

    def _flush_log_messages(self):
        self._log_alloc.print_log_messages()
        del self._log_alloc.allocated[:]
        del self._log_alloc.allocated_sizes[:]

    def setModel(self, model):
        error_alloc = StringAllocator()
        success = _get_library().robot_model_renderer_RobotModelRenderer_setModel(
            self._handle, model.encode("utf-8"), error_alloc.get_cfunc())
        self._flush_log_messages()
        return success, error_alloc.values

    def updateCameraInfo(self, cameraInfo):
        success = _get_library().robot_model_renderer_RobotModelRenderer_updateCameraInfo(
            self._handle, sensor_msgs_CameraInfo(cameraInfo))
        self._camera_info = cameraInfo
        self._flush_log_messages()
        return success

    def render(self, time_or_cameraInfo):
        image_data_alloc = BytesAllocator()
        link_error_alloc = BytesAllocator()
        error_alloc = StringAllocator()

        if isinstance(time_or_cameraInfo, (CameraInfo, sensor_msgs_CameraInfo)):
            if not self.updateCameraInfo(time_or_cameraInfo):
                return None, "Update of camera parameters failed.", None
            time = time_or_cameraInfo.header.stamp
        else:
            time = time_or_cameraInfo

        image_step = c_size_t()

        success = _get_library().robot_model_renderer_RobotModelRenderer_render(
            self._handle, RosTime(time), byref(image_step), image_data_alloc.get_cfunc(), link_error_alloc.get_cfunc(),
            error_alloc.get_cfunc())

        self._flush_log_messages()

        error_messages = error_alloc.values
        link_errors = []
        for error_bytes in link_error_alloc.values:
            if len(error_bytes) < 3:
                rospy.logerr("Invalid link error data")
                continue
            maySucceedLater = error_bytes[0] != 0
            splits = error_bytes[1:].split(b'\x00')
            if len(splits) != 3:
                rospy.logerr("Invalid link error data")
                continue
            link_errors.append(LinkError(_decode(splits[0]), _decode(splits[1]), maySucceedLater))

        if not success:
            return None, error_messages, link_errors

        image = Image()
        if self._camera_info is not None:
            image.header.frame_id = self._camera_info.header.frame_id
            image.height = self._camera_info.height
            image.width = self._camera_info.width
        if self._imageEncoding is not None:
            image.encoding = self._imageEncoding
        image.header.stamp = time
        image.step = image_step.value
        image.is_bigendian = sys.byteorder == 'big'
        image.data = image_data_alloc.value

        return image, error_messages, link_errors

    def setTransform(self, transform, authority, isStatic):
        success = _get_library().robot_model_renderer_LinkUpdater_set_transform(
            self._handle, geometry_msgs_TransformStamped(transform), authority.encode("utf-8"), isStatic)
        self._flush_log_messages()
        return success


__all__ = [
    sensorMsgsEncodingToOgrePixelFormat.__name__,
    RobotModelRenderer.__name__,
]


if __name__ == '__main__':
    def main():
        import cv2  # has to be here because of cv_bridge bug on arm64
        from cv_bridge import CvBridge
        from geometry_msgs.msg import TransformStamped
        import matplotlib.pyplot as plt

        cv_bridge = CvBridge()

        rospy.init_node("robot_model_renderer")
        config = RobotModelRendererConfig()
        r = RobotModelRenderer('<robot name="test"><link name="link1"/></robot>', config, "bgra8")
        model = ('<robot name="test">'
                 '  <link name="link1">'
                 '    <visual>'
                 '      <geometry><box size="0.1 0.2 0.3" /></geometry>'
                 '      <material name="blue"><color rgba="0.0 0.0 1.0 1.0"/></material>'
                 '    </visual>'
                 '    <collision>'
                 '      <geometry><box size="0.1 0.2 0.3" /></geometry>'
                 '    </collision>'
                 '  </link>'
                 '  <link name="link2">'
                 '    <visual>'
                 '      <geometry><sphere radius="0.5" /></geometry>'
                 '      <material name="red"><color rgba="1.0 0.0 0.0 1.0"/></material>'
                 '    </visual>'
                 '    <collision>'
                 '      <geometry><sphere radius="0.5" /></geometry>'
                 '    </collision>'
                 '  </link>'
                 '  <joint name="joint" type="fixed">'
                 '    <origin xyz="0.3 0 1.0" />'
                 '    <parent link="link1" />'
                 '    <child link="link2" />'
                 '  </joint>'
                 '</robot>')
        success, errors = r.setModel(model)
        if not success:
            print("Failed to set model", file=sys.stderr)
            print(errors, file=sys.stderr)
            return 1

        camInfo = CameraInfo()
        camInfo.header.frame_id = "link1"
        camInfo.header.stamp = rospy.Time(time.time())
        camInfo.height = 1616
        camInfo.width = 1212
        camInfo.distortion_model = "plumb_bob"
        camInfo.D = [1.0, 0, 0, 0, 0, 0.5, 0, 0]
        camInfo.K = [800.0, 0.0, 600.0, 0.0, 800.0, 800.0, 0.0, 0.0, 1.0]
        camInfo.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camInfo.P = [800.0, 0.0, 600.0, 0.0, 0.0, 800.0, 800.0, 0.0, 0.0, 0.0, 1.0, 0.0]

        success = r.updateCameraInfo(camInfo)
        if not success:
            print("Failed to update cameraInfo", file=sys.stderr)
            return 2

        tf = TransformStamped()
        tf.header = camInfo.header
        tf.child_frame_id = "link2"
        tf.transform.translation.x = 0.3
        tf.transform.translation.z = 1.0
        tf.transform.rotation.w = 1.0

        success = r.setTransform(tf, "me", True)
        if not success:
            print("Failed to set transform", file=sys.stderr)
            return 3

        image, errors, link_errors = r.render(camInfo.header.stamp)
        if image is None:
            print("Robot model rendering failed", file=sys.stderr)
            print(errors, file=sys.stderr)
            print(repr(link_errors), file=sys.stderr)
            return 4

        print("Image: ", image.header, image.width, image.height, image.step, len(image.data))
        for link_error in link_errors:
            print(str(link_error), file=sys.stderr)

        plt.imshow(cv_bridge.imgmsg_to_cv2(image))
        plt.waitforbuttonpress()

        config = RobotModelRendererConfig()
        config.renderingMode = RenderingMode.COLOR
        config.colorModeColor = [0.0, 1.0, 0.0, 1.0]
        config.drawOutline = True
        config.outlineWidth = 20.0
        config.outlineColor = [0.0, 0.0, 1.0, 1.0]
        r2 = RobotModelRenderer(model, config, "bgra8")
        r2.setTransform(tf, "me", True)

        image, errors, link_errors = r2.render(camInfo)
        if image is None:
            print("Robot model rendering failed", file=sys.stderr)
            print(errors, file=sys.stderr)
            print(repr(link_errors), file=sys.stderr)
            return 4

        print("Image: ", image.header, image.width, image.height, image.step, len(image.data))
        for link_error in link_errors:
            print(str(link_error), file=sys.stderr)

        plt.imshow(cv_bridge.imgmsg_to_cv2(image))
        plt.waitforbuttonpress()

        image, errors, link_errors = r.render(camInfo.header.stamp)
        if image is None:
            print("Robot model rendering failed", file=sys.stderr)
            print(errors, file=sys.stderr)
            print(repr(link_errors), file=sys.stderr)
            return 4

        print("Image: ", image.header, image.width, image.height, image.step, len(image.data))
        for link_error in link_errors:
            print(str(link_error), file=sys.stderr)

        plt.imshow(cv_bridge.imgmsg_to_cv2(image))
        plt.waitforbuttonpress()

        config.renderingMode = RenderingMode.MASK
        config.outlineColor = [1.0, 1.0, 1.0, 1.0]
        r3 = RobotModelRenderer(model, config, "mono8")
        r3.setTransform(tf, "me", True)

        image, errors, link_errors = r3.render(camInfo)
        if image is None:
            print("Robot model rendering failed", file=sys.stderr)
            print(errors, file=sys.stderr)
            print(repr(link_errors), file=sys.stderr)
            return 5

        print("Image: ", image.header, image.width, image.height, image.step, len(image.data))
        for link_error in link_errors:
            print(str(link_error), file=sys.stderr)

        plt.imshow(cv_bridge.imgmsg_to_cv2(image), cmap='binary')
        plt.waitforbuttonpress()

        return 0

    main()
