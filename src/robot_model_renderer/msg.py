# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague
import ctypes
from ctypes import c_bool, c_char_p, c_double, c_size_t, c_uint, c_uint32, c_uint8
from ctypes import cast, POINTER, Structure
import sys

import genpy
import rospy
from geometry_msgs.msg import Transform, TransformStamped
from sensor_msgs.msg import CameraInfo, RegionOfInterest
from std_msgs.msg import Header


def _decode(s, enc='utf-8'):
    if sys.version_info[0] == 2:
        return s
    return s.decode(enc)


class _RosTime(Structure):
    _fields_ = [
        ("sec", c_uint32),
        ("nsec", c_uint32),
    ]


class RosTime(rospy.Time):
    def __init__(self, secs=0, nsecs=0):
        if isinstance(secs, _RosTime):
            c = secs
            super(RosTime, self).__init__(c.sec, c.nsec)
        elif isinstance(secs, (rospy.Time, genpy.rostime.Time)):
            c = secs
            super(RosTime, self).__init__(c.secs, c.nsecs)
        else:
            super(RosTime, self).__init__(secs, nsecs)

    @classmethod
    def from_param(cls, obj):
        c = _RosTime()
        c.sec = obj.secs
        c.nsec = obj.nsecs
        return c


class _std_msgs_Header(Structure):
    _fields_ = [
        ("seq", c_uint32),
        ("stamp", _RosTime),
        ("frame_id", c_char_p),
    ]


class std_msgs_Header(Header):
    def __init__(self, header=None, *args, **kwargs):
        if header is not None and isinstance(header, _std_msgs_Header):
            super(std_msgs_Header, self).__init__(header.seq, RosTime(header.stamp), _decode(header.frame_id))
        if header is not None:
            super(std_msgs_Header, self).__init__(header.seq, RosTime(header.stamp), header.frame_id)
        else:
            super(std_msgs_Header, self).__init__(*args, **kwargs)

    @classmethod
    def from_param(cls, obj):
        c = _std_msgs_Header()
        c.seq = obj.seq
        c.stamp = RosTime.from_param(obj.stamp)
        obj._frame_id = obj.frame_id.encode("utf-8")
        c.frame_id = obj._frame_id
        return c


class _sensor_msgs_RegionOfInterest(Structure):
    _fields_ = [
        ("x_offset", c_uint32),
        ("y_offset", c_uint32),
        ("height", c_uint32),
        ("width", c_uint32),
        ("do_rectify", c_uint8),
    ]


class sensor_msgs_RegionOfInterest(RegionOfInterest):
    def __init__(self, roi=None, *args, **kwargs):
        if roi is not None:
            super(sensor_msgs_RegionOfInterest, self).__init__(
                roi.x_offset, roi.y_offset, roi.height, roi.width, roi.do_rectify)
        else:
            super(sensor_msgs_RegionOfInterest, self).__init__(*args, **kwargs)

    @classmethod
    def from_param(cls, obj):
        c = _sensor_msgs_RegionOfInterest()
        c.x_offset = obj.x_offset
        c.y_offset = obj.y_offset
        c.height = obj.height
        c.width = obj.width
        c.do_rectify = obj.do_rectify
        return c


class _sensor_msgs_CameraInfo(Structure):
    _fields_ = [
        ("header", _std_msgs_Header),
        ("height", c_uint32),
        ("width", c_uint32),
        ("distortion_model", c_char_p),
        ("D_count", c_size_t),
        ("D", POINTER(c_double)),
        ("K", c_double * 9),
        ("R", c_double * 9),
        ("P", c_double * 12),
        ("binning_x", c_uint32),
        ("binning_y", c_uint32),
        ("roi", _sensor_msgs_RegionOfInterest),
    ]


class sensor_msgs_CameraInfo(CameraInfo):
    def __init__(self, camInfo=None, *args, **kwargs):
        if camInfo is not None and isinstance(camInfo, _sensor_msgs_CameraInfo):
            super(sensor_msgs_CameraInfo, self).__init__(
                std_msgs_Header(header=camInfo.header), camInfo.height, camInfo.width,
                _decode(camInfo.distortion_model), list(cast(camInfo.D, POINTER(c_double))), camInfo.K, camInfo.R,
                camInfo.P, camInfo.binning_x, camInfo.binning_y, sensor_msgs_RegionOfInterest(roi=camInfo.roi))
        elif camInfo is not None:
            super(sensor_msgs_CameraInfo, self).__init__(
                std_msgs_Header(header=camInfo.header), camInfo.height, camInfo.width, camInfo.distortion_model,
                camInfo.D, camInfo.K, camInfo.R, camInfo.P, camInfo.binning_x, camInfo.binning_y,
                sensor_msgs_RegionOfInterest(roi=camInfo.roi))
        else:
            super(sensor_msgs_CameraInfo, self).__init__(*args, **kwargs)

    @classmethod
    def from_param(cls, obj):
        c = _sensor_msgs_CameraInfo()
        c.header = std_msgs_Header.from_param(obj.header)
        c.height = obj.height
        c.width = obj.width
        obj._distortion_model = obj.distortion_model.encode('utf-8')
        c.distortion_model = obj._distortion_model
        c.D_count = len(obj.D)
        obj._D = cast((c_double * len(obj.D))(*obj.D), POINTER(c_double))
        c.D = obj._D
        c.K = (c_double * 9)(*obj.K)
        c.R = (c_double * 9)(*obj.R)
        c.P = (c_double * 12)(*obj.P)
        c.binning_x = obj.binning_x
        c.binning_y = obj.binning_y
        c.roi = sensor_msgs_RegionOfInterest.from_param(obj.roi)
        return c


class _geometry_msgs_Transform(Structure):
    _fields_ = [
        ("translation", c_double * 3),
        ("rotation", c_double * 4),
    ]


class geometry_msgs_Transform(Transform):
    def __init__(self, tf=None, *args, **kwargs):
        if tf is not None and isinstance(tf, _geometry_msgs_Transform):
            super(geometry_msgs_Transform, self).__init__()
            self.translation.x = tf.translation[0]
            self.translation.y = tf.translation[1]
            self.translation.z = tf.translation[2]
            self.rotation.x = tf.rotation[0]
            self.rotation.y = tf.rotation[1]
            self.rotation.z = tf.rotation[2]
            self.rotation.w = tf.rotation[3]
        elif tf is not None:
            super(geometry_msgs_Transform, self).__init__()
            self.translation.x = tf.translation.x
            self.translation.y = tf.translation.y
            self.translation.z = tf.translation.z
            self.rotation.x = tf.rotation.x
            self.rotation.y = tf.rotation.y
            self.rotation.z = tf.rotation.z
            self.rotation.w = tf.rotation.w
        else:
            super(geometry_msgs_Transform, self).__init__(*args, **kwargs)

    @classmethod
    def from_param(cls, obj):
        c = _geometry_msgs_Transform()
        c.translation = (c_double * 3)(obj.translation.x, obj.translation.y, obj.translation.z)
        c.rotation = (c_double * 4)(obj.rotation.x, obj.rotation.y, obj.rotation.z, obj.rotation.w)
        return c


class _geometry_msgs_TransformStamped(Structure):
    _fields_ = [
        ("header", _std_msgs_Header),
        ("child_frame_id", c_char_p),
        ("transform", _geometry_msgs_Transform),
    ]


class geometry_msgs_TransformStamped(TransformStamped):
    def __init__(self, tf=None, *args, **kwargs):
        if tf is not None and isinstance(tf, _geometry_msgs_TransformStamped):
            super(geometry_msgs_TransformStamped, self).__init__()
            self.header = std_msgs_Header(header=tf.header)
            self.child_frame_id = _decode(tf.child_frame_id)
            self.transform = geometry_msgs_Transform(tf=tf.transform)
        elif tf is not None:
            super(geometry_msgs_TransformStamped, self).__init__()
            self.header = std_msgs_Header(header=tf.header)
            self.child_frame_id = tf.child_frame_id
            self.transform = geometry_msgs_Transform(tf=tf.transform)
        else:
            super(geometry_msgs_TransformStamped, self).__init__(*args, **kwargs)

    @classmethod
    def from_param(cls, obj):
        c = _geometry_msgs_TransformStamped()
        c.header = std_msgs_Header.from_param(obj.header)
        obj._child_frame_id = obj.child_frame_id.encode('utf-8')
        c.child_frame_id = obj._child_frame_id
        c.transform = geometry_msgs_Transform.from_param(obj.transform)
        return c


__all__ = [
    RosTime.__name__,
    std_msgs_Header.__name__,
    geometry_msgs_Transform.__name__,
    geometry_msgs_TransformStamped.__name__,
    sensor_msgs_RegionOfInterest.__name__,
    sensor_msgs_CameraInfo.__name__,
]
