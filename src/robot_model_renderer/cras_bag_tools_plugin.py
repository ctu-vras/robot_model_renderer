# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

import copy
import os
import sys

import rospy
from cras import image_encodings
from cras_bag_tools import DeserializedMessageFilter
from cras_bag_tools.message_filter import fix_connection_header
from sensor_msgs.msg import Image
from robot_model_renderer import RobotModelRenderer, RobotModelRendererConfig, RenderingMode


class RenderCameraMask(DeserializedMessageFilter):
    def __init__(self, camera_info_topic, mask_topic=None, description_param=None, description_file=None,
                 encoding=image_encodings.MONO8, tf_timeout=1.5,
                 min_stamp=None, max_stamp=None, include_time_ranges=None, exclude_time_ranges=None, **kwargs):
        super(RenderCameraMask, self).__init__(
            include_topics=[camera_info_topic, "/tf", "/tf_static"], min_stamp=min_stamp, max_stamp=max_stamp,
            include_time_ranges=include_time_ranges, exclude_time_ranges=exclude_time_ranges)

        self.camera_info_topic = camera_info_topic
        self.mask_topic = mask_topic
        if mask_topic is None:
            self.mask_topic = camera_info_topic.replace("/camera_info", "/image_raw/mask")
        self.tf_timeout = rospy.Duration(tf_timeout)
        self.description_param = description_param

        self._model_from_file = None
        if description_file is not None:
            if os.path.exists(description_file):
                try:
                    with open(description_file, "r") as f:
                        self._model_from_file = f.read()
                    print("Loaded URDF model from file " + description_file)
                except Exception as e:
                    print('Could not read URDF model from file %s: %s' % (description_file, str(e)), file=sys.stderr)
            else:
                print('URDF model file "%s" does not exist.' % (description_file,), file=sys.stderr)

        self._config = RobotModelRendererConfig()
        self._config.renderingMode = RenderingMode.MASK
        self._config.drawOutline = True
        self._config.outlineColor = [1.0, 1.0, 1.0, 1.0]
        self._config.outlineWidth = 10.0
        self._config.renderImageScale = 0.25
        self._config.maxRenderImageSize = 2048
        for k, v in kwargs.items():
            setattr(self._config, k, v)
        self._encoding = encoding

        self.renderer = None
        self._failed_msgs = []

    def set_bag(self, bag):
        super(RenderCameraMask, self).set_bag(bag)

        model = self._model_from_file
        if model is None:
            if self.description_param is None:
                self.description_param = "robot_description"
            model = self._get_param(self.description_param)
            if model is not None:
                print("Read URDF model from ROS parameter " + self.description_param)

        if model is None:
            raise RuntimeError("RenderCameraMask requires robot URDF either as a file or ROS parameter.")

        self.renderer = RobotModelRenderer(model, self._config, self._encoding)

    def filter(self, topic, msg, stamp, header):
        if self.renderer is None:
            return None

        to_try = []
        for m, s, h, e in self._failed_msgs:
            if stamp - s < self.tf_timeout:
                to_try.append((m, s, h))
            else:
                print("Failed transforming robot for mask rendering at time %s: %r" % (m.header.stamp, e),
                      file=sys.stderr)
        self._failed_msgs = []

        if topic.lstrip('/') == 'tf':
            for transform in msg.transforms:
                self.renderer.setTransform(transform, "robot_model_renderer.cras_bag_tools_plugin", False)
        elif topic.lstrip('/') == 'tf_static':
            for transform in msg.transforms:
                self.renderer.setTransform(transform, "robot_model_renderer.cras_bag_tools_plugin", True)
        else:
            to_try.append((msg, stamp, header))
        result = [(topic, msg, stamp, header)]

        for msg, stamp, header in to_try:
            img, err, link_err = self.renderer.render(msg)

            may_succeed_later = any([e.maySucceedLater for e in link_err]) if link_err is not None else False
            if may_succeed_later:
                self._failed_msgs.append((msg, stamp, header, err))
            elif img is not None:
                header = fix_connection_header(copy.copy(header), self.mask_topic, Image._type, Image._md5sum, Image)
                result.append((self.mask_topic, img, stamp, header))
            else:
                print("Failed rendering robot mask for time %s: %r" % (msg.header.stamp, err), file=sys.stderr)

        return result

    def reset(self):
        self.renderer = None
        self._failed_msgs = []
        super(RenderCameraMask, self).reset()

    def _str_params(self):
        parts = [
            'camera_info_topic=' + self.camera_info_topic,
            'mask_topic=' + self.mask_topic
        ]
        if self.description_param != "robot_description":
            parts.append('description_param=' + self.description_param)
        parent_params = super(RenderCameraMask, self)._str_params()
        if len(parent_params) > 0:
            parts.append(parent_params)
        return ", ".join(parts)
