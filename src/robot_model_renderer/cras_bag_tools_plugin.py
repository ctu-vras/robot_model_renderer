# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

import copy
import os
import re
import sys

import cv2
import rospkg
import rospy
from cras import image_encodings
from cras_bag_tools import DeserializedMessageFilter
from cras_bag_tools.message_filter import fix_connection_header
from sensor_msgs.msg import Image
from robot_model_renderer import (RobotModelRenderer, RobotModelRendererConfig, RenderingMode, ShapeFilterConfig,
                                  ShapeInflationRegistry, ScaleAndPadding, PerShapeInflation)


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
        # Default config for this plugin
        self._config.renderingMode = RenderingMode.MASK
        self._config.drawOutline = True
        self._config.outlineColor = [1.0, 1.0, 1.0, 1.0]
        self._config.outlineWidth = 10.0
        self._config.renderImageScale = 0.25
        self._config.maxRenderImageSize = 2048
        # Custom config from kwargs
        for k, v in kwargs.items():
            if k == "renderingMode":
                if isinstance(kwargs[k], int):
                    self._config.renderingMode = RenderingMode(kwargs[k])
                else:  # string
                    self._config.renderingMode = RenderingMode[kwargs[k].upper()]
            elif k == "shapeFilter":
                self._config.shapeFilter = ShapeFilterConfig(
                    v.get("visualAllowed", True),
                    v.get("collisionAllowed", False),
                    v.get("ignoreShapes", []),
                    v.get("onlyShapes", []))
            elif k == "shapeInflationRegistry":
                defaultInflation = ScaleAndPadding(
                    kwargs.get("defaultInflation", {}).get("scale", 1.0),
                    kwargs.get("defaultInflation", {}).get("padding", 0.0))
                defaultVisualInflation = ScaleAndPadding(
                    kwargs.get("defaultVisualInflation", {}).get("scale", defaultInflation.scale),
                    kwargs.get("defaultVisualInflation", {}).get("padding", defaultInflation.padding))
                defaultCollisionInflation = ScaleAndPadding(
                    kwargs.get("defaultCollisionInflation", {}).get("scale", defaultInflation.scale),
                    kwargs.get("defaultCollisionInflation", {}).get("padding", defaultInflation.padding))
                self._config.shapeInflationRegistry = ShapeInflationRegistry(
                    defaultInflation, defaultVisualInflation, defaultCollisionInflation)
                for kk, vv in kwargs.get("perShape", {}).items():
                    i = PerShapeInflation(kk, ScaleAndPadding(vv.get("scale", 1.0), vv.get("padding", 0.0)))
                    self._config.shapeInflationRegistry.perShapeInflation.append(i)
            elif k == "staticMaskImage":
                try:
                    image_file = kwargs[k]
                    matches = re.match(r'\$\(find ([^)]+)\)', image_file)
                    if matches is not None:
                        rospack = rospkg.RosPack()
                        package_path = rospack.get_path(matches[1])
                        image_file = image_file.replace('$(find %s)' % (matches[1],), package_path)
                    self._config.staticMaskImage = cv2.imread(image_file, cv2.IMREAD_UNCHANGED)
                except Exception as e:
                    print("Error reading static mask image %s: %s" % (kwargs[k], str(e)), file=sys.stderr)
            else:
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

        self.renderer = RobotModelRenderer(model, self._config, self._encoding, warnExtrapolation=False)

    def filter(self, topic, msg, stamp, header):
        if self.renderer is None:
            return None

        to_try = []
        for m, s, h, e in self._failed_msgs:
            if stamp - s < self.tf_timeout:
                to_try.append((m, s, h))
            else:
                print("Failed transforming robot for mask rendering at time %i.%09i: %r" % (
                    m.header.stamp.secs, m.header.stamp.nsecs, e), file=sys.stderr)
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
                print("Failed rendering robot mask for time %i.%09i: %r" % (
                    msg.header.stamp.secs, msg.header.stamp.nsecs, err), file=sys.stderr)

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
