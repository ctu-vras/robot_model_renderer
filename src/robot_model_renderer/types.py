# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

import ctypes
from ctypes import c_bool, c_char_p, c_double, c_float, c_int, c_size_t, c_uint, c_uint8, c_uint32, c_void_p
from ctypes import POINTER, RTLD_GLOBAL, Structure
from enum import IntEnum
import sys

from cras.ctypes_utils import load_library


__lib = None


robot_model_renderer_RobotModelRendererHandle = c_void_p


def _get_library():
    global __lib
    if __lib is None:
        __lib = load_library('robot_model_renderer_c_api', mode=RTLD_GLOBAL)
        if __lib is None:
            return None

        # Add function signatures

        __lib.robot_model_renderer_createDefaultRobotModelRendererConfig.restype = _RobotModelRendererConfig
        __lib.robot_model_renderer_createDefaultRobotModelRendererConfig.argtypes = []

    return __lib


class CtypesEnum(IntEnum):
    @classmethod
    def from_param(cls, obj):
        return int(obj)


class RenderingMode(CtypesEnum):
    NORMAL = 0
    COLOR = 1
    MASK = 2


class _ShapeFilterConfig(Structure):
    _fields_ = [
        ("visualAllowed", c_bool),
        ("collisionAllowed", c_bool),
        ("ignoreShapes", c_char_p),
        ("onlyShapes", c_char_p),
    ]

    def __init__(self, *args, **kwargs):
        self.visualAllowed = True
        self.collisionAllowed = False
        self.ignoreShapes = b""
        self.onlyShapes = b""
        super(_ShapeFilterConfig, self).__init__(*args, **kwargs)


class ShapeFilterConfig(object):
    def __init__(self, visualAllowed=True, collisionAllowed=False, ignoreShapes=None, onlyShapes=None):
        super(ShapeFilterConfig, self).__init__()
        if isinstance(visualAllowed, _ShapeFilterConfig):
            conf = visualAllowed
            self.visualAllowed = conf.visualAllowed
            self.collisionAllowed = conf.collisionAllowed
            self.ignoreShapes = conf.ignoreShapes.split(",")
            self.onlyShapes = conf.onlyShapes.split(",")
        else:
            self.visualAllowed = visualAllowed
            self.collisionAllowed = collisionAllowed
            self.ignoreShapes = ignoreShapes if ignoreShapes else []
            self.onlyShapes = onlyShapes if onlyShapes else []

    @classmethod
    def from_param(cls, obj):
        c = _ShapeFilterConfig()
        c.visualAllowed = obj.visualAllowed
        c.collisionAllowed = obj.collisionAllowed
        c.ignoreShapes = ",".join(obj.ignoreShapes).encode("utf-8")
        c.onlyShapes = ",".join(obj.onlyShapes).encode("utf-8")
        return c


class ScaleAndPadding(Structure):
    _fields_ = [
        ("scale", c_double),
        ("padding", c_double),
    ]

    def __init__(self, *args, **kwargs):
        self.scale = 1.0
        self.padding = 0.0
        super(ScaleAndPadding, self).__init__(*args, **kwargs)


class PerShapeInflation(Structure):
    _fields_ = [
        ("shapeName", c_char_p),
        ("inflation", ScaleAndPadding),
    ]

    def __init__(self, *args, **kwargs):
        self.shapeName = ""
        self.inflation = ScaleAndPadding()
        super(PerShapeInflation, self).__init__(*args, **kwargs)


class _ShapeInflationRegistry(Structure):
    _fields_ = [
        ("defaultInflation", ScaleAndPadding),
        ("defaultVisualInflation", ScaleAndPadding),
        ("defaultCollisionInflation", ScaleAndPadding),
        ("perShapeInflationCount", c_size_t),
        ("perShapeInflation", POINTER(PerShapeInflation)),
    ]

    def __init__(self, *args, **kwargs):
        self.defaultInflation = ScaleAndPadding()
        self.defaultVisualInflation = ScaleAndPadding()
        self.defaultCollisionInflation = ScaleAndPadding()
        self.perShapeInflationCount = 0
        self.perShapeInflation = ctypes.cast((PerShapeInflation * 0)(), POINTER(PerShapeInflation))
        super(_ShapeInflationRegistry, self).__init__(*args, **kwargs)


class ShapeInflationRegistry(object):
    def __init__(self, defaultInflation=ScaleAndPadding(), defaultVisualInflation=ScaleAndPadding(),
                 defaultCollisionInflation=ScaleAndPadding(), perShapeInflation=None):
        if isinstance(defaultInflation, _ShapeInflationRegistry):
            conf = defaultInflation
            self.defaultInflation = conf.defaultInflation
            self.defaultVisualInflation = conf.defaultVisualInflation
            self.defaultCollisionInflation = conf.defaultCollisionInflation
            self.perShapeInflation = list(ctypes.cast(
                conf.perShapeInflation, PerShapeInflation * conf.perShapeInflationCount))
        else:
            self.defaultInflation = defaultInflation
            self.defaultVisualInflation = defaultVisualInflation
            self.defaultCollisionInflation = defaultCollisionInflation
            self.perShapeInflation = perShapeInflation if perShapeInflation else []

    @classmethod
    def from_param(cls, obj):
        c = _ShapeInflationRegistry()
        c.defaultInflation = obj.defaultInflation
        c.defaultVisualInflation = obj.defaultVisualInflation
        c.defaultCollisionInflation = obj.defaultCollisionInflation
        c.perShapeInflationCount = len(obj.perShapeInflation)
        c.perShapeInflation = (PerShapeInflation * c.perShapeInflationCount)()
        c.perShapeInflation.value = obj.perShapeInflation
        return c


class _RobotModelRendererConfig(Structure):
    _fields_ = [
        ("setupDefaultLighting", c_bool),
        ("pixelFormat", c_int),
        ("backgroundColor", c_float * 4),
        ("doDistort", c_bool),
        ("gpuDistortion", c_bool),
        ("nearClipDistance", c_float),
        ("farClipDistance", c_float),
        ("renderingMode", c_int),
        ("colorModeColor", c_float * 4),
        ("drawOutline", c_bool),
        ("outlineWidth", c_double),
        ("outlineColor", c_float * 4),
        ("outlineFromClosestColor", c_bool),
        ("invertColors", c_bool),
        ("invertAlpha", c_bool),
        ("allLinksRequired", c_bool),
        ("requiredLinks", c_char_p),
        ("shapeFilter", _ShapeFilterConfig),
        ("shapeInflationRegistry", _ShapeInflationRegistry),
    ]

    def __init__(self, *args, **kwargs):
        conf = _get_library().robot_model_renderer_createDefaultRobotModelRendererConfig()
        for field, _ in self._fields_:
            setattr(self, field, getattr(conf, field))
        super(_RobotModelRendererConfig, self).__init__(*args, **kwargs)


_simple_types = (c_bool, c_float, c_double, c_int, c_uint, c_uint8, c_uint32, c_size_t)


def _decode(s, enc='utf-8'):
    if sys.version_info[0] == 2:
        return s
    return s.decode(enc)


class RobotModelRendererConfig(object):
    def __init__(self):
        conf = _get_library().robot_model_renderer_createDefaultRobotModelRendererConfig()

        self.setupDefaultLighting = conf.setupDefaultLighting
        self.pixelFormat = conf.pixelFormat
        self.backgroundColor = list(conf.backgroundColor)
        self.doDistort = conf.doDistort
        self.gpuDistortion = conf.gpuDistortion
        self.nearClipDistance = conf.nearClipDistance
        self.farClipDistance = conf.farClipDistance
        self.renderingMode = conf.renderingMode
        self.colorModeColor = list(conf.colorModeColor)
        self.drawOutline = conf.drawOutline
        self.outlineWidth = conf.outlineWidth
        self.outlineColor = list(conf.outlineColor)
        self.outlineFromClosestColor = conf.outlineFromClosestColor
        self.invertColors = conf.invertColors
        self.invertAlpha = conf.invertAlpha
        self.allLinksRequired = conf.allLinksRequired
        self.requiredLinks = _decode(conf.requiredLinks).split(",") if len(conf.requiredLinks) > 0 else []
        self.shapeFilter = ShapeFilterConfig()
        self.shapeInflationRegistry = ShapeInflationRegistry()

    @classmethod
    def from_param(cls, obj):
        c = _RobotModelRendererConfig()
        for f, t in _RobotModelRendererConfig._fields_:
            if t in _simple_types:
                setattr(c, f, getattr(obj, f))

        c.backgroundColor = (c_float * 4)(*obj.backgroundColor)
        c.colorModeColor = (c_float * 4)(*obj.colorModeColor)
        c.outlineColor = (c_float * 4)(*obj.outlineColor)
        c.requiredLinks = (",".join(obj.requiredLinks)).encode("utf-8")
        c.shapeFilter = ShapeFilterConfig.from_param(obj.shapeFilter)
        c.shapeInflationRegistry = ShapeInflationRegistry.from_param(obj.shapeInflationRegistry)
        return c


class LinkError(object):
    def __init__(self, name, error, maySucceedLater):
        self.name = name
        self.error = error
        self.maySucceedLater = maySucceedLater

    def __str__(self):
        return "%s: %s %s" % (self.name, self.error, "(may succeed later)" if self.maySucceedLater else "")


__all__ = [
    RenderingMode.__name__,
    ShapeFilterConfig.__name__,
    ScaleAndPadding.__name__,
    PerShapeInflation.__name__,
    ShapeInflationRegistry.__name__,
    RobotModelRendererConfig.__name__,
    LinkError.__name__,
]
