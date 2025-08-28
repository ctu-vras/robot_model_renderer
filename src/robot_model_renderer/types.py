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


def _decode(s, enc='utf-8'):
    if sys.version_info[0] == 2:
        return s
    return s.decode(enc)


class CtypesEnum(IntEnum):
    @classmethod
    def from_param(cls, obj):
        return int(obj)


class RenderingMode(CtypesEnum):
    """Mode of robot model rendering."""

    NORMAL = 0
    """Normal mode, visual meshes use their textures, collision meshes use red or link color."""

    COLOR = 1
    """All meshes are rendered with the specified color (with lighting effects)."""

    MASK = 2
    """All meshes are rendered as a binary mask (robot = white, background = black)."""


class _ShapeFilterConfig(Structure):
    """
    ctypes interop with class robot_model_renderer_ShapeFilterConfig.

    :note: Do not use directly in user code. Use ShapeFilterConfig instead.
    """

    _fields_ = [
        ("visualAllowed", c_bool),
        ("collisionAllowed", c_bool),
        ("ignoreShapes", c_char_p),
        ("onlyShapes", c_char_p),
    ]

    def __init__(self, visualAllowed=True, collisionAllowed=False, ignoreShapes=b"", onlyShapes=b""):
        super(_ShapeFilterConfig, self).__init__(visualAllowed=visualAllowed, collisionAllowed=collisionAllowed,
                                                 ignoreShapes=ignoreShapes, onlyShapes=onlyShapes)


class ShapeFilterConfig(object):
    """
    Filter of shapes (visual/collision elements).

    The methods setIgnoreShapes() and setOnlyShapes() accept "shape name templates". These can be:

    - `link` (match all shapes in the link with name `link`)
    - `*::shape` (match all shapes with name `shape` in any link)
    - `link::shape#` (match the `shape#`-th visual or collision in the given link)
    - `link::shape` (match shape with name `shape` in link with name `link`)
    - `*:visual:shape` (match visual with name `shape` in any link)
    - `link:visual:shape#` (match `shape#`-th visual in link with name `link`)
    - `link:visual:shape` (match visual with name `shape` in link with name `link`)
    - `*:collision:shape` (match collision with name `shape` in any link)
    - `link:collision:shape#` (match `shape#`-th collision in link with name `link`)
    - `link:collision:shape` (match collision with name `shape` in link with name `link`)

    :ivar visualAllowed: Whether visual elements are generally allowed.
    :vartype visualAllowed: bool
    :ivar collisionAllowed: Whether collision elements are generally allowed.
    :vartype collisionAllowed: bool
    :ivar ignoreShapes: "Shape name templates" of the shapes which will be ignored.
    :vartype ignoreShapes: list
    :ivar onlyShapes: "Shape name templates" of white-listed shapes that will be allowed.
    :vartype onlyShapes: list
    """
    def __init__(self, visualAllowed=True, collisionAllowed=False, ignoreShapes=None, onlyShapes=None):
        """Construct a default filter that accepts everything.

        :param bool visualAllowed: Whether visual elements are generally allowed. You can also pass an instance of
                                   _ShapeFilterConfig to initialize this class from values contained there.
        :type visualAllowed: bool or _ShapeFilterConfig
        :param bool collisionAllowed: Whether collision elements are generally allowed.
        :param ignoreShapes: "Shape name templates" of the shapes which will be ignored.
        :type ignoreShapes: list or none
        :param onlyShapes: "Shape name templates" of white-listed shapes that will be allowed.
        :type onlyShapes: list or none
        """
        super(ShapeFilterConfig, self).__init__()
        if isinstance(visualAllowed, _ShapeFilterConfig):
            conf = visualAllowed
            self.visualAllowed = conf.visualAllowed
            self.collisionAllowed = conf.collisionAllowed
            self.ignoreShapes = _decode(conf.ignoreShapes).split(",")
            self.onlyShapes = _decode(conf.onlyShapes).split(",")
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
        obj._ignoreShapes = ",".join(obj.ignoreShapes).encode("utf-8")
        c.ignoreShapes = obj._ignoreShapes
        obj._onlyShapes = ",".join(obj.onlyShapes).encode("utf-8")
        c.onlyShapes = obj._onlyShapes
        return c


class ScaleAndPadding(Structure):
    """A structure holding the configuration of scaling and padding for a single shape.

    ctypes interop with struct robot_model_renderer_ScaleAndPadding.

    :ivar scale: Scaling (1.0 = no scaling).
    :vartype scale: float
    :ivar padding: Padding (meters).
    :vartype padding: float
    """

    _fields_ = [
        ("scale", c_double),
        ("padding", c_double),
    ]

    def __init__(self, scale=1.0, padding=0.0):
        super(ScaleAndPadding, self).__init__(scale=scale, padding=padding)


class _PerShapeInflation(Structure):
    _fields_ = [
        ("shapeName", c_char_p),
        ("inflation", ScaleAndPadding),
    ]

    def __init__(self, shapeName=b"", inflation=ScaleAndPadding()):
        super(_PerShapeInflation, self).__init__(shapeName=shapeName, inflation=inflation)


class PerShapeInflation(object):
    """Scaling and padding configuration for a single shape name template.

    :ivar shapeName: Shape name template.
    :vartype shapeName: str
    :ivar inflation: The applied scaling and padding.
    :vartype inflation: ScaleAndPadding
    """
    def __init__(self, shapeName="", inflation=ScaleAndPadding()):
        if isinstance(shapeName, _PerShapeInflation):
            self.shapeName = _decode(shapeName.shapeName)
            self.inflation = shapeName.inflation
        else:
            self.shapeName = shapeName
            self.inflation = inflation

    @classmethod
    def from_param(cls, obj):
        c = _PerShapeInflation()
        obj._shapeName = obj.shapeName.encode("utf-8")
        c.shapeName = obj._shapeName
        c.inflation = obj.inflation
        return c


class _ShapeInflationRegistry(Structure):
    """ctypes interop with struct robot_model_renderer_ShapeInflationRegistry.

    :note: Do not use directly in user code. Use ShapeInflationRegistry instead.
    """
    _fields_ = [
        ("defaultInflation", ScaleAndPadding),
        ("defaultVisualInflation", ScaleAndPadding),
        ("defaultCollisionInflation", ScaleAndPadding),
        ("perShapeInflationCount", c_size_t),
        ("perShapeInflation", POINTER(_PerShapeInflation)),
    ]

    def __init__(self, defaultInflation=ScaleAndPadding(), defaultVisualInflation=ScaleAndPadding(),
                 defaultCollisionInflation=ScaleAndPadding(), perShapeInflationCount=0,
                 perShapeInflation=ctypes.cast((_PerShapeInflation * 0)(), POINTER(_PerShapeInflation))):
        super(_ShapeInflationRegistry, self).__init__(
            defaultInflation=defaultInflation,
            defaultVisualInflation=defaultVisualInflation, defaultCollisionInflation=defaultCollisionInflation,
            perShapeInflationCount=perShapeInflationCount, perShapeInflation=perShapeInflation)


class ShapeInflationRegistry(object):
    """A registry of scaling and padding settings for shapes (visual/collision elements).

    The methods setIgnoreShapes() and setOnlyShapes() accept "shape name templates". These can be:

    - `link` (match all shapes in the link with name `link`)
    - `*::shape` (match all shapes with name `shape` in any link)
    - `link::shape#` (match the `shape#`-th visual or collision in the given link)
    - `link::shape` (match shape with name `shape` in link with name `link`)
    - `*:visual:shape` (match visual with name `shape` in any link)
    - `link:visual:shape#` (match `shape#`-th visual in link with name `link`)
    - `link:visual:shape` (match visual with name `shape` in link with name `link`)
    - `*:collision:shape` (match collision with name `shape` in any link)
    - `link:collision:shape#` (match `shape#`-th collision in link with name `link`)
    - `link:collision:shape` (match collision with name `shape` in link with name `link`)

    :ivar defaultInflation: Default scale and padding to be used for shapes without an explicit override.
    :vartype defaultInflation: ScaleAndPadding
    :ivar defaultVisualInflation: Default scale and padding to be used for visuals without an explicit override.
    :vartype defaultVisualInflation: ScaleAndPadding
    :ivar defaultCollisionInflation: Default scale and padding to be used for collisions without an explicit override.
    :vartype defaultCollisionInflation: ScaleAndPadding
    :ivar perShapeInflation: The configured per-shape scale and padding overrides.
    :vartype perShapeInflation: list
    """
    def __init__(self, defaultInflation=ScaleAndPadding(), defaultVisualInflation=ScaleAndPadding(),
                 defaultCollisionInflation=ScaleAndPadding(), perShapeInflation=None):
        """A registry of scaling and padding settings for shapes (visual/collision elements).

        :param defaultInflation: Default scale and padding to be used for shapes without explicit override.
                                  You can also pass an instance of __ShapeInflationRegistry to initialize this class
                                  from values contained there.
        :type defaultInflation: ScaleAndPadding or _ShapeInflationRegistry
        :param ScaleAndPadding defaultVisualInflation: Default scale and padding to be used for visuals without an
                                                       explicit override.
        :param ScaleAndPadding defaultCollisionInflation: Default scale and padding to be used for collisions without an
                                                          explicit override.
        :param list perShapeInflation: The configured per-shape scale and padding overrides.
        """
        if isinstance(defaultInflation, _ShapeInflationRegistry):
            conf = defaultInflation
            self.defaultInflation = conf.defaultInflation
            self.defaultVisualInflation = conf.defaultVisualInflation
            self.defaultCollisionInflation = conf.defaultCollisionInflation
            self.perShapeInflation = []
            for i in range(conf.perShapeInflationCount):
                self.perShapeInflation.append(PerShapeInflation(conf.perShapeInflation[i]))
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
        c.perShapeInflation = ctypes.cast(
            (_PerShapeInflation * c.perShapeInflationCount)(), POINTER(_PerShapeInflation))
        for i in range(len(obj.perShapeInflation)):
            c.perShapeInflation[i] = PerShapeInflation.from_param(obj.perShapeInflation[i])
        return c


class _RobotModelRendererConfig(Structure):
    """ctypes interop with struct robot_model_renderer_RobotModelRendererConfig.

    :note: Do not use directly in user code. Use ShapeInflationRegistry instead.
    """
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
        ("upscalingInterpolation", c_int),
        ("renderImageScale", c_double),
        ("maxRenderImageSize", c_size_t),
        ("staticMaskImageWidth", c_size_t),
        ("staticMaskImageHeight", c_size_t),
        ("staticMaskImageStep", c_size_t),
        ("staticMaskImageCVType", c_int),
        ("staticMaskImage", c_void_p),
        ("staticMaskImageEncoding", c_char_p),
        ("staticMaskIsBackground", c_bool),
        ("renderedImageIsStatic", c_bool),
    ]

    def __init__(self, *args, **kwargs):
        conf = _get_library().robot_model_renderer_createDefaultRobotModelRendererConfig()
        for field, _ in self._fields_:
            setattr(self, field, getattr(conf, field))
        super(_RobotModelRendererConfig, self).__init__(*args, **kwargs)


_simple_types = (c_bool, c_float, c_double, c_int, c_uint, c_uint8, c_uint32, c_size_t)


class RobotModelRendererConfig(object):
    """Configuration of RobotModelRenderer.

    :ivar setupDefaultLighting: If true, a default point and ambient light will be added.
    :vartype setupDefaultLighting: bool

    :ivar pixelFormat: Pixel format of the rendered image (one of Ogre::PF_* constants).
    :vartype pixelFormat: int
    :ivar backgroundColor: Color of pixels that are not a part of the robot model (RGBA, 0-1).
    :vartype backgroundColor: list

    :ivar doDistort: Apply lens distortion to the rendered images. If false, the output images are rectified.
    :vartype doDistort: bool
    :ivar gpuDistortion: Do the lens distortion on GPU.
    :vartype gpuDistortion: bool

    :ivar nearClipDistance: Near clip plane of the camera (meters).
    :vartype nearClipDistance: float
    :ivar farClipDistance: Far clip plane of the camera (meters). 0.0 means infinity.
    :vartype farClipDistance: float

    :ivar renderingMode: The mode of rendering.
    :vartype renderingMode: RenderingMode
    :ivar colorModeColor: Color of the robot that will be used in COLOR mode (RGBA, 0-1).
    :vartype colorModeColor: list

    :ivar drawOutline: Whether to draw an outline.
    :vartype drawOutline: bool
    :ivar outlineWidth: Width of the outline.
    :vartype outlineWidth: float
    :ivar outlineColor: Color of the outline (RGBA, 0-1).
    :vartype outlineColor: list
    :ivar outlineFromClosestColor: Whether outline color should be taken from the nearest on-robot pixel.
    :vartype outlineFromClosestColor: bool

    :ivar invertColors: Invert RGB channel values.
    :vartype invertColors: bool
    :ivar invertAlpha: Invert alpha channel values.
    :vartype invertAlpha: bool

    :ivar allLinksRequired: If true, all links from URDF are required to have a valid TF for rendering to happen.
    :vartype allLinksRequired: bool
    :ivar requiredLinks: The list of links whose TFs are required for the rendering to happen.
    :vartype requiredLinks: list

    :ivar shapeFilter: Filter of visuals/collisions. Non-matching shapes are not rendered.
    :vartype shapeFilter: ShapeFilter
    :ivar shapeInflationRegistry: Registry of scaling and padding parameters for individual links/visuals/collisions.
    :vartype shapeInflationRegistry: ShapeInflationRegistry

    :ivar upscalingInterpolation: Interpolation method used of upscaling (one of cv2.INTER_* constants).
    :vartype upscalingInterpolation: int
    :ivar renderImageScale: If lower than 1.0, scale down the rendered image to save resources.
    :vartype renderImageScale: float
    :ivar maxRenderImageSize: If non-zero, sets the maximum size of render textures. Decreases quality.
    :vartype maxRenderImageSize: int

    :ivar staticMaskImage: If non-null, this image will be drawn over/below the robot model.
    :vartype staticMaskImageWidth: np.ndarray
    :ivar staticMaskImageEncoding: Encoding from sensor_msgs/image_encodings.h . If empty, BGR(A) is assumed.
    :vartype staticMaskImageEncoding: str
    :ivar staticMaskIsBackground: If false, the static mask image will be drawn over the rendered image.
    :vartype staticMaskIsBackground: bool

    :ivar renderedImageIsStatic: If true, cache rendered images for identical camera geometry.
    :vartype renderedImageIsStatic: bool
    """
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
        self.upscalingInterpolation = conf.upscalingInterpolation
        self.renderImageScale = conf.renderImageScale
        self.maxRenderImageSize = conf.maxRenderImageSize
        self.staticMaskImage = None  # An OpenCV-compatible Numpy array
        self.staticMaskImageEncoding = ""
        self.staticMaskIsBackground = conf.staticMaskIsBackground
        self.renderedImageIsStatic = conf.renderedImageIsStatic

    @classmethod
    def from_param(cls, obj):
        c = _RobotModelRendererConfig()
        for f, t in _RobotModelRendererConfig._fields_:
            if t in _simple_types and hasattr(obj, f):
                setattr(c, f, getattr(obj, f))

        c.backgroundColor = (c_float * 4)(*obj.backgroundColor)
        c.colorModeColor = (c_float * 4)(*obj.colorModeColor)
        c.outlineColor = (c_float * 4)(*obj.outlineColor)
        obj._requiredLinks = ",".join(obj.requiredLinks).encode("utf-8")
        c.requiredLinks = obj._requiredLinks
        c.shapeFilter = ShapeFilterConfig.from_param(obj.shapeFilter)
        c.shapeInflationRegistry = ShapeInflationRegistry.from_param(obj.shapeInflationRegistry)

        if obj.staticMaskImage is not None and obj.staticMaskImage.size > 0:
            import cv2
            import cv_bridge
            import numpy as np

            bridge = cv_bridge.CvBridge()
            image = obj.staticMaskImage
            assert isinstance(image, (np.ndarray, np.generic))

            c.staticMaskImageWidth = image.shape[1]
            c.staticMaskImageHeight = image.shape[0]
            c.staticMaskImageStep = image.ctypes.strides_as(c_size_t)[0]
            obj._staticMaskImage = image.ctypes.data_as(c_void_p)
            c.staticMaskImage = obj._staticMaskImage
            obj._staticMaskImageEncoding = obj.staticMaskImageEncoding.encode("utf-8")
            c.staticMaskImageEncoding = obj._staticMaskImageEncoding

            cv_type_str = bridge.numpy_type_to_cvtype[image.dtype.name]
            n_channels = image.shape[2] if len(image.shape) == 3 else 1
            c.staticMaskImageCVType = getattr(cv2, "CV_%sC%i" % (cv_type_str, n_channels))

        return c


class LinkError(object):
    """Error updating a link.

    :ivar name: The name of the link.
    :vartype name: str
    :ivar error: The error message.
    :vartype error: str
    :ivar maySucceedLater: Whether asking later (with more TFs) has a chance to succeed.
    :vartype maySucceedLater: bool
    """

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
