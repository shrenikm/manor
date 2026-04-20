"""
Enumerations used by image-related definitions.
"""

from enum import StrEnum


class ImageEncoding(StrEnum):
    """
    Pixel encoding for RGBImageData.

    RAW_* variants carry raw pixel bytes in the named channel order.
    RGB and BGR have identical shape and byte count; the tag exists purely
    to disambiguate channel order (OpenCV defaults to BGR; most other
    libraries use RGB).

    JPEG / PNG variants carry compressed container bytes. The stream itself
    is colorspace-neutral at rest; the decoded pixel order depends on the
    decoder library the consumer uses (PIL decodes to RGB, OpenCV's imdecode
    to BGR).
    """

    RAW_RGB8 = "raw_rgb8"
    RAW_BGR8 = "raw_bgr8"
    JPEG = "jpeg"
    PNG = "png"


class DepthEncoding(StrEnum):
    """
    Encoding for DepthImageData. Scale to meters is carried separately in
    depth_scale; the unit suffix on each variant documents the conventional
    unit a well-behaved producer emits.
    """

    RAW_FLOAT32_M = "raw_float32_m"
    RAW_UINT16_MM = "raw_uint16_mm"
    PNG_UINT16_MM = "png_uint16_mm"
