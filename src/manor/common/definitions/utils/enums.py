"""
Enumerations used by image-related definitions.
"""

from enum import StrEnum


class ImageEncoding(StrEnum):
    """
    Pixel encoding for RGBImageData.
    """

    RAW_RGB8 = "raw_rgb8"
    RAW_BGR8 = "raw_bgr8"
    JPEG = "jpeg"
    PNG = "png"


class DepthEncoding(StrEnum):
    """
    Encoding for DepthImageData. Scale to meters is carried separately.
    """

    RAW_FLOAT32_M = "raw_float32_m"
    RAW_UINT16_MM = "raw_uint16_mm"
    PNG_UINT16 = "png_uint16"
