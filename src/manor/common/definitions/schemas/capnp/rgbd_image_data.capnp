@0xa22e6523fbb07f0d;

using Header = import "/timestamp_header.capnp";
using RGB = import "/rgb_image_data.capnp";
using Depth = import "/depth_image_data.capnp";

struct RgbdImageDataV1 {
    header @0 :Header.TimestampHeaderV1;
    rgb    @1 :RGB.RgbImageDataV1;
    depth  @2 :Depth.DepthImageDataV1;
}

struct VersionedRgbdImageData {
    union {
        unset @0 :Void;
        v1    @1 :RgbdImageDataV1;
    }
}
