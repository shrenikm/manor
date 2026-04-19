@0xd9bc8af60606bb56;

using Header = import "/timestamp_header/timestamp_header_v1.capnp";
using RGB = import "/rgb_image_data/rgb_image_data_v1.capnp";
using Depth = import "/depth_image_data/depth_image_data_v1.capnp";

struct RgbdImageDataV1 {
    header @0 :Header.TimestampHeaderV1;
    rgb    @1 :RGB.RgbImageDataV1;
    depth  @2 :Depth.DepthImageDataV1;
}
