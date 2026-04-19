@0xbfb0689f7056528e;

using V1 = import "rgb_image_data_v1.capnp";

struct VersionedRgbImageData {
    union {
        unset @0 :Void;
        v1    @1 :V1.RgbImageDataV1;
    }
}
