@0xa22e6523fbb07f0d;

using V1 = import "rgbd_image_data_v1.capnp";

struct VersionedRgbdImageData {
    union {
        unset @0 :Void;
        v1    @1 :V1.RgbdImageDataV1;
    }
}
