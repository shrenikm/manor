@0x868dabcfab74771f;

using V1 = import "depth_image_data_v1.capnp";

struct VersionedDepthImageData {
    union {
        unset @0 :Void;
        v1    @1 :V1.DepthImageDataV1;
    }
}
