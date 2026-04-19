@0x8b8af80462604d48;

using Header = import "/timestamp_header.capnp";
using P = import "/proprioception.capnp";
using RGB = import "/rgb_image_data.capnp";
using RGBD = import "/rgbd_image_data.capnp";

struct ObservationV1 {
    header @0 :Header.TimestampHeaderV1;

    proprioception :union {
        none @1 :Void;
        some @2 :P.ProprioceptionV1;
    }
    rgbImage :union {
        none @3 :Void;
        some @4 :RGB.RgbImageDataV1;
    }
    rgbdImage :union {
        none @5 :Void;
        some @6 :RGBD.RgbdImageDataV1;
    }
}

struct VersionedObservation {
    union {
        unset @0 :Void;
        v1    @1 :ObservationV1;
    }
}
