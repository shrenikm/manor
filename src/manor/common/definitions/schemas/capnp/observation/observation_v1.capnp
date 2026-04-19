@0x80679935f03eaa25;

using Header = import "/timestamp_header/timestamp_header_v1.capnp";
using P = import "/proprioception/proprioception_v1.capnp";
using RGB = import "/rgb_image_data/rgb_image_data_v1.capnp";
using RGBD = import "/rgbd_image_data/rgbd_image_data_v1.capnp";

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
