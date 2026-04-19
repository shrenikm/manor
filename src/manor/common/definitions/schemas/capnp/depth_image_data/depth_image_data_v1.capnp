@0x87e2deb092fbcc09;

using Header = import "/timestamp_header/timestamp_header_v1.capnp";

enum DepthEncodingV1 {
    rawFloat32M @0;
    rawUint16Mm @1;
    pngUint16   @2;
}

struct DepthImageDataV1 {
    header     @0 :Header.TimestampHeaderV1;
    height     @1 :UInt32;
    width      @2 :UInt32;
    encoding   @3 :DepthEncodingV1;
    data       @4 :Data;
    depthScale @5 :Float64;
}
