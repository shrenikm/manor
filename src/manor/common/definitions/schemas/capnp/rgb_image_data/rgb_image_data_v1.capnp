@0x9b20640d585730f5;

using Header = import "/timestamp_header/timestamp_header_v1.capnp";

enum ImageEncodingV1 {
    rawRgb8 @0;
    rawBgr8 @1;
    jpeg    @2;
    png     @3;
}

struct RgbImageDataV1 {
    header   @0 :Header.TimestampHeaderV1;
    height   @1 :UInt32;
    width    @2 :UInt32;
    encoding @3 :ImageEncodingV1;
    data     @4 :Data;
}
