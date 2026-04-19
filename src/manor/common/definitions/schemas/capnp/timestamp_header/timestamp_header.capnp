@0x9200fe87de06de71;

using V1 = import "timestamp_header_v1.capnp";

struct VersionedTimestampHeader {
    union {
        # Reserved sentinel -- Cap'n Proto unions require at least two members.
        # Never set in practice; deserialization treats this arm as an error.
        unset @0 :Void;
        v1    @1 :V1.TimestampHeaderV1;
    }
}
