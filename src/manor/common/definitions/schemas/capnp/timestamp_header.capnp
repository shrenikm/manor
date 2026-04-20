@0x9200fe87de06de71;

struct TimestampHeaderV1 {
    monotonicNs @0 :Int64;
    systemNs    @1 :Int64;
}

struct VersionedTimestampHeader {
    union {
        # Reserved sentinel -- Cap'n Proto unions require at least two members.
        # Never set in practice; deserialization treats this arm as an error.
        unset @0 :Void;
        v1    @1 :TimestampHeaderV1;
    }
}
