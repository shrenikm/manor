@0xa6000f69a9fff8f9;

# Unversioned shared Cap'n Proto types used by every versioned definition schema.
# Changes here are breaking changes -- treat these structs as stable.

struct Float64Array {
    shape @0 :List(UInt32);
    data  @1 :List(Float64);
}

struct UInt32Array {
    shape @0 :List(UInt32);
    data  @1 :List(UInt32);
}
