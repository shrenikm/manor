@0xa3980f723764a3c7;

using V1 = import "command_v1.capnp";

struct VersionedCommand {
    union {
        unset @0 :Void;
        v1    @1 :V1.CommandV1;
    }
}
