@0xd3c86a063a393cb4;

using V1 = import "action_v1.capnp";

struct VersionedAction {
    union {
        unset @0 :Void;
        v1    @1 :V1.ActionV1;
    }
}
