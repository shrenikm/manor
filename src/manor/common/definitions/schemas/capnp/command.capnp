@0xa3980f723764a3c7;

using Header = import "/timestamp_header.capnp";
using JC = import "/joint_command.capnp";
using EC = import "/ee_command.capnp";

struct CommandV1 {
    header       @0 :Header.VersionedTimestampHeader;
    jointCommand @1 :JC.VersionedJointCommand;

    eeCommand :union {
        none @2 :Void;
        some @3 :EC.VersionedEECommand;
    }
}

struct VersionedCommand {
    union {
        unset @0 :Void;
        v1    @1 :CommandV1;
    }
}
