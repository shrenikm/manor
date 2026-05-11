@0x863fe3bcbe2b6a15;

using Header = import "/timestamp_header.capnp";
using JP = import "/joint_positions.capnp";
using JV = import "/joint_velocities.capnp";

struct JointCommandV1 {
    header @0 :Header.VersionedTimestampHeader;

    union {
        jointPositions  @1 :JP.VersionedJointPositions;
        jointVelocities @2 :JV.VersionedJointVelocities;
    }
}

struct VersionedJointCommand {
    union {
        unset @0 :Void;
        v1    @1 :JointCommandV1;
    }
}
