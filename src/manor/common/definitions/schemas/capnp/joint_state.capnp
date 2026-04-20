@0xd308f77bf52c226e;

using Header = import "/timestamp_header.capnp";
using JP = import "/joint_positions.capnp";
using JV = import "/joint_velocities.capnp";

struct JointStateV1 {
    header          @0 :Header.VersionedTimestampHeader;
    jointPositions  @1 :JP.VersionedJointPositions;
    jointVelocities @2 :JV.VersionedJointVelocities;
}

struct VersionedJointState {
    union {
        unset @0 :Void;
        v1    @1 :JointStateV1;
    }
}
