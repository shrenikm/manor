@0xdf8933d4a3418239;

using Header = import "/timestamp_header/timestamp_header_v1.capnp";
using JP = import "/joint_positions/joint_positions_v1.capnp";
using JV = import "/joint_velocities/joint_velocities_v1.capnp";

struct JointStateV1 {
    header          @0 :Header.TimestampHeaderV1;
    jointPositions  @1 :JP.JointPositionsV1;
    jointVelocities @2 :JV.JointVelocitiesV1;
}
