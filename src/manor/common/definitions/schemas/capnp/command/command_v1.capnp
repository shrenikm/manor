@0x8a678795e4df844e;

using Header = import "/timestamp_header/timestamp_header_v1.capnp";
using JP = import "/joint_positions/joint_positions_v1.capnp";
using JV = import "/joint_velocities/joint_velocities_v1.capnp";
using EP = import "/eef_pose/eef_pose_v1.capnp";
using ET = import "/eef_twist/eef_twist_v1.capnp";

struct CommandV1 {
    header @0 :Header.TimestampHeaderV1;

    union {
        jointPositions  @1 :JP.JointPositionsV1;
        jointVelocities @2 :JV.JointVelocitiesV1;
        eefPose         @3 :EP.EefPoseV1;
        eefTwist        @4 :ET.EefTwistV1;
    }
}
