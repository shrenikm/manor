@0xf7ebb38f526e99f6;

using Header = import "/timestamp_header/timestamp_header_v1.capnp";
using JP = import "/joint_positions/joint_positions_v1.capnp";
using JPT = import "/joint_positions_trajectory/joint_positions_trajectory_v1.capnp";
using JV = import "/joint_velocities/joint_velocities_v1.capnp";
using JVT = import "/joint_velocities_trajectory/joint_velocities_trajectory_v1.capnp";
using EP = import "/eef_pose/eef_pose_v1.capnp";
using EPT = import "/eef_pose_trajectory/eef_pose_trajectory_v1.capnp";
using ET = import "/eef_twist/eef_twist_v1.capnp";
using ETT = import "/eef_twist_trajectory/eef_twist_trajectory_v1.capnp";

struct ActionV1 {
    header @0 :Header.TimestampHeaderV1;

    union {
        jointPositions           @1 :JP.JointPositionsV1;
        jointPositionsTrajectory @2 :JPT.JointPositionsTrajectoryV1;
        jointVelocities          @3 :JV.JointVelocitiesV1;
        jointVelocitiesTrajectory @4 :JVT.JointVelocitiesTrajectoryV1;
        eefPose                  @5 :EP.EefPoseV1;
        eefPoseTrajectory        @6 :EPT.EefPoseTrajectoryV1;
        eefTwist                 @7 :ET.EefTwistV1;
        eefTwistTrajectory       @8 :ETT.EefTwistTrajectoryV1;
    }
}
