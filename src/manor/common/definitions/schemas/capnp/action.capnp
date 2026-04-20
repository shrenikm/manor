@0xd3c86a063a393cb4;

using Header = import "/timestamp_header.capnp";
using JP = import "/joint_positions.capnp";
using JPT = import "/joint_positions_trajectory.capnp";
using JV = import "/joint_velocities.capnp";
using JVT = import "/joint_velocities_trajectory.capnp";
using EP = import "/eef_pose.capnp";
using EPT = import "/eef_pose_trajectory.capnp";
using ET = import "/eef_twist.capnp";
using ETT = import "/eef_twist_trajectory.capnp";

struct ActionV1 {
    header @0 :Header.TimestampHeaderV1;

    union {
        jointPositions            @1 :JP.JointPositionsV1;
        jointPositionsTrajectory  @2 :JPT.JointPositionsTrajectoryV1;
        jointVelocities           @3 :JV.JointVelocitiesV1;
        jointVelocitiesTrajectory @4 :JVT.JointVelocitiesTrajectoryV1;
        eefPose                   @5 :EP.EEFPoseV1;
        eefPoseTrajectory         @6 :EPT.EEFPoseTrajectoryV1;
        eefTwist                  @7 :ET.EEFTwistV1;
        eefTwistTrajectory        @8 :ETT.EEFTwistTrajectoryV1;
    }
}

struct VersionedAction {
    union {
        unset @0 :Void;
        v1    @1 :ActionV1;
    }
}
