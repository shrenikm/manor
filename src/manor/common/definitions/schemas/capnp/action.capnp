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
    header @0 :Header.VersionedTimestampHeader;

    union {
        jointPositions            @1 :JP.VersionedJointPositions;
        jointPositionsTrajectory  @2 :JPT.VersionedJointPositionsTrajectory;
        jointVelocities           @3 :JV.VersionedJointVelocities;
        jointVelocitiesTrajectory @4 :JVT.VersionedJointVelocitiesTrajectory;
        eefPose                   @5 :EP.VersionedEEFPose;
        eefPoseTrajectory         @6 :EPT.VersionedEEFPoseTrajectory;
        eefTwist                  @7 :ET.VersionedEEFTwist;
        eefTwistTrajectory        @8 :ETT.VersionedEEFTwistTrajectory;
    }
}

struct VersionedAction {
    union {
        unset @0 :Void;
        v1    @1 :ActionV1;
    }
}
