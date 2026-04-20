@0xa3980f723764a3c7;

using Header = import "/timestamp_header.capnp";
using JP = import "/joint_positions.capnp";
using JV = import "/joint_velocities.capnp";
using EP = import "/eef_pose.capnp";
using ET = import "/eef_twist.capnp";

struct CommandV1 {
    header @0 :Header.VersionedTimestampHeader;

    union {
        jointPositions  @1 :JP.VersionedJointPositions;
        jointVelocities @2 :JV.VersionedJointVelocities;
        eefPose         @3 :EP.VersionedEEFPose;
        eefTwist        @4 :ET.VersionedEEFTwist;
    }
}

struct VersionedCommand {
    union {
        unset @0 :Void;
        v1    @1 :CommandV1;
    }
}
