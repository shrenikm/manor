@0xd9cdde76041c9a7f;

using Header = import "/timestamp_header.capnp";
using JS = import "/joint_state.capnp";
using ES = import "/eef_state.capnp";
using EP = import "/eef_pose.capnp";
using ET = import "/eef_twist.capnp";

struct ProprioceptionV1 {
    header     @0 :Header.TimestampHeaderV1;
    jointState @1 :JS.JointStateV1;

    eefState :union {
        none @2 :Void;
        some @3 :ES.EEFStateV1;
    }
    eefPose :union {
        none @4 :Void;
        some @5 :EP.EEFPoseV1;
    }
    eefTwist :union {
        none @6 :Void;
        some @7 :ET.EEFTwistV1;
    }
}

struct VersionedProprioception {
    union {
        unset @0 :Void;
        v1    @1 :ProprioceptionV1;
    }
}
