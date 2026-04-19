@0xf765dd607ed971d4;

using Header = import "/timestamp_header/timestamp_header_v1.capnp";
using JS = import "/joint_state/joint_state_v1.capnp";
using ES = import "/eef_state/eef_state_v1.capnp";
using EP = import "/eef_pose/eef_pose_v1.capnp";
using ET = import "/eef_twist/eef_twist_v1.capnp";

struct ProprioceptionV1 {
    header     @0 :Header.TimestampHeaderV1;
    jointState @1 :JS.JointStateV1;

    eefState :union {
        none @2 :Void;
        some @3 :ES.EefStateV1;
    }
    eefPose :union {
        none @4 :Void;
        some @5 :EP.EefPoseV1;
    }
    eefTwist :union {
        none @6 :Void;
        some @7 :ET.EefTwistV1;
    }
}
