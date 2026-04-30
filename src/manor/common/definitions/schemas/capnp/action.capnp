@0xd3c86a063a393cb4;

using Header = import "/timestamp_header.capnp";
using JC = import "/joint_command.capnp";
using JTC = import "/joint_trajectory_command.capnp";
using CC = import "/cartesian_command.capnp";
using CTC = import "/cartesian_trajectory_command.capnp";
using EC = import "/ee_command.capnp";
using ETC = import "/ee_trajectory_command.capnp";

# Action carries one of four group-1 arm commands (required) and at most
# one group-2 ee command (optional).
struct ActionV1 {
    header @0 :Header.VersionedTimestampHeader;

    arm :union {
        jointCommand                @1 :JC.VersionedJointCommand;
        jointTrajectoryCommand      @2 :JTC.VersionedJointTrajectoryCommand;
        cartesianCommand            @3 :CC.VersionedCartesianCommand;
        cartesianTrajectoryCommand  @4 :CTC.VersionedCartesianTrajectoryCommand;
    }

    ee :union {
        none                @5 :Void;
        eeCommand           @6 :EC.VersionedEECommand;
        eeTrajectoryCommand @7 :ETC.VersionedEETrajectoryCommand;
    }
}

struct VersionedAction {
    union {
        unset @0 :Void;
        v1    @1 :ActionV1;
    }
}
