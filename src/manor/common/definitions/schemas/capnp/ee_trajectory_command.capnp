@0xe6d39dcfc0eaf2ec;

using Header = import "/timestamp_header.capnp";
using EPT = import "/ee_positions_trajectory.capnp";
using EVT = import "/ee_velocities_trajectory.capnp";

struct EETrajectoryCommandV1 {
    header @0 :Header.VersionedTimestampHeader;

    union {
        eePositionsTrajectory  @1 :EPT.VersionedEEPositionsTrajectory;
        eeVelocitiesTrajectory @2 :EVT.VersionedEEVelocitiesTrajectory;
    }
}

struct VersionedEETrajectoryCommand {
    union {
        unset @0 :Void;
        v1    @1 :EETrajectoryCommandV1;
    }
}
