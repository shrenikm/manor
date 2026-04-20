@0xe111e26a4e054f79;

using Header = import "/timestamp_header.capnp";
using EP = import "/eef_positions.capnp";
using EV = import "/eef_velocities.capnp";

struct EEFStateV1 {
    header        @0 :Header.TimestampHeaderV1;
    eefPositions  @1 :EP.EEFPositionsV1;
    eefVelocities @2 :EV.EEFVelocitiesV1;
}

struct VersionedEEFState {
    union {
        unset @0 :Void;
        v1    @1 :EEFStateV1;
    }
}
