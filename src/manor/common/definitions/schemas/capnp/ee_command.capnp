@0xdaa4776f357fd948;

using Header = import "/timestamp_header.capnp";
using EP = import "/ee_positions.capnp";
using EV = import "/ee_velocities.capnp";

struct EECommandV1 {
    header @0 :Header.VersionedTimestampHeader;

    union {
        eePositions  @1 :EP.VersionedEEPositions;
        eeVelocities @2 :EV.VersionedEEVelocities;
    }
}

struct VersionedEECommand {
    union {
        unset @0 :Void;
        v1    @1 :EECommandV1;
    }
}
