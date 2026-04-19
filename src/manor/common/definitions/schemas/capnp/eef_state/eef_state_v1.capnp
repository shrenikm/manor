@0x90c989bf4d389c92;

using Header = import "/timestamp_header/timestamp_header_v1.capnp";
using EP = import "/eef_positions/eef_positions_v1.capnp";
using EV = import "/eef_velocities/eef_velocities_v1.capnp";

struct EefStateV1 {
    header        @0 :Header.TimestampHeaderV1;
    eefPositions  @1 :EP.EefPositionsV1;
    eefVelocities @2 :EV.EefVelocitiesV1;
}
