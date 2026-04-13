import sys
from contextlib import contextmanager
from typing import Callable, Generator, Optional

import attr
from pydrake.geometry import Meshcat, SceneGraph
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.framework import Diagram

from manr.common.exceptions import Lite6Error
from manr.common.logging_utils import MRLogger


@attr.define
class MultibodyPliantContainer:
    """
    For hardware, the plant, scene_graph and meshcat will all be None
    """

    pliant_diagram: Diagram
    plant: MultibodyPlant
    scene_graph: Optional[SceneGraph] = None
    meshcat: Optional[Meshcat] = None

    post_run_hook: Optional[Callable[[Diagram], None]] = None

    @contextmanager
    def auto_meshcat_visualization(
        self,
        record: bool,
    ) -> Generator[None, None, None]:
        """
        Sets up meshcat recording if a meshcat instance is given.

        Usage:
            with container.auto_meshcat_visualization(record=...):
                simulator.AdvanceTo(...)

        TODO: Maybe "record" needs to be renamed.
        If record is True, records and publishes the recording instead of playing it live.
        If False, plays the recording live.
        """
        if self.meshcat is not None and record:
            self.meshcat.StartRecording(set_visualizations_while_recording=False)
        try:
            yield
        except (Lite6Error, KeyboardInterrupt) as e:
            # If interrupted, we log, run the post hook cleanup and then exit.
            MRLogger(self.__class__.__name__).info(f"Simulation interrupted: {e}")
            if self.post_run_hook is not None:
                self.post_run_hook(self.pliant_diagram)
            sys.exit(1)

        # Post hook for any cleanup/reset in the simulation or hardware.
        if self.post_run_hook is not None:
            self.post_run_hook(self.pliant_diagram)

        if self.meshcat is not None and record:
            self.meshcat.StopRecording()
            self.meshcat.PublishRecording()
