from gradysim.simulator.extension.visualization_controller import VisualizationController

from utils.viz_templates.constants import COLOR_UAV
from utils.actor_templates.uav_template import make_uav_protocol


class UAVVizMixin:
    """
    Mixin that adds visualization (node colour) to any UAV protocol.

    This mixin should be placed BEFORE the base protocol in the MRO so that
    its initialize() and handle_timer() run first and then delegate to the
    base protocol via super().

    Usage (direct):
        class MyUAVViz(UAVVizMixin, MyUAVProtocol): pass

    Usage (via make_uav_protocol_viz factory — recommended):
        UAVViz = make_uav_protocol_viz(waypoints, departure_delay=0.0)
        builder.add_node(UAVViz, start_pos)
    """

    def initialize(self) -> None:
        super().initialize()
        self._vc = VisualizationController(self)
        self.provider.schedule_timer("repaint", self.provider.current_time() + 1)

    def handle_timer(self, timer: str) -> None:
        super().handle_timer(timer)
        if timer == "repaint":
            self._repaint()
            self.provider.schedule_timer("repaint", self.provider.current_time() + 1)

    def _repaint(self) -> None:
        """
        Paint this node based on its current state.
        Override to add colour-coded states (e.g. charging, low battery).
        """
        self._vc.paint_node(self.provider.get_id(), COLOR_UAV)


def make_uav_protocol_viz(
    waypoints,
    cluster_index: int = 0,
    num_uavs: int = 1,
    departure_delay: float = 0.0,
) -> type:
    """
    Convenience factory that combines UAVVizMixin with make_uav_protocol.

    All arguments are forwarded directly to make_uav_protocol.
    The returned class is ready to pass to SimulationBuilder.add_node().
    """
    BaseProtocol = make_uav_protocol(
        waypoints      = waypoints,
        cluster_index  = cluster_index,
        num_uavs       = num_uavs,
        departure_delay= departure_delay,
    )

    class _UAVProtocolViz(UAVVizMixin, BaseProtocol):
        pass

    _UAVProtocolViz.__name__ = f"UAVProtocolViz_slot{cluster_index}"
    return _UAVProtocolViz