
from gradysim.simulator.extension.visualization_controller import VisualizationController

from utils.viz_templates.constants import COLOR_SENSOR, VIZ_NODE_SIZE

class SensorVizMixin:
    """
    Mix this into your SensorProtocol to add visualization support.
    Must be placed BEFORE SensorProtocol in the MRO:
        class MySensorViz(SensorVizMixin, MySensorProtocol): pass
    """
    def initialize(self) -> None:
        super().initialize()
        self._vc = VisualizationController(self)
        # Paint immediately so the node is never white on startup
        self._vc.paint_node(self.provider.get_id(), COLOR_SENSOR)
        # Resize all nodes to the configured size
        self._vc.resize_nodes(VIZ_NODE_SIZE)
        self.provider.schedule_timer("repaint", self.provider.current_time() + 1)

    def handle_timer(self, timer: str) -> None:
        super().handle_timer(timer)
        if timer == "repaint":
            self._vc.paint_node(self.provider.get_id(), COLOR_SENSOR)
            self.provider.schedule_timer("repaint", self.provider.current_time() + 1)
