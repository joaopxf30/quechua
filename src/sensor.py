from utils.actor_templates.sensor_template import SensorProtocol
from utils.viz_templates.sensor_viz_template import SensorVizMixin

class Sensor(SensorVizMixin, SensorProtocol):
    """
    Example Sensor Protocol with added Visualization support.
    If you want to implement your own variation of the protocol,
    add stuff to this class or create another class that inherits
    from SensorProtocol and SensorVizMixin and mix in other
    behaviors you want.
    """
    pass