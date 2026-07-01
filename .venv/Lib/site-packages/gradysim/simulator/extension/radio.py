"""
This module declares a plugin for the protocol that allows a protocol to instantiate multiple radios, each with their own
communication characteristics. This plugin is only available in a Python simulation environment and will raise an
error if used in other environments. Alternative implementations should be provided for other simulation environments,
"""
import copy
from typing import Optional

from gradysim.encapsulator.python import PythonProvider
from gradysim.protocol.interface import IProtocol
from gradysim.protocol.messages.communication import CommunicationCommand
from gradysim.simulator.handler.communication import CommunicationMedium, CommunicationHandler

class Radio:
    """
    A plugin that allows a protocol to instantiate multiple radios, each with their own communication characteristics.

    Multiple radios can be instantiated in a single protocol. Messages sent through the radio will use the radio's
    communication characteristics, such as transmission range. Messages sent through other radios or directly through
    the protocol will not be affected by the radio's characteristics.

    This plugin is only available in a Python simulation environment and will raise an error if used in other
    environments. Alternative implementations should be provided for other simulation environments, ones that
    interface with real hardware radios or other communication systems.

    !!!warning
        This plugin can only be used in a Python simulation environment.
    """
    _radio_medium: CommunicationMedium
    _communication_handler: CommunicationHandler

    def __init__(self, protocol: IProtocol):
        """
        Initializes the Radio plugin.
        """

        provider = protocol.provider
        if not isinstance(provider, PythonProvider):
            raise TypeError("Radio plugin can only be used in a Python simulation environment.")
        self._provider = provider
        communication_handler: Optional[CommunicationHandler] = self._provider.handlers.get("communication")
        if communication_handler is None or not isinstance(communication_handler, CommunicationHandler):
            raise RuntimeError("The radio extension is only compatible with the Python Simulator and a "
                               "CommunicationHandler has to be present")

        self._communication_handler = communication_handler

        self._radio_medium = copy.copy(self._communication_handler.default_medium)

    def set_configuration(self,
                          transmission_range: float = None,
                          delay: float = None,
                          failure_rate: float = None) -> None:
        """
        Sets a new configuration for the radio. Any parameter set to None will keep its previous value.

        Args:
            transmission_range: Maximum range in meters for message delivery. Messages destined to nodes outside this range will not be delivered.
            delay: Sets a delay in seconds for message delivery, representing network delay. Range is evaluated before the delay is applied.
            failure_rate: Failure chance between 0 and 1 for message delivery. 0 represents messages never failing and 1 always fails.
        """
        if transmission_range is not None:
            self._radio_medium.transmission_range = transmission_range
        if delay is not None:
            self._radio_medium.delay = delay
        if failure_rate is not None:
            self._radio_medium.failure_rate = failure_rate

    def send_communication_command(self, command: CommunicationCommand) -> None:
        """
        Sends a message via the radio. Messages sent through the radio function identically to those sent through
        the provider, but will use the radio's communication characteristics.

        Args:
            command: The communication command to send. Same CommunicationCommand used
                     in [IProvider][gradysim.protocol.interface.IProvider]
        """
        self._communication_handler.handle_command(command, self._provider.node, self._radio_medium)
