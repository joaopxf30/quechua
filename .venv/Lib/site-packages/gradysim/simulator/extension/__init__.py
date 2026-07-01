"""
Simulator extensions or simply *extensions* are modules that extend the functionality of the **python** simulator.
They are used to implement new features or to provide new ways to interact with the simulation environment.

Extensions are akin to [plugins][gradysim.protocol.plugin], but instead of expanding the functionality
of a protocol they act on a simulation-level to implement new features that protocols can use. While
plugins are designed to be environment-agnostic, extensions are tied to the python simulation environment,
this is reflected in the fact that extensions are located within the `simulator` package. Extensions can be used
to implement new simulation features, such as new types of sensors, actuators, or other simulation components.

Extensions are implemented as classes that inherit from the [Extension][gradysim.simulator.extension.extension.Extension]
class. Extensions are attached to a protocol instance but have ways of interacting with the simulation environment
that the protocol does not. In practice, extensions can directly access [handlers][gradysim.simulator.handler] and
modify the simulation environment.

!!!warning
    Extensions are attached to an initialized protocol. Instantiating an extension on an uninitialized protocol will
    raise a `ReferenceError`.

!!!info
    Most extensions rely on a specific handler being present in the simulation. Check their own documentation to see
    which handlers they rely on.
"""