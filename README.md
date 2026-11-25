# ASTERIA Remote Computers

This repository contains the code for the flight computer in the rocket and the RFS computer in the remote filling station, and all code shared between them.

> [!NOTE]
> This software was developed by the ASTERIA Software team specifically for our purposes at ARIS for EuRoC 2026.
> No guarantees are given, and external contributions are unlikely to be accepted.
> Additionally, this will very likely not be maintained beyond 2026, see other repositories.
>
> Point of contact: ASTERIA Software, e.g. cyrill.bucher@aris-space.ch

## Architecture

The Software Team is responsible for all software systems integrated within the launch system, spanning three main areas.

The first covers the remote computers (FC and the RFS Computer) including their interfaces with onboard sensors and actuators as well as the data links between remote systems and mission control. The second encompasses the MC software itself, comprising the user interface, while the third focuses on the data pipeline, including graphing and storage infrastructure.

The FC on a Raspberry Pi communicates with one instance of the MC running on a laptop while the RFS computer communicates with an independent one. However, they are connected on the same CAN bus and can also communicate with each other.


The systems are organized in separated repositories as shown in the software repositories figure. The common data framework `data-definitions` defines the messages and is the core of the launch system.

### Communication Framework

> `./data-definitions` submodule

The message architecture has three layers. The datatype layer defines core Rust types (like possible valve states or temperature representations) with shared traits and methods. The datapoint layer has one crate per data system, each declaring the messages it sends and receives. The application layer then lets a system pick the datapoints it needs, either the complete set from all other systems or a sparse subset for constrained firmware.

CAN connects the avionics firmware to remote computers, using a fixed-size encoding based on an open source library. The IDs are assigned to messages and are ordered by priority, from system management and event messages down to sensor data, debug metadata, and reserved ranges (table in the system data appendix).

MAVLink connects the remote computers further to mission control via a custom dialect derived from the datapoints. Types recursively derive schema metadata and encoding/decoding, and support primitives, arrays, and numeric enums. IDs are disjoint from other dialects and assigned only at the message definition. A `mavlink-hal` crate manages the actual encoding/decoding across four alignment sizes according to the spec, but we are using an open source library for framing with a wrapper to make it easy to use.

Another component annotates some of the datapoints, such as sensors and actuators, as controllable or viewable widgets in the mission control and assigns a textual ID to them (such as `fss_tnk_vnt` for the fuel tank vent valve). This is also used to log the data.

The framework ships as a library embedded as a submodule across firmware, remote computers and mission control, built with feature flags to exclude CAN, MAVLink or the widget information when not needed. CI runs rustfmt, clippy, and tests via GitHub Actions. More details are in the communication framework chapter.

### Flight Computer

> `./flight-computer`

The main flight computer runs as a Rust binary on a Pi Compute Module with Raspbian (Linux) installed. Its main responsibilities are to collect and forward data received from the various boards to mission control (see the flight computer data flow figure), and to run the logic of the state machine.

The state machine is defined by a TOML configuration file which defines the state names, and selects a logic class along with parameters for each one. That logic checks transition criteria, such as if a sensor threshold has been reached,
manual transition and possibly manual actuator commands which are sent from the MC, and might also set valve and other actuator states.

Some of them also run sequences. A sequence is a short script embedded as a string in a state's parameter entry, used where an operation is a fixed number of typed commands: waiting, opening and closing actuators, activating or deactivating a dynamic pressure regulator, and commands to set up and ignore sensor thresholds or wait for a specific sensor value. All of it runs inside the state's usual race against manual transitions, so an abort from the ground cancels it wherever it has got to.

Each config file has a diagram alongside it drawn in draw.io and saved as an Editable Vector Image. The state machine chapter shows the one of the flight computer used for the EuRoC flight. The flight computer and RFS computer chapter further describes the functionality of the components.

### RFS Computer

> `./rfs-computer`

The RFS computer is also a Rust binary on a Raspberry Pi, and shares the architecture and a core library with the FC.

Unlike the Flight Computer, however, most of its hardware is not on custom boards via CAN. A Weidmüller u-remote coupler carries most of the valves, the spark plug and the thermocouples over Modbus TCP. A Controllino in the smart box below the launch rail controls the decouplers, the shower and engine purge valves, rocket power and heating, also over Modbus TCP. The Keller pressure transducers are polled over Modbus RTU on a serial bus. The throttleable LOX fill valve shares this bus.

Beyond that, it also speaks CAN to the rocket. During filling, the RFSC commands the valves inside the rocket directly over CAN; once the vehicle is armed for flight, the Flight Computer takes over and instead controls the igniter on the RFS. CAN messages are duplicated to carry the identity of the sender, so the two do not interfere.

### Safety Considerations

A core goal of the software architecture is safety. Every state of the state machine has a subset of allowed state transitions and manual actuators.

We enforce this through multiple levels. Firstly, the MC UI only shows the allowed actions and the rest are not interactable. Secondly, the message for such an action needs to include the number of state transitions that have happened so far. This prevents old commands from taking effect later on again. Thirdly, the state machine logic also just forwards the allowed actions to the hardware, even if a message were to be sent for a disallowed one. Lastly, the hardware has arming circuits which the software also shows, and without it, nothing moves anyway.

