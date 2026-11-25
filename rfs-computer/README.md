# Remote Filling Station Computer

This folder contains software for the electronics of the RFS.
It leans heavily on the `lib_mission` and the design of the flight computer.

## Uremote

An open source library that we extended is used to connect with the uremote coupler ur20 from Weidmüller which drives digital outputs and inputs, as well as thermocouple inputs.

It is polled at a configurable frequency and automatically reconnects the TCP socket upon connection loss.

## Launch Rail Electronics

There is another smart junction box below the launch rail which contains a Controllino MAXI Automation,
a open source PLC system compatible with Arduino.
It acts purely as a server of the main RFS software's requests.

It does this over over a Modbus TCP connection with several coils for actuators, discrete inputs for proximity and arming feedback sensors and holding registers for pressure and temperature readings.
The complete list is provided in the "Instrumentation Naming" excel sheet.
The sketch running on the Controllino constantly reads the newest request,
sets the actuators accordingly and reads the pressures and temperatures.
It also ensures that the mutually exlusive valves cannot be activated at the same time,
and instead toggles off the previously on one after a deadtime delay.

On the RFS side, a four tasks periodically poll latest data at different rates for actuator, digital inputs, pressures and temperatures.
Additionally, whenever a valve is supposed to actuate, a write request is sent.
Whenever a read or write request is sent, the connection can automatically be reestablished, if it was broken,
and all messages are logged in our standardized format.
