# Flight Computer

This is the software that will run on the Raspberry Pi compute module on the rocket.


The Flight Computer (FC) is tasked to:

- Receive data from sensors on the rocket and on the remote filling station and stream them to the mission control
- Run the engine sequence of the rocket
- Enable the recovery GNC to run
- Drive the actuators as commanded by the mission control

# Architecture Overview


Actors listen to CAN messages and telemetry and update a shared datastructure, the `Input Tree`.
The `State Machine` has an active state which while running waits for changes in the input tree and based on these performs actions or decides for a transition.
The actions to perform are written to the `Output Tree`, which is listened to by `Output Drivers` or directly contains a channel, which then send CAN and telemetry messages.

Both the data trees are also listened to by loggers, which log the changes to disk and send them via telemetry.


# Components

## Main file

The main files set up a `tokio` runtime to enable the use of async code.
This allows easier cancellation of tasks and waiting for one of multiple conditions. However, it introduces an incompatiblity as existing non-async code cannot call the async functions.

It will then read in the config file, prints its hash and information about the build information, such as the git commit. This is used to verify the proper version of the code has been run.

Afterwards the threads and drivers for CAN, MavLink and logging are initialized before starting the main loop of the state machine.



## State Machine

The implementation of the state machine is split into multiple parts, the first one is shared between the RFSC and FC: 

- The `lib-mission` crate, which contains the state trait definition and main loop, which fetches the next state and waits for it to finish.
- The `StateLogic` blocks, which implement the state trait and receive references to the input and output tree, as well as a parameter struct.
- The configuration definition, which summarizes all available logic blocks and their options.
- The configuration files which define the mission profile, i.e. the names of the states, which logic block they use and where they will transition. They can also define which transitions can be manually performed and which actuators can be manually moved.


### Reactive utilities

Under the hood we are using the `watchable` library to have data we can watch for changes. This works both in async and non-async contexts. However, because of this, the interface is a bit cumbersome, in particular if we want to wait for a condition that could already hold now.
For this purpose we have introduced the `reactive` module with the `Watched` trait which has a `next` and `current` method. The `next` will return directly after initialization or a call to `reset`, which makes conditions checks easier.
This trait has been implemented for `watchable` data as well as mpsc channels. A trait `IntoWatched` is implemented to allow easy creation of `Watched`.

Using an extension trait we also have a nice method `next_satisfying` that takes a closure and only returns once the closure returns true. This is exactly the sort of condition-waiting that is commonly done for the state machine transitions.
The extension trait also defines `map` which is used for transparently converting data, e.g. into a readable format for logging.

## Telemetry

### Uplink

The function `lib_mission::telemetry::read_uplink` runs in a separate thread and whenever a transition command is received for the target `this_system`, and the state name corresponds to a valid state, then the input tree is updated with the command.
Similarly, manual actuator inputs are also stored in the input tree.

Another field in the input tree collects telemetry health data, that is the number and last timestamp of both the last received and last correctly processed message.

### Downlink

We downlink most measurements in the input and output tree at a configurable lower and upper interval, based on how often it actually changes.
For this a `Broadcast` macro exists which is placed on the input tree structure and can spawn tasks with given arguments for each field. The spawned tasked defers to a configurable loop implementation, which reads the changed values and forwards them such that a max frequency is respected, and also sends stale values at least at the a min frequency.

`lib_mission::telemetry::downlink_actor` provides logic to receive all the measurement streams and send the messages sequentially. This follows the actor pattern and is necessary because only a single thread can send at a time.

### Manual Transitions

The module `transitions` provides the implementation state logic to listen for manual transition commands.

The idea is that a transition command specifies a target state and optionally a precondition in terms of the index of the transition in the transitions path. If the preconition is specified, it is called a `validated` transitions and it will be used for planned mandatory transitions, to ensure consistency. If it isn't, these `unvalidated` transitions can be used to abort to an abort scenario, independently of whether some other transitions have triggered in the meantime.

In the config file, setting the `abort = true` flag causes an transitions to be allowed to be occur unvalidatedly.

### Manual Actuator Inputs

The command for a manual actuator input includes a human readable actuator ID (max 20 bytes), the path index of the state machine, and the target value as a float.
This is very generic and can be used for differnt actuators. For simple values, only 0.0 and 1.0 are valid and mean closed/open. For throttleable valves, 0.0-1.0 is valid. For dynmaic pressure regulators, NaN is used to disable, and any other value to set the setpoint.

Since only one command can be sent and the link is unreliable, we cannot change many things at once. This is no issue, as we only use this during testing in non-critical phases. Additionally, in the config file we have to enable specific actuators which can be controlled. Lastly, to make sure no command arrives which shouldn't, we allow only commands with the right path index, similarly to the precondition of manual transitions.


## CAN

We are communicating with most hardware using a CAN FD bus. The high level communication is defined in the [AVN-SFW Interface](/rocketry/teams/ASTERIA/systems_engineering/ICD/avn_sfw_icd) and the message format through the [asteria-data-definition](https://github.com/aris-space/asteria-data-definitions) repository.

On the flight computer we are using the `socketcan` crate which exposes the linux can implementation. This doesn't work on other platforms and we took care to allow building crate anyway, with minimal differences (no actual CAN implementation, but mockable).

The code also contains utilities for the user level, in particular the `Collector` trait and derive, that can be used to set fields of a struct based on a message in the form of an enum. This replaces input drivers.

On the other end, we use a thread which writes all messages it receives from drivers. They are independent and just need to send structured messages to the mpsc channel.
The `lib_mission::can` module documentation describes the API in more detail.

While for most things the flight computer is the single source of truth, during filling, instead the remote filling station computer takes over control of the valves in the rocket. Care has been taken that they do not infere.
Additionally, the RFS completely disables its CAN sending when the rocket is not eletrically connected to prevent error accumulation.

## Logging

### Timeseries

Timeseries of the input and output trees, as well as the current state, is done by writing a csv file for each measurement.

The module `lib_mission::util::log` provides the functions `spawn_watchable_logger` and `make_timeseries_writer` to start a task which logs every change in a uniform way.
You need to pass it a measurement id name and the headers.

The CSVs have a column for the mission time as a millisecond precision unix timestamp and the other columns are measurement dependent.
Every measurement has its own file.


### Tracing

Logging of the execution is done with the crate `tracing`. This crate is async aware and allows looking at interleaving operations.
The trace events, including span interactions, are logged to a JSON-lines file. Data from there can be extracted using the `jq` tool. Events are also logged to stdout in a more readable format and the set of shown events can be selected with the ˋRUST_LOGˋ environment variable. See [docs.rs on EnvFilter](https://docs.rs/tracing-subscriber/latest/tracing_subscriber/filter/struct.EnvFilter.html#directives) for its syntax.

In general, use `info` for non-periodic spans and events, `debug` for human readable periodic spans/events which help understanding the full system better and `trace` for internals and non-human readable spans/events, such as raw CAN frames.


### CAN

All valid frames will be logged to a tracing trace-level event with the decimal id and hex encoded frames. Additionally, the structured messages will be logged in debug representation to debug-level.

# Infrastructure

The flight computer runs on a raspberrry pi compute module 4 and is built using `cargo-zigbuild`. It is started in a tmux environment via ssh in order to persist after connection loss.
`fschk` has been validated to not cause any problems.

There is setup documentation along with the files in the repository to setup a new compute module running raspberry pi OS. It installs necessary services to run mavlink-router to multiplex the telemetry to wifi and RF, as well as services to blink status LEDs.
