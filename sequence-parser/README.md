# Sequences

## System Functions
### sys.msg
`sys.msg(text)`
Sends a message to the console.

**Parameters**
- **text:** `string` *Required.* Content of the message. This text will be displayed in the console, together with the time the message was sent.

### sys.warning
`sys.warning(text)`
Sends a message to the console. The message will be displayed in yellow.

**Parameters**
- **text:** `string` *Required.* Content of the message. This text will be displayed in the console, together with the time the message was sent.

### sys.error
`sys.error(text)`
Sends a message to the console. The message will be displayed in red.

**Parameters**
- **text:** `string` *Required.* Content of the message. This text will be displayed in the console, together with the time the message was sent.

### sys.wait
`sys.wait(time)`
Pauses the sequence execution for the specified timespan. This function works asynchronously, such that all aborts can be triggered while the sequence is paused.

**Parameters**
- **time:** `time` *Required.* Waiting time.

### sys.to_state
`sys.to_state(state)`
Switches the statemachine to another state.

**Parameters**
- **state:** `string` *Required.* State. Specifies the state that the system should switch into.

## Actuator Functions
### actuator.open
`Actuator.open()`
Opens a non-DPR actuator. Used to open a valve or activate the spark plug. The code automatically figures out, if the valve is normally closed or normally open.

### actuator.close
`Actuator.close()`
Closes a non-DPR acutator. Used to close a valve or deactivate the spark plug. The code automatically figures out, if the valve is normally closed or normally open.

### dpr.activate
`DprActuator.activate(setpoint)`
Activates a DPR actuator. This sets the DPR setpoint to `setpoint` and opens the valve.

**Parameters**
- **setpoint:** `float` *Required*. Setpoint for the DPR valve.

### dpr.deactivate
`DprActuator.deactivate()`
Deactivates a DPR actuator. This closes the valve by setting the setpoint to `NAN`.

## Sensor Functions

### sensor.watch
`Sensor.watch(abort, max=INF, min=INF, key=None)`
Adds an abort threshold to a sensor. The function will asynchronously run in the background and trigger the specified abort if the sensor value exeeds one of the speciefied threasholds.

**Parameters**
- **abort:** `string` *Required.* Abort state. Specifies the name of the state that we transition to if one of the sensor thresholds is crossed.
- **max:** `float` *Optional. Default = `INF`* Upper threshold. If the sensor value rises above `max`, the abort is triggered. If `max` is `INF` no upper threshold is monitored.
- **min:** `float` *Optional. Default = `INF`* Lower threshold. If the sensor value falls below `min`, the abort is triggered. If `min` is `INF` no lower threshold is monitored.
- **key:** `string` *Optional. Default = `None`* Key to identify monitoring function. Needed if multiple functions are defined on the same sensor. 

### sensor.ignore
`Sensor.ignore(key=None)`
Removes the `Sensor.watch()` function.

**Parameters**
- **key:** `string` *Optional. Default = `None`* Key to identify monitoring function to remove. If `key` is `None`, all monitors will be removed from the sensor.

### sensor.detect
`Sensor.detect(value, rising=true, abort, patience, duration=None)`

Halts sequence execution until a specific sensor value is reached. If the value is not reached in the specified `patience` time, `abort` is triggered.

**Parameters**
- **value:** `float` *Required.* Value to be reached. If the sensor value does not cross this threshold, an **abort** is triggered.
- **rising:** `bool` *Optional. Default = `true`* Specifies if the value is expected to be above or below the threshold. If **rising** is `true`, sequence execution continues as soon as the sensor rises above **value**.
- **abort:** `string` *Required.* Abort state. Specifies the name of the state that we transition to if the time runs out.
- **patience:** `time` *Required.* Timer. If the sensor does not cross the threshold in the specified time, an **abort** is triggered.
- **duration:** `time` *Optional. Default = `None`* Timer. The sensor has to stay above **value** for the specified time, otherwhise **abort** is triggered.