# Adaptive Multi-Cell Battery Intelligence Engine

This is a Wokwi-ready ESP32 simulation for a production-style 4-cell lithium battery intelligence engine.

## What It Does

- Reads four simulated analog cell-voltage inputs.
- Converts each input into a lithium cell voltage range of 3.0 V to 4.2 V.
- Calculates pack voltage, average cell voltage, voltage spread, and imbalance percentage.
- Identifies the weakest and strongest cell in real time.
- Runs a fully non-blocking millis-based safety protection kernel.
- Detects weak cell voltage, overvoltage, sensor anomalies, and rapid voltage fluctuation.
- Uses relay anti-chatter timing and recovery hold logic before reconnecting the load.
- Provides a professional LCD HMI with automatic diagnostic page rotation.
- Uses cached LCD line updates for smoother, flicker-free rendering.
- Overrides normal HMI pages with fault-priority warning and cutoff screens.
- Sends event-driven cloud telemetry to Blynk only on state changes, anomalies, and threshold violations.
- Queues cloud events while offline and drains them after WiFi/Blynk reconnection.
- Monitors WiFi signal quality without blocking embedded safety functions.
- Publishes executive dashboard analytics including risk score, severity, diagnostics, fault history, and operator recommendations.
- Classifies pack health as:
  - Healthy
  - Minor Imbalance
  - Critical Imbalance
  - Pack Failure
- Displays live results on a 16x2 LCD.
- Prints a complete dashboard-style telemetry report to Serial Monitor.
- Uses green, yellow, and red LEDs plus a buzzer for health indication.

## Wokwi Setup

Create a new ESP32 project in Wokwi and add these files:

- `sketch.ino`
- `diagram.json`
- `libraries.txt`

Then start the simulation and open Serial Monitor at `115200` baud.

For cloud telemetry, replace these placeholders at the top of `sketch.ino` with your Blynk template/device credentials:

- `BLYNK_TEMPLATE_ID`
- `BLYNK_TEMPLATE_NAME`
- `BLYNK_AUTH_TOKEN`

## Simulated Hardware

- ESP32 DevKit
- 4 potentiometers representing Cell 1 to Cell 4
- 16x2 LCD
- Green LED: Healthy
- Yellow LED: Minor/Critical Imbalance
- Red LED: Pack Failure
- Buzzer: Critical/Failure alarm
- Relay module: automatic load cut-off during pack failure
- Cyan LED: protected load indicator through relay contacts

## ESP32 Pin Map

| Function | ESP32 Pin |
| --- | --- |
| Cell 1 analog input | GPIO 34 |
| Cell 2 analog input | GPIO 35 |
| Cell 3 analog input | GPIO 32 |
| Cell 4 analog input | GPIO 33 |
| LCD RS | GPIO 19 |
| LCD E | GPIO 18 |
| LCD D4 | GPIO 5 |
| LCD D5 | GPIO 17 |
| LCD D6 | GPIO 16 |
| LCD D7 | GPIO 4 |
| Healthy LED | GPIO 25 |
| Warning LED | GPIO 26 |
| Failure LED | GPIO 27 |
| Buzzer | GPIO 14 |
| Protection relay input | GPIO 13 |
| Relay feedback input | GPIO 15 |

## Relay Behavior

The relay acts as the battery protection output.

| Health State | Relay | Protected Load |
| --- | --- | --- |
| Healthy | ON | Connected |
| Minor Imbalance | ON | Connected |
| Critical Imbalance | ON | Connected with warning |
| Pack Failure | OFF | Disconnected |

## Event-Driven Safety Kernel

The firmware uses millis-based scheduling only. It does not use `delay()`.

| Protection Event | System Response |
| --- | --- |
| Weak cell or minor imbalance | Warning state, yellow LED blink, LCD warning, relay remains connected |
| Overvoltage | Safety cutoff, red LED blink, buzzer alarm, relay disconnects load |
| Sensor anomaly | Safety cutoff, red LED blink, buzzer alarm, relay disconnects load |
| Rapid voltage fluctuation | Safety cutoff, red LED blink, buzzer alarm, relay disconnects load |
| Critical imbalance | Safety cutoff, red LED blink, buzzer alarm, relay disconnects load |
| Stable recovery | Relay reconnects only after the pack remains safe for 5 seconds |

Anti-relay chatter protection prevents rapid relay switching by enforcing a 3-second minimum switching interval.

## Fault-Tolerant Runtime System

The runtime supervisor operates above the safety kernel and transitions between these modes:

| Runtime Mode | Meaning |
| --- | --- |
| NORMAL | All modules healthy, relay feedback valid |
| DEGRADED | One sensor module is isolated while healthy modules keep running |
| FAILSAFE | Safety cutoff/recovery is active and the relay is held open |
| SHUTDOWN | Relay mismatch or too many isolated sensor modules; relay is forced open |

Runtime faults are timestamped in a ring buffer and printed to Serial Monitor:

- Sensor disconnection
- Invalid readings
- Frozen ADC channel conditions
- Relay command/feedback mismatch

The relay feedback input on GPIO 15 reads the relay contact output, allowing the firmware to detect whether the relay state matches the commanded protection state.

## Blynk Cloud Telemetry

Telemetry is event-driven, not fixed-interval. The device sends data only when:

- Safety state changes
- Runtime mode changes
- Fault condition changes
- A voltage threshold is violated
- A runtime anomaly occurs
- Recovery completes
- WiFi signal quality becomes poor

Suggested Blynk virtual pins:

| Virtual Pin | Data |
| --- | --- |
| V0 | Pack voltage |
| V1 | Average cell voltage |
| V2 | Imbalance percentage |
| V3 | Safety state |
| V4 | Runtime mode |
| V5 | Fault |
| V6 | Relay state |
| V7 | WiFi RSSI |
| V8 | Cloud event type |
| V9 | Runtime fault |
| V10-V13 | Cell 1-4 voltages |
| V14 | Weakest cell number |
| V15 | Executive risk score |
| V16 | Risk severity |
| V17 | Operator recommendation |
| V18 | Fault log count |
| V19 | Isolated cell count |
| V20 | Latest fault-history summary |
| V21 | Load/protection status |

Create Blynk event codes:

- `battery_fault`
- `battery_recovered`

If WiFi or Blynk disconnects, the embedded system continues running locally and the telemetry queue synchronizes after reconnection.

## Embedded HMI Screens

The 16x2 LCD automatically rotates through diagnostic pages during safe operation:

| Screen | Information |
| --- | --- |
| Pack Overview | Pack voltage, average cell voltage, imbalance percentage |
| Cell Voltages | Live voltage for all four cells |
| Cell Analytics | Weakest and strongest cell |
| Protection Status | Safety state and relay status |
| Fault Diagnostics | Current fault condition |

During warning, cutoff, or recovery states, the HMI overrides normal rotation and displays the highest-priority fault screen immediately.

## Health Logic

| State | Rule |
| --- | --- |
| Healthy | Imbalance below 2% and all cells are within safe voltage range |
| Minor Imbalance | Imbalance from 2% to below 5% |
| Critical Imbalance | Imbalance at or above 5% |
| Pack Failure | Any cell below 3.0 V, any cell at or below 2.75 V, or any cell above 4.25 V |

## Testing Scenarios

1. Keep all four potentiometers close together.
   - Expected result: `Healthy`
2. Move one cell slightly lower than the others.
   - Expected result: `Minor Imbalance`
3. Move one cell far away from the others.
   - Expected result: `Critical Imbalance`
4. Move one cell to the minimum.
   - Expected result: `Pack Failure`

## Submission Notes

This folder is the unified ESP32 Wokwi project for the integrated battery intelligence system. It combines battery analytics, safety protection, HMI diagnostics, runtime fault tolerance, cloud telemetry, and executive dashboard telemetry in one project.
