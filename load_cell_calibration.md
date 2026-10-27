# Load Cell — Tare and Calibration

Fresh calibration is required after every Teensy power cycle. Use a **200 g** reference mass.

## Commands

**Tare:** With the load cell unloaded, run:

```bash
ros2 topic pub --once /eagle11/load_cell/tare std_msgs/msg/Empty "{}"
```
or
```bash
load_cell_tare
```

**Calibrate:** Start with the load cell unloaded. After sending the command, keep it unloaded for the initial zero step, then attach the **200 g** mass when prompted in the node logs (the current ROS node uses a fixed 2 s zero delay and 5 s weight-placement delay).

```bash
ros2 topic pub --once /eagle11/load_cell/calibrate std_msgs/msg/Float64 "{data: 200.0}"
```
or
```bash
load_cell_calib
```


**Note:** In the current Teensy firmware, tare disables streaming until calibration is performed again. For a fresh power cycle, calibrate before taking measurements.

## Topics

| Topic | Type | Purpose |
|---|---|---|
| `/eagle11/load_cell/grams` | `std_msgs/msg/Float64` | Measured load in grams |
| `/eagle11/load_cell/tension` | `std_msgs/msg/Float64` | Tension in Newtons |
| `/eagle11/fluidpressure` | `sensor_msgs/msg/FluidPressure` | Legacy output (contains Newtons, not Pascals) |
| `/eagle11/load_cell/tare` | `std_msgs/msg/Empty` | Tare command |
| `/eagle11/load_cell/calibrate` | `std_msgs/msg/Float64` | Calibration command; mass in grams |
