# Car vitals for the UI, published by f1tenth_tools/telemetry_aggregator
std_msgs/Header header

# Motion
float32 speed               # m/s, from the VESC electrical RPM
float32 speed_command       # m/s, final command sent to the VESC
float32 steering_angle      # rad, from the servo position the VESC applied
float32 steering_command    # rad, final command sent to the VESC
float32 erpm
float32 duty_cycle          # -1..1

# Power
bool vesc_connected
float32 battery_voltage     # V
float32 battery_percent     # 0..100, estimated from the cell voltage
float32 motor_current       # A
float32 input_current       # A
float32 brake_current       # A, size of the latest brake command (0 when released)
float32 temp_fet            # degC
float32 temp_motor          # degC
float32 energy_drawn        # Wh since the VESC started
float32 distance            # m since the VESC started
int32 fault_code
string fault

# Driver inputs (joystick)
bool joystick_connected
bool deadman                # teleop deadman (LB) held
float32 throttle            # throttle stick, -1..1
float32 steer_stick         # steering stick, -1..1
float32 brake_trigger       # brake trigger (RT), 0..1
float32 boost               # speed boost trigger (LT), 0..1

# Locks
bool teleop_lock            # kill switch (RB) engaged: nothing can drive
bool autonomous_lock        # autonomous commands blocked (RT brake or safety brake)
bool estop                  # UI emergency stop engaged (mux_bool/ui_estop): nothing can drive
