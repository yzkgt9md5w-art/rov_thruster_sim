# ROV Thruster Firmware Simulator & ROS 2 Bridge

A ROS 2 (`rclpy`) motor control firmware simulator and telemetry bridge designed for a 6-thruster underwater Remotely Operated Vehicle (ROV). The package translates `geometry_msgs/Twist` velocity commands into vectored thruster RPM targets, models physical rotor inertia and hydrodynamic drag lag, simulates ESC PWM signals with deadband compensation, and streams noisy Hall-effect RPM telemetry.

## System Architecture

~~~text
  /cmd_vel (Twist)
         │
         ▼
┌──────────────────────────────────────────────────────────┐
│                 rov_thruster_sim_node                    │
│                                                          │
│  1. Vectored Mixing Matrix (4 Horizontal + 2 Vertical)   │
│  2. Command Watchdog Failsafe (0.5s Timeout -> 0 RPM)    │
│  3. 1st-Order Hydrodynamic & Rotor Lag Filter            │
│  4. ESC PWM Mapper (1100-1900 us + Deadband) & Noise     │
└──────────────┬───────────────────────────┬───────────────┘
               │                           │
               ▼                           ▼
 /thrusters/rpm_telemetry      /thrusters/pwm_debug
 (sensor_msgs/JointState)      (std_msgs/Int32MultiArray)
~~~

## Thruster Physics & Firmware Model

### 1. 6-DOF Thruster Allocation (Mixing)
The simulator models four $45^\circ$ vectored horizontal thrusters (`fl`, `fr`, `rl`, `rr`) for surge ($v_x$), sway ($v_y$), and yaw ($\omega_z$), plus two vertical thrusters (`vl`, `vr`) for heave ($v_z$). Outputs are normalized when combined commands exceed unity to preserve steering authority.

### 2. First-Order Motor & Water Drag Dynamics
At each firmware control tick ($\Delta t = 1 / f_{\text{sim}}$), actual rotor speed tracks the target RPM via a discrete low-pass filter governed by time constant $\tau$:

$$\text{RPM}_t = \text{RPM}_{t-1} + \alpha \left(\text{RPM}_{\text{target}} - \text{RPM}_{t-1}\right), \quad \alpha = \frac{\Delta t}{\tau + \Delta t}$$

### 3. Firmware Failsafe & ESC Mapping
* **Watchdog Timer:** If `/cmd_vel` updates cease for longer than `cmd_timeout` (`0.5 s`), target RPMs automatically zero out and thrusters coast down smoothly.
* **ESC PWM Translation:** Maps $[-\text{RPM}_{\max}, +\text{RPM}_{\max}]$ to standard $[1100, 1900]\,\mu\text{s}$ pulses with a neutral deadband at $1500\,\mu\text{s}$.

## Build & Run

~~~bash
cd ~/rov_ws
colcon build --packages-select rov_thruster_sim
source install/setup.bash
ros2 launch rov_thruster_sim sim_bridge.launch.py
~~~

## Testing & Verification

### Automated Unit Tests
~~~bash
cd ~/rov_ws
colcon test --packages-select rov_thruster_sim --pytest-args -k test_thruster_sim
colcon test-result --verbose
~~~

### Manual Telemetry Check
~~~bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.8, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
ros2 topic echo /thrusters/rpm_telemetry
~~~
