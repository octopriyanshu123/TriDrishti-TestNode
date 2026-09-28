# MPC MOTOR CONTROLLER FOR DWA LOCAL PLANNER

## 🎯 System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    NAVIGATION STACK                              │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Global Planner  →  DWA Local Planner  →  /cmd_vel             │
│                     (velocity commands)    [v, ω]               │
│                                                ↓                 │
│                                    ┌───────────────────┐        │
│                                    │  MPC MOTOR        │        │
│                                    │  CONTROLLER       │        │
│                                    └───────────────────┘        │
│                                                ↓                 │
│                              ┌─────────────────────────┐        │
│                              │  Motor Commands         │        │
│                              │  [V_left, V_right]      │        │
│                              └─────────────────────────┘        │
│                                                ↓                 │
│                              ┌─────────────────────────┐        │
│                              │   ROBOT MOTORS          │        │
│                              │   (Hardware)            │        │
│                              └─────────────────────────┘        │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

## 📊 What MPC Does

### Traditional Approach (No MPC):
```
DWA → cmd_vel → Direct motor control → Motors
```
**Problem**: No consideration for motor dynamics, delays, or constraints!

### With MPC:
```
DWA → cmd_vel → MPC (predicts motor response) → Optimal voltages → Motors
```
**Benefit**: Smooth tracking, respects motor limits, handles dynamics!

---

## 🔧 How It Works

### Step 1: DWA Publishes cmd_vel
```
/cmd_vel:
  linear.x: 0.5    # Forward velocity [m/s]
  angular.z: 0.3   # Turn rate [rad/s]
```

### Step 2: Convert to Wheel Speeds
```python
# Differential drive kinematics:
ω_L = (v - ω*L/2) / R
ω_R = (v + ω*L/2) / R

Where:
  R = wheel radius
  L = wheelbase (distance between wheels)
```

### Step 3: MPC Predicts & Optimizes
```python
# MPC looks ahead N steps:
for each voltage_pair in [V_left, V_right]:
    # Predict motor response
    predicted_speeds = simulate_motor_dynamics(current, voltage, N_steps)
    
    # Calculate cost
    cost = Q*(predicted - target)² + R*voltage²
    
# Select voltage with minimum cost
best_voltage = min_cost_voltage
```

### Step 4: Apply to Motors
```
/motor_commands: [V_left, V_right]
```

---

## 📦 Installation & Setup

### 1. Copy the File
```bash
cp mpc_motor_controller_ros.py ~/catkin_ws/src/your_package/scripts/
chmod +x ~/catkin_ws/src/your_package/scripts/mpc_motor_controller_ros.py
```

### 2. Create Launch File
```xml
<!-- mpc_motor_control.launch -->
<launch>
  <node name="mpc_motor_controller" 
        pkg="your_package" 
        type="mpc_motor_controller_ros.py" 
        output="screen">
    
    <!-- Robot parameters -->
    <param name="wheel_radius" value="0.05"/>     <!-- 5cm wheels -->
    <param name="wheelbase" value="0.3"/>         <!-- 30cm between wheels -->
    <param name="motor_constant" value="10.0"/>   <!-- Motor K_v -->
    <param name="motor_tau" value="0.1"/>         <!-- Motor time constant -->
    <param name="max_voltage" value="12.0"/>      <!-- 12V -->
    <param name="max_rpm" value="300"/>           <!-- 300 RPM -->
    
    <!-- MPC parameters -->
    <param name="mpc_dt" value="0.05"/>           <!-- 50ms control loop -->
    <param name="mpc_horizon" value="10"/>        <!-- 10 steps = 0.5s -->
    <param name="mpc_Q" value="100.0"/>           <!-- Tracking weight -->
    <param name="mpc_R" value="0.1"/>             <!-- Control smoothness -->
    
  </node>
</launch>
```

### 3. Run It
```bash
# Start your navigation stack
roslaunch your_package navigation.launch

# Start MPC motor controller
roslaunch your_package mpc_motor_control.launch
```

---

## ⚙️ Configuration

### Robot Parameters (CRITICAL!)

#### Measure Your Robot:
1. **wheel_radius**: Measure wheel diameter, divide by 2
   ```
   wheel_radius = diameter / 2
   ```

2. **wheelbase**: Distance between left and right wheel centers
   ```
   wheelbase = distance_between_wheels
   ```

3. **motor_constant (K_v)**: Motor speed per volt
   ```
   K_v ≈ (max_rpm * 2π/60) / max_voltage
   
   Example: 300 RPM at 12V
   K_v = (300 * 2π/60) / 12 = 2.62 rad/s/V
   ```

4. **motor_tau**: Motor time constant (response speed)
   ```
   Typical: 0.05 - 0.2 seconds
   Fast motors: ~0.05s
   Slow motors: ~0.2s
   ```

### MPC Parameters

#### Tuning Guide:

| Parameter | Effect | Typical Range | Start With |
|-----------|--------|---------------|------------|
| `mpc_dt` | Control frequency | 0.02 - 0.1s | 0.05s |
| `mpc_horizon` | Lookahead steps | 5 - 20 | 10 |
| `mpc_Q` | Tracking aggression | 10 - 1000 | 100 |
| `mpc_R` | Control smoothness | 0.01 - 10 | 0.1 |

#### Tuning Tips:

**Want faster response?**
- ↑ Increase `mpc_Q` (e.g., 200)
- ↓ Decrease `mpc_R` (e.g., 0.01)
- ↑ Increase `mpc_horizon` (e.g., 15)

**Want smoother motion?**
- ↓ Decrease `mpc_Q` (e.g., 50)
- ↑ Increase `mpc_R` (e.g., 1.0)

**Having oscillations?**
- ↑ Increase `mpc_R`
- Check if `motor_tau` is correct

**Slow to reach target?**
- ↑ Increase `mpc_Q`
- ↓ Decrease `motor_tau` if motors are actually faster

---

## 🔌 ROS Topics

### Subscribed Topics:
- `/cmd_vel` (geometry_msgs/Twist) - From DWA planner

### Published Topics:
- `/motor_commands` (std_msgs/Float64MultiArray) - Motor voltages [V_left, V_right]
- `/mpc_diagnostics` (std_msgs/Float64MultiArray) - Debug info

### Diagnostics Data:
```
Index 0-1: [target_v, actual_v]           # Linear velocity
Index 2-3: [target_omega, actual_omega]   # Angular velocity
Index 4-5: [omega_left, omega_right]      # Wheel speeds
Index 6-7: [V_left, V_right]              # Motor voltages
```

---

## 🔍 Monitoring & Debugging

### View Diagnostics:
```bash
rostopic echo /mpc_diagnostics
```

### Plot in rqt:
```bash
rqt_plot /mpc_diagnostics/data[0]:data[1]  # Velocity tracking
rqt_plot /mpc_diagnostics/data[6]:data[7]  # Motor voltages
```

### Check Motor Commands:
```bash
rostopic echo /motor_commands
```

---

## 🧪 Testing Without ROS

### Standalone Test:
```bash
python mpc_motor_controller_ros.py --test
```

This runs 4 test scenarios:
1. **Forward** - Straight line motion
2. **Turn left** - Combined linear + angular
3. **Spin** - Pure rotation
4. **Stop** - Deceleration

---

## 🎮 Integration with Your Hardware

### Option 1: Arduino/Microcontroller

Create a subscriber node that converts voltages to PWM:

```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
import serial

class MotorDriver:
    def __init__(self):
        self.serial = serial.Serial('/dev/ttyUSB0', 115200)
        rospy.Subscriber('/motor_commands', Float64MultiArray, self.callback)
    
    def callback(self, msg):
        V_left = msg.data[0]
        V_right = msg.data[1]
        
        # Convert voltage to PWM (0-255)
        PWM_left = int((V_left / 12.0) * 255)
        PWM_right = int((V_right / 12.0) * 255)
        
        # Send to Arduino
        command = f"{PWM_left},{PWM_right}\n"
        self.serial.write(command.encode())
```

### Option 2: Motor Controller ROS Package

If using `ros_control` or similar:

```python
# Publish to your motor controller's topics
self.left_motor_pub = rospy.Publisher('/left_motor/command', Float64, queue_size=1)
self.right_motor_pub = rospy.Publisher('/right_motor/command', Float64, queue_size=1)

# In callback:
self.left_motor_pub.publish(V_left)
self.right_motor_pub.publish(V_right)
```

---

## 📈 Performance Comparison

### Without MPC:
```
cmd_vel → motor_driver
  ✗ Ignores motor dynamics
  ✗ No prediction
  ✗ Can violate motor limits
  ✗ Jerky motion
```

### With MPC:
```
cmd_vel → MPC → motor_driver
  ✓ Considers motor dynamics
  ✓ Predicts future states
  ✓ Respects motor limits
  ✓ Smooth trajectories
```

---

## 🐛 Troubleshooting

### Problem: Motors oscillating
**Solution**: 
- Increase `mpc_R` (more smoothing)
- Check `motor_tau` is correct
- Reduce `mpc_Q`

### Problem: Slow to respond
**Solution**:
- Increase `mpc_Q`
- Decrease `mpc_R`
- Check `motor_constant` is correct

### Problem: Not reaching target velocity
**Solution**:
- Verify `max_voltage` is correct
- Check `wheel_radius` and `wheelbase` measurements
- Ensure motors can actually reach target speed

### Problem: Motors saturating
**Solution**:
- Check if DWA is commanding realistic velocities
- Verify `max_rpm` parameter
- Consider increasing `max_voltage` if safe

---

## 💡 Advanced: Add Encoder Feedback

Currently the code simulates motor state. For real robots, read encoders:

```python
def encoder_callback(self, msg):
    """Get actual wheel speeds from encoders"""
    self.current_motor_speeds[0] = msg.left_wheel_speed
    self.current_motor_speeds[1] = msg.right_wheel_speed
```

Subscribe to encoder topic:
```python
self.encoder_sub = rospy.Subscriber(
    '/wheel_encoders',
    YourEncoderMsg,
    self.encoder_callback
)
```

---

## 📚 Key Concepts

### Why MPC for Motors?

1. **Motor Dynamics**: Motors don't respond instantly to voltage
   ```
   dω/dt = (V*K_v - ω) / τ
   ```

2. **Prediction**: MPC looks ahead to see motor response

3. **Constraints**: Automatically respects voltage and speed limits

4. **Optimization**: Finds best voltage to track cmd_vel

### Differential Drive Kinematics

```
v = R/2 * (ω_L + ω_R)      # Linear velocity
ω = R/L * (ω_R - ω_L)      # Angular velocity

Inverse:
ω_L = (v - ω*L/2) / R
ω_R = (v + ω*L/2) / R
```

---

## 🎯 Quick Start Checklist

- [ ] Measure robot: wheel_radius, wheelbase
- [ ] Determine motor specs: max_voltage, max_rpm, K_v, tau
- [ ] Copy script to ROS package
- [ ] Create launch file with parameters
- [ ] Test with `--test` flag first
- [ ] Launch with navigation stack
- [ ] Monitor `/mpc_diagnostics`
- [ ] Tune Q and R for your application
- [ ] Add encoder feedback (if available)
- [ ] Connect to actual motor driver

---

## 📊 Example Parameter Sets

### Small Indoor Robot (TurtleBot-like):
```xml
<param name="wheel_radius" value="0.033"/>    <!-- 33mm -->
<param name="wheelbase" value="0.23"/>        <!-- 230mm -->
<param name="motor_constant" value="15.0"/>
<param name="motor_tau" value="0.05"/>
<param name="max_voltage" value="12.0"/>
<param name="max_rpm" value="400"/>
<param name="mpc_Q" value="150.0"/>
<param name="mpc_R" value="0.05"/>
```

### Larger Outdoor Robot:
```xml
<param name="wheel_radius" value="0.1"/>      <!-- 100mm -->
<param name="wheelbase" value="0.5"/>         <!-- 500mm -->
<param name="motor_constant" value="8.0"/>
<param name="motor_tau" value="0.15"/>
<param name="max_voltage" value="24.0"/>
<param name="max_rpm" value="200"/>
<param name="mpc_Q" value="80.0"/>
<param name="mpc_R" value="0.2"/>
```

---

**Your robot is now ready for smooth, optimal motor control! 🚀**
