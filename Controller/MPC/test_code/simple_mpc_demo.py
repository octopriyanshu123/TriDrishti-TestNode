"""
SIMPLIFIED 3D MPC EXAMPLE
This shows the core concepts without the full complexity
"""

import numpy as np
import matplotlib.pyplot as plt

# ============================================
# SIMPLE 3-AXIS MPC DEMONSTRATION
# ============================================

print("="*60)
print("SIMPLIFIED 3D MPC EXAMPLE")
print("="*60)

# Physical parameters
m = 1.0      # mass [kg]
g = 9.81     # gravity [m/s²]
dt = 0.1     # time step [s]

# MPC parameters
N = 5        # prediction horizon
Q = 100.0    # position error weight
R = 0.1      # control effort weight

# Current state
x, y, z = 0.0, 0.0, 0.0        # position [m]
vx, vy, vz = 0.0, 0.0, 0.0     # velocity [m/s]

# Target
target_x, target_y, target_z = 5.0, 3.0, 8.0

print(f"\nCurrent: ({x:.1f}, {y:.1f}, {z:.1f}) m")
print(f"Target:  ({target_x:.1f}, {target_y:.1f}, {target_z:.1f}) m")
print("\n" + "-"*60)

# ============================================
# Z-AXIS MPC (ALTITUDE)
# ============================================
print("\n1. Z-AXIS MPC (ALTITUDE CONTROL)")
print("-"*60)

# Try different thrust values
thrust_options = np.linspace(0, 20, 10)
best_thrust = m * g
best_cost = float('inf')

print(f"{'Thrust[N]':<12} {'Final Z[m]':<12} {'Final Vz[m/s]':<15} {'Cost':<10}")
print("-"*60)

for F in thrust_options:
    # Predict forward
    z_pred = z
    vz_pred = vz
    cost = 0
    
    for step in range(N):
        az = F/m - g
        vz_pred = vz_pred + az * dt
        z_pred = z_pred + vz_pred * dt
        
        # Cost = position error + velocity error + control effort
        cost += Q * (z_pred - target_z)**2 + Q * vz_pred**2 + R * F**2
    
    print(f"{F:<12.2f} {z_pred:<12.3f} {vz_pred:<15.3f} {cost:<10.2f}")
    
    if cost < best_cost:
        best_cost = cost
        best_thrust = F

print(f"\n✓ Best thrust: {best_thrust:.2f} N (Cost: {best_cost:.2f})")

# ============================================
# X-AXIS MPC (FORWARD/BACKWARD)
# ============================================
print("\n2. X-AXIS MPC (HORIZONTAL CONTROL)")
print("-"*60)

# For horizontal control, we tilt the drone
# Tilt angle creates horizontal acceleration: ax = g * tan(pitch)

pitch_options = np.linspace(-0.3, 0.3, 10)  # ±17 degrees
best_pitch = 0.0
best_cost = float('inf')

print(f"{'Pitch[rad]':<12} {'Pitch[deg]':<12} {'Final X[m]':<12} {'Final Vx[m/s]':<15} {'Cost':<10}")
print("-"*60)

for pitch in pitch_options:
    # Predict forward
    x_pred = x
    vx_pred = vx
    cost = 0
    
    for step in range(N):
        ax = g * np.tan(pitch)  # horizontal acceleration from tilt
        vx_pred = vx_pred + ax * dt
        x_pred = x_pred + vx_pred * dt
        
        cost += Q * (x_pred - target_x)**2 + Q * vx_pred**2 + R * pitch**2
    
    pitch_deg = np.degrees(pitch)
    print(f"{pitch:<12.3f} {pitch_deg:<12.1f} {x_pred:<12.3f} {vx_pred:<15.3f} {cost:<10.2f}")
    
    if cost < best_cost:
        best_cost = cost
        best_pitch = pitch

print(f"\n✓ Best pitch: {best_pitch:.3f} rad = {np.degrees(best_pitch):.1f}° (Cost: {best_cost:.2f})")

# ============================================
# Y-AXIS MPC (LEFT/RIGHT)
# ============================================
print("\n3. Y-AXIS MPC (LATERAL CONTROL)")
print("-"*60)

roll_options = np.linspace(-0.3, 0.3, 10)
best_roll = 0.0
best_cost = float('inf')

print(f"{'Roll[rad]':<12} {'Roll[deg]':<12} {'Final Y[m]':<12} {'Final Vy[m/s]':<15} {'Cost':<10}")
print("-"*60)

for roll in roll_options:
    y_pred = y
    vy_pred = vy
    cost = 0
    
    for step in range(N):
        ay = g * np.tan(roll)
        vy_pred = vy_pred + ay * dt
        y_pred = y_pred + vy_pred * dt
        
        cost += Q * (y_pred - target_y)**2 + Q * vy_pred**2 + R * roll**2
    
    roll_deg = np.degrees(roll)
    print(f"{roll:<12.3f} {roll_deg:<12.1f} {y_pred:<12.3f} {vy_pred:<15.3f} {cost:<10.2f}")
    
    if cost < best_cost:
        best_cost = cost
        best_roll = roll

print(f"\n✓ Best roll: {best_roll:.3f} rad = {np.degrees(best_roll):.1f}° (Cost: {best_cost:.2f})")

# ============================================
# MOTOR MIXING
# ============================================
print("\n4. CONVERT TO MOTOR SPEEDS")
print("-"*60)

# Quadcopter mixing (simplified)
base_thrust = best_thrust / 4.0  # distribute thrust to 4 motors

# Apply roll and pitch corrections
motor1 = base_thrust + best_pitch * 0.5  # front
motor2 = base_thrust + best_roll * 0.5   # right
motor3 = base_thrust - best_pitch * 0.5  # back
motor4 = base_thrust - best_roll * 0.5   # left

# Convert thrust to RPM (F = k * omega^2)
k_f = 1e-5
motor1_rpm = np.sqrt(max(0, motor1) / k_f)
motor2_rpm = np.sqrt(max(0, motor2) / k_f)
motor3_rpm = np.sqrt(max(0, motor3) / k_f)
motor4_rpm = np.sqrt(max(0, motor4) / k_f)

print(f"Total thrust: {best_thrust:.2f} N")
print(f"Roll angle:   {np.degrees(best_roll):.1f}°")
print(f"Pitch angle:  {np.degrees(best_pitch):.1f}°")
print(f"\nMotor speeds:")
print(f"  Motor 1 (Front): {motor1_rpm:>6.0f} RPM")
print(f"  Motor 2 (Right): {motor2_rpm:>6.0f} RPM")
print(f"  Motor 3 (Back):  {motor3_rpm:>6.0f} RPM")
print(f"  Motor 4 (Left):  {motor4_rpm:>6.0f} RPM")

# ============================================
# VISUALIZATION OF MPC PREDICTION
# ============================================
print("\n5. VISUALIZING MPC PREDICTIONS")
print("-"*60)

fig, axes = plt.subplots(2, 2, figsize=(12, 10))

# Z-axis prediction
ax1 = axes[0, 0]
z_traj = [z]
vz_traj = [vz]
z_sim = z
vz_sim = vz
for step in range(N):
    az = best_thrust/m - g
    vz_sim = vz_sim + az * dt
    z_sim = z_sim + vz_sim * dt
    z_traj.append(z_sim)
    vz_traj.append(vz_sim)

time_steps = np.arange(N+1) * dt
ax1.plot(time_steps, z_traj, 'b-o', linewidth=2, label='Predicted Z')
ax1.axhline(y=target_z, color='r', linestyle='--', label='Target Z')
ax1.set_xlabel('Time [s]')
ax1.set_ylabel('Z Position [m]')
ax1.set_title('Z-Axis MPC Prediction')
ax1.legend()
ax1.grid(True)

# X-axis prediction
ax2 = axes[0, 1]
x_traj = [x]
vx_traj = [vx]
x_sim = x
vx_sim = vx
for step in range(N):
    ax_val = g * np.tan(best_pitch)
    vx_sim = vx_sim + ax_val * dt
    x_sim = x_sim + vx_sim * dt
    x_traj.append(x_sim)
    vx_traj.append(vx_sim)

ax2.plot(time_steps, x_traj, 'g-o', linewidth=2, label='Predicted X')
ax2.axhline(y=target_x, color='r', linestyle='--', label='Target X')
ax2.set_xlabel('Time [s]')
ax2.set_ylabel('X Position [m]')
ax2.set_title('X-Axis MPC Prediction')
ax2.legend()
ax2.grid(True)

# Y-axis prediction
ax3 = axes[1, 0]
y_traj = [y]
vy_traj = [vy]
y_sim = y
vy_sim = vy
for step in range(N):
    ay_val = g * np.tan(best_roll)
    vy_sim = vy_sim + ay_val * dt
    y_sim = y_sim + vy_sim * dt
    y_traj.append(y_sim)
    vy_traj.append(vy_sim)

ax3.plot(time_steps, y_traj, 'm-o', linewidth=2, label='Predicted Y')
ax3.axhline(y=target_y, color='r', linestyle='--', label='Target Y')
ax3.set_xlabel('Time [s]')
ax3.set_ylabel('Y Position [m]')
ax3.set_title('Y-Axis MPC Prediction')
ax3.legend()
ax3.grid(True)

# Cost breakdown
ax4 = axes[1, 1]
costs = {
    'Position\nError': Q * ((z_traj[-1] - target_z)**2 + 
                            (x_traj[-1] - target_x)**2 + 
                            (y_traj[-1] - target_y)**2),
    'Velocity\nError': Q * (vz_traj[-1]**2 + vx_traj[-1]**2 + vy_traj[-1]**2),
    'Control\nEffort': R * (best_thrust**2 + best_pitch**2 + best_roll**2)
}
colors = ['#FF6B6B', '#4ECDC4', '#45B7D1']
ax4.bar(costs.keys(), costs.values(), color=colors, alpha=0.7)
ax4.set_ylabel('Cost')
ax4.set_title('MPC Cost Breakdown')
ax4.grid(True, axis='y')

plt.tight_layout()
plt.savefig('/home/claude/mpc_prediction_visualization.png', dpi=150, bbox_inches='tight')
print("✓ Visualization saved to: mpc_prediction_visualization.png")

# ============================================
# KEY CONCEPTS SUMMARY
# ============================================
print("\n" + "="*60)
print("KEY MPC CONCEPTS")
print("="*60)


print("""
1. PREDICTION HORIZON (N=5 steps)
   - MPC looks N steps into the future
   - Predicts where the drone will be
   - Chooses control that minimizes future cost

2. COST FUNCTION
   Cost = Q × (position_error)² + Q × (velocity)² + R × (control)²
   
   Q (weight=100): How much we care about reaching target
   R (weight=0.1):  How much we care about smooth control

3. SEPARATE AXIS CONTROL
   Z-axis: Thrust (vertical)
   X-axis: Pitch angle (forward/back)
   Y-axis: Roll angle (left/right)

4. RECEDING HORIZON
   - Compute optimal control
   - Apply ONLY the first step
   - Repeat at next time step
   - This makes it robust to disturbances

5. MOTOR MIXING
   Total thrust + Roll + Pitch → 4 motor speeds
   This is how the quadcopter actually moves
""")

print("="*60)
print("✓ Run the full simulation: python quadcopter_3d_mpc.py")
print("="*60)

plt.show()


'''
# Try different thrusts
for F in [10N, 15N, 20N, 25N]:
    # Predict where drone will be after N steps
    z_future = simulate(z_current, F, N_steps)
    
    # Calculate cost
    cost = (z_future - target)²  # How far from target?
    
# Pick the F with lowest cost
best_F = 20N  # (example)

# Apply it for ONE step only
# Then repeat everything next timestep
'''




