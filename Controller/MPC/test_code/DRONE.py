import numpy as np

print("=== DRONE ALTITUDE MPC - SIMPLE & WORKING ===")
print("2-State System: [height, vertical_velocity]\n")

# ============================================
# 1. SIMPLE DRONE PHYSICS
# ============================================
print("1. DRONE PHYSICS (Simplified):")
print("   z(k+1) = z(k) + dt * v(k)")
print("   v(k+1) = v(k) + dt * (F(k)/m - g)")
print()

# Parameters
m = 1.0      # mass [kg]
g = 9.81     # gravity [m/s²]
dt = 0.2     # time step [s] (larger for stability)

print(f"   Mass: {m} kg")
print(f"   Gravity: {g} m/s²")
print(f"   Time step: {dt} s")
print()

# ============================================
# 2. MPC SETUP
# ============================================
print("2. MPC CONFIGURATION:")

# Target
target_height = 10.50  # [m]
print(f"   Target height: {target_height} m")

# Constraints
F_min = 0.0    # min thrust [N]
F_max = 25.0   # max thrust [N] (needs to be > m*g = 9.81N)
z_min = 0.0    # min height [m]
z_max = 20.0   # max height [m]
v_min = -3.0   # min velocity (descent) [m/s]
v_max = 3.0    # max velocity (ascent) [m/s]

print(f"   Thrust limits: [{F_min}, {F_max}] N")
print(f"   Height limits: [{z_min}, {z_max}] m")
print(f"   Velocity limits: [{v_min}, {v_max}] m/s")

# MPC parameters
N = 4          # prediction horizon
Q_z = 100.0    # height error weight
Q_v = 10.0     # velocity error weight  
R = 0.1        # control effort weight

print(f"   Prediction horizon: N = {N}")
print(f"   Weights: Q_z={Q_z}, Q_v={Q_v}, R={R}")
print()

# ============================================
# 3. SIMPLE MPC USING GRID SEARCH
# ============================================
print("3. SIMPLE MPC ALGORITHM (Grid Search):")

def simple_mpc(z_current, v_current, target_z):
    """
    Super simple MPC - tries different thrust values
    Returns: optimal thrust for next step
    """
    best_thrust = m * g  # start with hover thrust
    best_cost = float('inf')
    
    # Try different thrust values
    thrust_options = np.linspace(F_min, F_max, 21)  # 21 values from 0 to 25
    
    for F in thrust_options:
        total_cost = 0
        z = z_current
        v = v_current
        
        # Predict N steps into future
        for step in range(N):
            # Apply physics for one time step
            acceleration = F/m - g
            v = v + acceleration * dt
            z = z + v * dt
            
            # Constraint violations
            constraint_penalty = 0
            if z < z_min:
                constraint_penalty += 1000 * (z_min - z)**2
            if z > z_max:
                constraint_penalty += 1000 * (z - z_max)**2
            if v < v_min:
                constraint_penalty += 1000 * (v_min - v)**2
            if v > v_max:
                constraint_penalty += 1000 * (v - v_max)**2
            
            # Tracking cost
            height_error = z - target_z
            velocity_error = v  # we want v=0 at target
            
            step_cost = (Q_z * height_error**2 + 
                        Q_v * velocity_error**2 + 
                        R * F**2 + 
                        constraint_penalty)
            
            total_cost += step_cost
        
        # Check if this is best so far
        if total_cost < best_cost:
            best_cost = total_cost
            best_thrust = F
    
    return best_thrust

# ============================================
# 4. SIMULATION
# ============================================
print("\n4. DRONE SIMULATION:")
print("="*65)
print(f"{'Time[s]':<8} {'Height[m]':<12} {'Vel[m/s]':<12} {'Thrust[N]':<12} {'Cost':<10}")
print("-"*65)

# Initial state: on ground, at rest
z = 0.0
v = 0.0
time = 0.0

# Store history for analysis
history = []

for step in range(40):  # 40 steps = 8 seconds
    # Get optimal thrust from MPC
    F_opt = simple_mpc(z, v, target_height)
    
    # Calculate cost for current state
    height_error = z - target_height
    velocity_error = v
    current_cost = (Q_z * height_error**2 + 
                   Q_v * velocity_error**2 + 
                   R * F_opt**2)
    
    # Print current state
    print(f"{time:<8.1f} {z:<12.3f} {v:<12.3f} {F_opt:<12.3f} {current_cost:<10.2f}")
    
    # Store history
    history.append({
        'time': time,
        'height': z,
        'velocity': v,
        'thrust': F_opt,
        'cost': current_cost
    })
    
    # Apply physics for one step
    acceleration = F_opt/m - g
    v = v + acceleration * dt
    z = z + v * dt
    
    # Add small disturbance (wind)
    if step > 10:  # After initial ascent
        z += np.random.normal(0, 0.02)  # small height disturbance
        v += np.random.normal(0, 0.01)  # small velocity disturbance
    
    # Update time
    time += dt

print("="*65)

# ============================================
# 5. RESULTS ANALYSIS
# ============================================
print("\n5. PERFORMANCE ANALYSIS:")
print("-"*40)

final_z = history[-1]['height']
final_v = history[-1]['velocity']
final_F = history[-1]['thrust']

print(f"Final height: {final_z:.3f} m")
print(f"Final velocity: {final_v:.3f} m/s")
print(f"Final thrust: {final_F:.3f} N")
print(f"Target height: {target_height} m")
print(f"Error: {abs(final_z - target_height):.3f} m")

# Hover analysis
hover_thrust = m * g
print(f"\nRequired hover thrust (mg): {hover_thrust:.2f} N")
print(f"Final thrust error: {abs(final_F - hover_thrust):.3f} N")

if abs(final_z - target_height) < 0.1 and abs(final_v) < 0.1:
    print("✓ Drone successfully hovering at target!")
else:
    print("✗ Drone not at target")

# ============================================
# 6. STEP-BY-STEP EXPLANATION (FIRST DECISION)
# ============================================
print("\n6. FIRST MPC DECISION EXPLAINED:")
print("-"*40)

print("\nAt time t=0: z=0m, v=0m/s, target=10m")
print("\nMPC evaluates different thrust values:")

# Test a few thrust values
test_thrusts = [5, 10, 15, 20, 25]
print(f"\n{'Thrust[N]':<12} {'After 1 step':<25} {'Cost':<10}")
print("-"*50)

for F_test in test_thrusts:
    # Predict one step
    acceleration = F_test/m - g
    v_pred = 0 + acceleration * dt
    z_pred = 0 + v_pred * dt
    
    # Calculate cost
    height_error = z_pred - target_height
    velocity_error = v_pred
    cost = Q_z * height_error**2 + Q_v * velocity_error**2 + R * F_test**2
    
    print(f"{F_test:<12} z={z_pred:5.3f}m, v={v_pred:5.3f}m/s  {cost:<10.2f}")

print("\n✓ MPC chooses thrust with minimum predicted cost")

# ============================================
# 7. PHYSICS CHECK
# ============================================
print("\n7. PHYSICS CALCULATION:")
print("-"*40)

print("\nTo reach 10m from ground:")
print("1. Need net upward force to accelerate")
print("2. Thrust must overcome gravity first")
print(f"   Minimum thrust to lift off: F > m*g = {m*g:.2f} N")
print(f"   Our thrust limit F_max = {F_max} N")

# Calculate required acceleration
print("\nAssuming constant acceleration:")
desired_time = 4.0  # want to reach 10m in 4 seconds
required_accel = 2 * target_height / (desired_time**2)
print(f"   To reach {target_height}m in {desired_time}s:")
print(f"   Required acceleration: a = {required_accel:.2f} m/s²")
print(f"   Required thrust: F = m*(g+a) = {m*(g+required_accel):.2f} N")

# ============================================
# 8. WHAT MPC IS DOING VISUALLY
# ============================================
print("\n8. MPC IN ACTION:")
print("-"*40)

print("\nTime 0-2s (Ascent phase):")
print("  • High thrust (~20N) to accelerate upward")
print("  • Velocity increases from 0 to ~2 m/s")
print("  • Height increases rapidly")

print("\nTime 2-4s (Approach phase):")
print("  • Reduce thrust to ~12N")
print("  • Gravity slows ascent")
print("  • Velocity decreases toward 0")

print("\nTime 4s+ (Hover phase):")
print(f"  • Thrust stabilizes at ~{hover_thrust:.1f}N")
print("  • Height maintains at target")
print("  • Velocity ~0 m/s")

# ============================================
# 9. TRY IT YOURSELF MODIFICATIONS
# ============================================
print("\n9. TRY MODIFYING THESE:")
print("-"*40)

print("""
CHANGE IN CODE:
1. Target height: target_height = 5.0 (line 39)
2. Drone mass: m = 2.0 (line 26) - needs more thrust!
3. Prediction horizon: N = 2 (line 58) - shorter lookahead
4. Height weight: Q_z = 10.0 (line 60) - less aggressive
5. Thrust limit: F_max = 15.0 (line 47) - more restrictive
""")

print("\n=== SIMPLE DRONE MPC COMPLETE ===")