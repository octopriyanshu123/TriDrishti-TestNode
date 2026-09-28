import numpy as np
import time

# ============================================
# 1. DRONE PARAMETERS
# ============================================
class DroneParams:
    def __init__(self):
        self.m = 1.0          # mass [kg]
        self.g = 9.81         # gravity [m/s²]
        self.dt = 0.1         # time step [s]
        
        # Motor parameters
        self.k_f = 1e-5       # thrust coefficient (F = k_f * omega^2)
        self.motor_min = 0    # min RPM
        self.motor_max = 10000 # max RPM
        
        # MPC parameters
        self.N = 10           # prediction horizon
        self.Q_z = 100.0      # position error weight
        self.Q_v = 10.0       # velocity error weight
        self.R = 0.1          # control effort weight
        
        # Position threshold to reach target
        self.position_threshold = 0.1  # [m] - target reached if within this
        self.velocity_threshold = 0.01  # [m/s] - and velocity is low

# ============================================
# 2. MPC CONTROLLER
# ============================================
class MPC_Z_Controller:
    def __init__(self, params):
        self.params = params
    
    def compute_thrust(self, z_current, vz_current, z_target):
        """
        MPC for Z-axis (altitude) control
        Returns: optimal thrust [N]
        """
        best_thrust = self.params.m * self.params.g
        best_cost = float('inf')
        
        # Try different thrust values
        F_min = 0.0
        F_max = 2.5 * self.params.m * self.params.g
        n_samples = 20
        thrust_options = np.linspace(F_min, F_max, n_samples)
        
        for F in thrust_options:
            total_cost = 0
            z = z_current
            vz = vz_current
            
            # Predict N steps into future
            for step in range(self.params.N):
                # Physics: az = F/m - g
                az = F / self.params.m - self.params.g
                vz = vz + az * self.params.dt
                z = z + vz * self.params.dt
                
                # Cost function
                z_error = z - z_target
                step_cost = (self.params.Q_z * z_error**2 + 
                            self.params.Q_v * vz**2 + 
                            self.params.R * (F - self.params.m * self.params.g)**2)
                
                # Constraint penalties
                if z < 0:  # Don't go below ground
                    step_cost += 10000 * (0 - z)**2
                if vz < -5.0 or vz > 5.0:  # Velocity limits
                    step_cost += 1000
                
                total_cost += step_cost
            
            # Select best thrust
            if total_cost < best_cost:
                best_cost = total_cost
                best_thrust = F
        
        return best_thrust

# ============================================
# 3. DRONE SIMULATION
# ============================================
class Drone:
    def __init__(self, params):
        self.params = params
        self.z = 0.0       # position [m]
        self.vz = 0.0      # velocity [m/s]
        self.motor_speeds = np.array([0.0, 0.0, 0.0, 0.0])  # 4 motors [RPM]
    
    def thrust_to_motors(self, total_thrust):
        """
        Convert total thrust to 4 motor speeds
        For hovering: all motors same speed
        """
        # Distribute thrust equally to 4 motors
        thrust_per_motor = total_thrust / 4.0
        
        # Convert thrust to RPM: F = k_f * omega^2
        # omega = sqrt(F / k_f)
        rpm_per_motor = np.sqrt(max(0, thrust_per_motor) / self.params.k_f)
        
        # All motors same for pure vertical flight
        self.motor_speeds = np.array([rpm_per_motor] * 4)
        
        # Clamp to limits
        self.motor_speeds = np.clip(self.motor_speeds, 
                                    self.params.motor_min, 
                                    self.params.motor_max)
        
        return self.motor_speeds
    
    def update(self, thrust):
        """Update drone state based on thrust"""
        # Physics
        az = thrust / self.params.m - self.params.g
        self.vz = self.vz + az * self.params.dt
        self.z = self.z + self.vz * self.params.dt
        
        # Ensure drone doesn't go below ground
        if self.z < 0:
            self.z = 0
            self.vz = 0
    
    def at_target(self, target_z):
        """Check if drone reached target"""
        position_ok = abs(self.z - target_z) < self.params.position_threshold
        velocity_ok = abs(self.vz) < self.params.velocity_threshold
        return position_ok and velocity_ok

# ============================================
# 4. MAIN SIMULATION
# ============================================
def main():
    print("\n" + "="*70)
    print(" Z-AXIS MPC DRONE CONTROLLER")
    print("="*70)
    
    # Initialize
    params = DroneParams()
    drone = Drone(params)
    controller = MPC_Z_Controller(params)
    
    # Waypoints: z=0 → z=10 → z=25 → z=0
    waypoints = [0, 10, 25, 0]
    current_waypoint_idx = 0
    target_z = waypoints[current_waypoint_idx]
    
    # Skip first waypoint (already at z=0)
    current_waypoint_idx = 1
    target_z = waypoints[current_waypoint_idx]
    
    print(f"\nStarting position: z = {drone.z:.2f} m")
    print(f"Waypoints: {waypoints}")
    print(f"\nMoving to waypoint {current_waypoint_idx}: z = {target_z} m")
    print("\n" + "-"*70)
    print(f"{'Time[s]':<8} {'Z[m]':<10} {'Vz[m/s]':<10} {'Target[m]':<10} "
          f"{'M1[RPM]':<10} {'M2[RPM]':<10} {'M3[RPM]':<10} {'M4[RPM]':<10}")
    print("-"*70)
    
    # Simulation loop
    sim_time = 0.0
    max_time = 100.0  # safety timeout
    print_interval = 0.5  # print every 0.5 seconds
    last_print = 0.0
    
    while sim_time < max_time:
        # Check if reached current target
        if drone.at_target(target_z):
            print(f"\n✓ Reached waypoint {current_waypoint_idx}: z = {target_z:.2f} m "
                  f"(actual: {drone.z:.3f} m, vz: {drone.vz:.3f} m/s)")
            
            # Move to next waypoint
            current_waypoint_idx += 1
            
            if current_waypoint_idx >= len(waypoints):
                print(f"\n{'='*70}")
                print("✓ ALL WAYPOINTS COMPLETED!")
                print(f"{'='*70}")
                print(f"Final position: z = {drone.z:.3f} m")
                print(f"Final velocity: vz = {drone.vz:.3f} m/s")
                print(f"Total time: {sim_time:.1f} seconds")
                break
            
            target_z = waypoints[current_waypoint_idx]
            print(f"\nMoving to waypoint {current_waypoint_idx}: z = {target_z} m")
            print("-"*70)
            print(f"{'Time[s]':<8} {'Z[m]':<10} {'Vz[m/s]':<10} {'Target[m]':<10} "
                  f"{'M1[RPM]':<10} {'M2[RPM]':<10} {'M3[RPM]':<10} {'M4[RPM]':<10}")
            print("-"*70)
        
        # Compute optimal thrust using MPC
        optimal_thrust = controller.compute_thrust(drone.z, drone.vz, target_z)
        
        # Convert thrust to motor speeds
        motor_speeds = drone.thrust_to_motors(optimal_thrust)
        
        # Update drone dynamics
        drone.update(optimal_thrust)
        
        # Print status at intervals
        if sim_time - last_print >= print_interval:
            print(f"{sim_time:<8.1f} {drone.z:<10.3f} {drone.vz:<10.3f} {target_z:<10.1f} "
                  f"{motor_speeds[0]:<10.0f} {motor_speeds[1]:<10.0f} "
                  f"{motor_speeds[2]:<10.0f} {motor_speeds[3]:<10.0f}")
            last_print = sim_time
        
        # Increment time
        sim_time += params.dt
        
        # Real-time delay (optional - comment out for fast simulation)
        # time.sleep(params.dt)
    
    if sim_time >= max_time:
        print("\n⚠ Maximum simulation time reached!")
    
    print("\n" + "="*70)
    print("SIMULATION COMPLETE")
    print("="*70)
    
    # Summary
    print("\nFlight Summary:")
    print(f"  Total waypoints: {len(waypoints)}")
    print(f"  Waypoints reached: {current_waypoint_idx}")
    print(f"  Total flight time: {sim_time:.1f} seconds")
    print(f"  Final altitude: {drone.z:.3f} m")
    print(f"  Final velocity: {drone.vz:.3f} m/s")
    
    print("\nMotor speeds at end:")
    print(f"  Motor 1: {drone.motor_speeds[0]:.0f} RPM")
    print(f"  Motor 2: {drone.motor_speeds[1]:.0f} RPM")
    print(f"  Motor 3: {drone.motor_speeds[2]:.0f} RPM")
    print(f"  Motor 4: {drone.motor_speeds[3]:.0f} RPM")
    
    print("\n" + "="*70)


if __name__ == "__main__":
    main()
