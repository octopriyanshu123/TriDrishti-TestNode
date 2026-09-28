import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import threading
import queue
import time

# ============================================
# 1. QUADCOPTER PARAMETERS
# ============================================
class QuadcopterParams:
    def __init__(self):
        # Physical parameters
        self.m = 1.0          # mass [kg]
        self.g = 9.81         # gravity [m/s²]
        self.L = 0.25         # arm length [m]
        self.k_f = 1e-5       # thrust coefficient
        self.k_m = 1e-7       # moment coefficient
        
        # Inertia matrix (simplified)
        self.Ixx = 0.01       # [kg⋅m²]
        self.Iyy = 0.01       # [kg⋅m²]
        self.Izz = 0.02       # [kg⋅m²]
        
        # Time step
        self.dt = 0.05        # [s] - 20Hz control loop
        
        # Motor constraints (RPM)
        self.motor_min = 0
        self.motor_max = 10000  # Max RPM
        
        # State constraints
        self.pos_min = -20.0  # [m]
        self.pos_max = 20.0   # [m]
        self.vel_min = -5.0   # [m/s]
        self.vel_max = 5.0    # [m/s]

# ============================================
# 2. MPC CONTROLLER
# ============================================
class MPC_3D_Controller:
    def __init__(self, params):
        self.params = params
        
        # MPC parameters
        self.N = 10           # prediction horizon
        self.Q_pos = 100.0    # position error weight
        self.Q_vel = 10.0     # velocity error weight
        self.R = 0.01         # control effort weight
        
        # Number of thrust samples to test
        self.n_samples = 15
        
    def compute_control(self, state, target):
        """
        State: [x, y, z, vx, vy, vz]
        Target: [x_target, y_target, z_target]
        Returns: [thrust_total, roll, pitch, yaw_rate]
        """
        x, y, z, vx, vy, vz = state
        x_t, y_t, z_t = target
        
        # For simplified 3D control, we separate:
        # 1. Z-axis (altitude) control -> Total thrust
        # 2. X-axis control -> Pitch
        # 3. Y-axis control -> Roll
        
        # Z-axis MPC (altitude)
        F_z = self._mpc_altitude(z, vz, z_t)
        
        # X-axis MPC (forward/backward)
        pitch = self._mpc_horizontal(x, vx, x_t, 'x')
        
        # Y-axis MPC (left/right)
        roll = self._mpc_horizontal(y, vy, y_t, 'y')
        
        # Yaw rate (simplified - keep at 0)
        yaw_rate = 0.0
        
        return F_z, roll, pitch, yaw_rate
    
    def _mpc_altitude(self, z_current, vz_current, z_target):
        """MPC for altitude control"""
        best_thrust = self.params.m * self.params.g
        best_cost = float('inf')
        
        # Test different thrust values
        F_min = 0.0
        F_max = 2.0 * self.params.m * self.params.g
        thrust_options = np.linspace(F_min, F_max, self.n_samples)
        
        for F in thrust_options:
            total_cost = 0
            z = z_current
            vz = vz_current
            
            # Predict N steps
            for step in range(self.N):
                # Physics
                az = F / self.params.m - self.params.g
                vz = vz + az * self.params.dt
                z = z + vz * self.params.dt
                
                # Cost
                z_error = z - z_target
                step_cost = (self.Q_pos * z_error**2 + 
                            self.Q_vel * vz**2 + 
                            self.R * (F - self.params.m * self.params.g)**2)
                
                # Constraint penalties
                if z < self.params.pos_min or z > self.params.pos_max:
                    step_cost += 10000
                if vz < self.params.vel_min or vz > self.params.vel_max:
                    step_cost += 10000
                
                total_cost += step_cost
            
            if total_cost < best_cost:
                best_cost = total_cost
                best_thrust = F
        
        return best_thrust
    
    def _mpc_horizontal(self, pos_current, vel_current, pos_target, axis):
        """MPC for horizontal position control (returns desired angle)"""
        best_angle = 0.0
        best_cost = float('inf')
        
        # Test different tilt angles (-20° to +20°)
        angle_options = np.linspace(-0.35, 0.35, self.n_samples)  # radians
        
        for angle in angle_options:
            total_cost = 0
            pos = pos_current
            vel = vel_current
            
            # Predict N steps
            for step in range(self.N):
                # Horizontal acceleration from tilt
                accel = self.params.g * np.tan(angle)
                vel = vel + accel * self.params.dt
                pos = pos + vel * self.params.dt
                
                # Cost
                pos_error = pos - pos_target
                step_cost = (self.Q_pos * pos_error**2 + 
                            self.Q_vel * vel**2 + 
                            self.R * angle**2)
                
                # Constraint penalties
                if pos < self.params.pos_min or pos > self.params.pos_max:
                    step_cost += 10000
                if vel < self.params.vel_min or vel > self.params.vel_max:
                    step_cost += 10000
                
                total_cost += step_cost
            
            if total_cost < best_cost:
                best_cost = total_cost
                best_angle = angle
        
        return best_angle

class Quadcopter:
    def __init__(self, params):
        self.params = params
        
        # State: [x, y, z, vx, vy, vz, roll, pitch, yaw]
        self.state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        # Motor speeds (RPM) - [front, right, back, left]
        self.motor_speeds = np.array([0.0, 0.0, 0.0, 0.0])
        
    def control_to_motors(self, F_total, roll_cmd, pitch_cmd, yaw_rate_cmd):
        """
        Convert high-level control to motor speeds
        F_total: total thrust [N]
        roll_cmd: desired roll angle [rad]
        pitch_cmd: desired pitch angle [rad]
        yaw_rate_cmd: desired yaw rate [rad/s]
        """
        # Simplified mixing matrix
        # For a + configuration quadcopter:
        # Motor 1 (front): pitch
        # Motor 2 (right): roll
        # Motor 3 (back): pitch
        # Motor 4 (left): roll
        
        # Base thrust per motor
        base_thrust = F_total / 4.0
        
        # Convert angles to thrust differences (simplified)
        pitch_thrust = pitch_cmd * 2.0  # gain
        roll_thrust = roll_cmd * 2.0
        yaw_thrust = yaw_rate_cmd * 0.5
        
        # Motor thrusts
        F1 = base_thrust + pitch_thrust - yaw_thrust   # front
        F2 = base_thrust + roll_thrust + yaw_thrust     # right
        F3 = base_thrust - pitch_thrust - yaw_thrust    # back
        F4 = base_thrust - roll_thrust + yaw_thrust     # left
        
        # Convert thrust to motor speed (simplified)
        # F = k_f * omega^2
        self.motor_speeds[0] = np.sqrt(max(0, F1) / self.params.k_f)
        self.motor_speeds[1] = np.sqrt(max(0, F2) / self.params.k_f)
        self.motor_speeds[2] = np.sqrt(max(0, F3) / self.params.k_f)
        self.motor_speeds[3] = np.sqrt(max(0, F4) / self.params.k_f)
        
        # Clamp to limits
        self.motor_speeds = np.clip(self.motor_speeds, 
                                    self.params.motor_min, 
                                    self.params.motor_max)
        
        return self.motor_speeds
    
    def update_dynamics(self, motor_speeds, dt):
        """Update quadcopter state based on motor speeds"""
        # Calculate total thrust from motors
        F_total = sum([self.params.k_f * omega**2 for omega in motor_speeds])
        
        # Calculate moments (simplified)
        # Roll moment (motors 2 and 4)
        M_roll = self.params.L * self.params.k_f * (motor_speeds[1]**2 - motor_speeds[3]**2)
        # Pitch moment (motors 1 and 3)
        M_pitch = self.params.L * self.params.k_f * (motor_speeds[0]**2 - motor_speeds[2]**2)
        
        # Extract current state
        x, y, z, vx, vy, vz, roll, pitch, yaw = self.state
        
        # Rotational dynamics (simplified)
        roll_acc = M_roll / self.params.Ixx
        pitch_acc = M_pitch / self.params.Iyy
        
        roll += roll_acc * dt**2
        pitch += pitch_acc * dt**2
        
        # Translational dynamics
        # Thrust in body frame -> world frame
        ax = (np.sin(pitch) * F_total) / self.params.m
        ay = (-np.sin(roll) * np.cos(pitch) * F_total) / self.params.m
        az = (np.cos(roll) * np.cos(pitch) * F_total) / self.params.m - self.params.g
        
        # Update velocities
        vx += ax * dt
        vy += ay * dt
        vz += az * dt
        
        # Update positions
        x += vx * dt
        y += vy * dt
        z += vz * dt
        
        # Update state
        self.state = np.array([x, y, z, vx, vy, vz, roll, pitch, yaw])

class DroneSimulator:
    def __init__(self):
        self.params = QuadcopterParams()
        self.drone = Quadcopter(self.params)
        self.controller = MPC_3D_Controller(self.params)
        
        # Target position queue
        self.target_queue = queue.Queue()
        self.current_target = np.array([0.0, 0.0, 5.0])  # Start target
        
        # History for plotting
        self.max_history = 500
        self.time_history = []
        self.pos_history = []
        self.vel_history = []
        self.target_history = []
        self.motor_history = []
        self.error_history = []
        
        # Control flag
        self.running = True
        self.sim_time = 0.0
        
    def add_target(self, x, y, z):
        """Add a new target position"""
        self.target_queue.put(np.array([x, y, z]))
        print(f"New target added: ({x:.2f}, {y:.2f}, {z:.2f})")
    
    def control_loop(self):
        """Main control loop (runs in separate thread)"""
        while self.running:
            # Check for new target
            try:
                new_target = self.target_queue.get_nowait()
                self.current_target = new_target
                print(f"Target updated to: {self.current_target}")
            except queue.Empty:
                pass
            
            # Get current state
            state = self.drone.state[:6]  # [x, y, z, vx, vy, vz]
            
            # Compute MPC control
            F_total, roll_cmd, pitch_cmd, yaw_rate_cmd = self.controller.compute_control(
                state, self.current_target
            )
            
            # Convert to motor speeds
            motor_speeds = self.drone.control_to_motors(F_total, roll_cmd, pitch_cmd, yaw_rate_cmd)
            
            # Update dynamics
            self.drone.update_dynamics(motor_speeds, self.params.dt)
            
            # Store history
            self.time_history.append(self.sim_time)
            self.pos_history.append(self.drone.state[:3].copy())
            self.vel_history.append(self.drone.state[3:6].copy())
            self.target_history.append(self.current_target.copy())
            self.motor_history.append(motor_speeds.copy())
            
            # Calculate error
            pos_error = np.linalg.norm(self.drone.state[:3] - self.current_target)
            self.error_history.append(pos_error)
            
            # Keep history limited
            if len(self.time_history) > self.max_history:
                self.time_history.pop(0)
                self.pos_history.pop(0)
                self.vel_history.pop(0)
                self.target_history.pop(0)
                self.motor_history.pop(0)
                self.error_history.pop(0)
            
            # Print status
            if int(self.sim_time * 10) % 10 == 0:  # Every 1 second
                x, y, z = self.drone.state[:3]
                # print(f"t={self.sim_time:.1f}s | Pos: ({x:.2f}, {y:.2f}, {z:.2f}) | "
                #       f"Error: {pos_error:.3f}m | Motors: {motor_speeds.astype(int)}")
            
            # Increment time
            self.sim_time += self.params.dt
            
            # Sleep to maintain real-time
            time.sleep(self.params.dt)
    
    def start_simulation(self):
        """Start the control loop in a separate thread"""
        control_thread = threading.Thread(target=self.control_loop, daemon=True)
        control_thread.start()
        return control_thread
    
    def visualize(self):
        """Create real-time visualization"""
        fig = plt.figure(figsize=(16, 10))
        
        # 3D trajectory plot
        ax1 = fig.add_subplot(2, 3, 1, projection='3d')
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Y [m]')
        ax1.set_zlabel('Z [m]')
        ax1.set_title('3D Trajectory')
        
        # Position vs time
        ax2 = fig.add_subplot(2, 3, 2)
        ax2.set_xlabel('Time [s]')
        ax2.set_ylabel('Position [m]')
        ax2.set_title('Position vs Time')
        ax2.grid(True)
        
        # Velocity vs time
        ax3 = fig.add_subplot(2, 3, 3)
        ax3.set_xlabel('Time [s]')
        ax3.set_ylabel('Velocity [m/s]')
        ax3.set_title('Velocity vs Time')
        ax3.grid(True)
        
        # Motor speeds
        ax4 = fig.add_subplot(2, 3, 4)
        ax4.set_xlabel('Time [s]')
        ax4.set_ylabel('Motor Speed [RPM]')
        ax4.set_title('Motor Speeds')
        ax4.grid(True)
        
        # Position error
        ax5 = fig.add_subplot(2, 3, 5)
        ax5.set_xlabel('Time [s]')
        ax5.set_ylabel('Error [m]')
        ax5.set_title('Position Error')
        ax5.grid(True)
        
        # XY trajectory (top view)
        ax6 = fig.add_subplot(2, 3, 6)
        ax6.set_xlabel('X [m]')
        ax6.set_ylabel('Y [m]')
        ax6.set_title('XY Trajectory (Top View)')
        ax6.grid(True)
        ax6.set_aspect('equal')
        
        def update(frame):
            if len(self.time_history) < 2:
                return
            
            # Convert to arrays
            times = np.array(self.time_history)
            positions = np.array(self.pos_history)
            velocities = np.array(self.vel_history)
            targets = np.array(self.target_history)
            motors = np.array(self.motor_history)
            errors = np.array(self.error_history)
            
            # Clear all axes
            ax1.clear()
            ax2.clear()
            ax3.clear()
            ax4.clear()
            ax5.clear()
            ax6.clear()
            
            # 3D trajectory
            ax1.plot(positions[:, 0], positions[:, 1], positions[:, 2], 
                    'b-', linewidth=2, label='Actual')
            ax1.plot(targets[:, 0], targets[:, 1], targets[:, 2], 
                    'r--', linewidth=1, label='Target')
            ax1.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2], 
                       c='blue', s=100, marker='o')
            ax1.scatter(targets[-1, 0], targets[-1, 1], targets[-1, 2], 
                       c='red', s=100, marker='*')
            ax1.set_xlabel('X [m]')
            ax1.set_ylabel('Y [m]')
            ax1.set_zlabel('Z [m]')
            ax1.set_title('3D Trajectory')
            ax1.legend()
            ax1.grid(True)
            
            # Position vs time
            ax2.plot(times, positions[:, 0], 'r-', label='X', linewidth=2)
            ax2.plot(times, positions[:, 1], 'g-', label='Y', linewidth=2)
            ax2.plot(times, positions[:, 2], 'b-', label='Z', linewidth=2)
            ax2.plot(times, targets[:, 0], 'r--', alpha=0.5)
            ax2.plot(times, targets[:, 1], 'g--', alpha=0.5)
            ax2.plot(times, targets[:, 2], 'b--', alpha=0.5)
            ax2.set_xlabel('Time [s]')
            ax2.set_ylabel('Position [m]')
            ax2.set_title('Position vs Time')
            ax2.legend()
            ax2.grid(True)
            
            # Velocity vs time
            ax3.plot(times, velocities[:, 0], 'r-', label='Vx', linewidth=2)
            ax3.plot(times, velocities[:, 1], 'g-', label='Vy', linewidth=2)
            ax3.plot(times, velocities[:, 2], 'b-', label='Vz', linewidth=2)
            ax3.axhline(y=0, color='k', linestyle='--', alpha=0.3)
            ax3.set_xlabel('Time [s]')
            ax3.set_ylabel('Velocity [m/s]')
            ax3.set_title('Velocity vs Time')
            ax3.legend()
            ax3.grid(True)

            # Get current motor speeds
            current_motor1 = motors[-1, 0] if len(motors) > 0 else 0
            current_motor2 = motors[-1, 1] if len(motors) > 0 else 0
            current_motor3 = motors[-1, 2] if len(motors) > 0 else 0
            current_motor4 = motors[-1, 3] if len(motors) > 0 else 0

            # Plot with current speeds in labels
            ax4.plot(times, motors[:, 0], label=f'Motor 1 (Front): {current_motor1:.0f} RPM', linewidth=2)
            ax4.plot(times, motors[:, 1], label=f'Motor 2 (Right): {current_motor2:.0f} RPM', linewidth=2)
            ax4.plot(times, motors[:, 2], label=f'Motor 3 (Back): {current_motor3:.0f} RPM', linewidth=2)
            ax4.plot(times, motors[:, 3], label=f'Motor 4 (Left): {current_motor4:.0f} RPM', linewidth=2)
            ax4.set_xlabel('Time [s]')
            ax4.set_ylabel('Motor Speed [RPM]')
            ax4.set_title('Motor Speeds')
            ax4.legend()
            ax4.grid(True)
            
            # Position error
            ax5.plot(times, errors, 'r-', linewidth=2)
            ax5.axhline(y=0.1, color='g', linestyle='--', alpha=0.5, label='Threshold')
            ax5.set_xlabel('Time [s]')
            ax5.set_ylabel('Error [m]')
            ax5.set_title('Position Error')
            ax5.legend()
            ax5.grid(True)
            ax5.set_ylim([0, max(errors) * 1.1 if max(errors) > 0 else 1])
            
            # XY trajectory
            ax6.plot(positions[:, 0], positions[:, 1], 'b-', linewidth=2, label='Actual')
            ax6.plot(targets[:, 0], targets[:, 1], 'r--', linewidth=1, label='Target')
            ax6.scatter(positions[-1, 0], positions[-1, 1], c='blue', s=100, marker='o')
            ax6.scatter(targets[-1, 0], targets[-1, 1], c='red', s=100, marker='*')
            ax6.set_xlabel('X [m]')
            ax6.set_ylabel('Y [m]')
            ax6.set_title('XY Trajectory (Top View)')
            ax6.legend()
            ax6.grid(True)
            ax6.set_aspect('equal')
            
            plt.tight_layout()
        
        # Create animation
        ani = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
        plt.show()


def interactive_mode():
    """Interactive mode to send target coordinates"""
    print("\n" + "="*60)
    print("3D QUADCOPTER MPC CONTROLLER")
    print("="*60)
    print("\nStarting simulation...")
    
    # Create simulator
    sim = DroneSimulator()
    
    # Start control loop
    sim.start_simulation()
    
    # Wait a moment for initialization
    time.sleep(0.5)
    
    print("\n" + "-"*60)
    print("COMMANDS:")
    print("  Enter coordinates: x y z (e.g., '5 3 10')")
    print("  Predefined paths:")
    print("    'square'  - Fly a square pattern")
    print("    'circle'  - Fly a circular pattern")
    print("    'hover'   - Hover at 5m")
    print("    'land'    - Land at origin")
    print("  'quit' or 'q' - Exit simulation")
    print("-"*60)
    
    # Start visualization in main thread
    viz_thread = threading.Thread(target=sim.visualize, daemon=True)
    viz_thread.start()
    
    # Give visualization time to start
    time.sleep(1.0)
    
    # Interactive input loop
    try:
        while sim.running:
            try:
                user_input = input("\nEnter target (x y z) or command: ").strip().lower()
                
                if user_input in ['quit', 'q', 'exit']:
                    print("Stopping simulation...")
                    sim.running = False
                    break
                
                elif user_input == 'square':
                    print("Flying square pattern...")
                    sim.add_target(5, 0, 5)
                    time.sleep(3)
                    sim.add_target(5, 5, 5)
                    time.sleep(3)
                    sim.add_target(0, 5, 5)
                    time.sleep(3)
                    sim.add_target(0, 0, 5)
                
                elif user_input == 'circle':
                    print("Flying circular pattern...")
                    radius = 5
                    for angle in np.linspace(0, 2*np.pi, 8):
                        x = radius * np.cos(angle)
                        y = radius * np.sin(angle)
                        sim.add_target(x, y, 5)
                        time.sleep(2)
                
                elif user_input == 'hover':
                    print("Hovering at 5m...")
                    sim.add_target(0, 0, 5)
                
                elif user_input == 'land':
                    print("Landing...")
                    sim.add_target(0, 0, 0.5)
                
                else:
                    # Parse coordinates
                    parts = user_input.split()
                    if len(parts) == 3:
                        x, y, z = map(float, parts)
                        sim.add_target(x, y, z)
                    else:
                        print("Invalid input. Use: x y z")
            
            except ValueError:
                print("Invalid coordinates. Please enter three numbers.")
            except KeyboardInterrupt:
                print("\nStopping simulation...")
                sim.running = False
                break
    
    except Exception as e:
        print(f"Error: {e}")
        sim.running = False
    
    print("Simulation ended.")


def demo_mode():
    """Run automated demo with predefined waypoints"""
    print("\n" + "="*60)
    print("3D QUADCOPTER MPC CONTROLLER - DEMO MODE")
    print("="*60)
    
    # Create simulator
    sim = DroneSimulator()
    
    # Start control loop
    sim.start_simulation()
    
    # Predefined waypoints
    waypoints = [
        (0, 0, 5),    # Takeoff
        (0, 5, 5),    # Move forward
        (0, 0, 5),    # Move right
    ]
    
    print(f"\nFlying through {len(waypoints)} waypoints...")
    
    # Add waypoints with delays
    def add_waypoints():
        for i, (x, y, z) in enumerate(waypoints):
            time.sleep(4)  # Wait between waypoints
            sim.add_target(x, y, z)
            print(f"Waypoint {i+1}/{len(waypoints)}: ({x}, {y}, {z})")
    
    waypoint_thread = threading.Thread(target=add_waypoints, daemon=True)
    waypoint_thread.start()
    
    # Start visualization
    sim.visualize()
    
    sim.running = False

if __name__ == "__main__":
    print("\nSelect mode:")
    print("1. Interactive mode (manual waypoint control)")
    print("2. Demo mode (automated flight)")
    interactive_mode()
    # try:
    #     choice = input("Enter choice (1 or 2): ").strip()
        
    #     if choice == '1':
    #         interactive_mode()
    #     elif choice == '2':
    #         demo_mode()
    #     else:
    #         print("Invalid choice. Running demo mode...")
    #         demo_mode()