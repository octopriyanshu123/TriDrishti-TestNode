"""
MPC MOTOR CONTROLLER FOR DIFFERENTIAL DRIVE ROBOT
Receives cmd_vel from DWA Local Planner and controls motors using MPC

SYSTEM FLOW:
DWA Planner → cmd_vel (linear, angular) → MPC Motor Controller → Motor speeds (left, right)
"""

import numpy as np
from abc import ABC, abstractmethod

# ROS imports (conditional)
try:
    import rospy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Float64MultiArray
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("ROS not available - running in test mode only")

# ============================================
# STEP 1: UNDERSTAND THE SYSTEM
# ============================================
"""
DWA gives you: cmd_vel with [v, ω]
    v = linear velocity [m/s]
    ω = angular velocity [rad/s]

Your robot has: 2 motors (left wheel, right wheel)
    ω_L = left wheel angular velocity [rad/s]
    ω_R = right wheel angular velocity [rad/s]

Relationship (differential drive kinematics):
    v = (R/2) * (ω_L + ω_R)        # linear velocity
    ω = (R/L) * (ω_R - ω_L)        # angular velocity

Where:
    R = wheel radius [m]
    L = wheelbase (distance between wheels) [m]

Goal: Use MPC to track cmd_vel while considering motor dynamics!
"""

# ============================================
# SYSTEM MODEL: DIFFERENTIAL DRIVE ROBOT
# ============================================
class SystemModel(ABC):
    @abstractmethod
    def get_state_size(self):
        pass
    
    @abstractmethod
    def get_control_size(self):
        pass
    
    @abstractmethod
    def dynamics(self, state, control, dt):
        pass
    
    @abstractmethod
    def get_control_bounds(self):
        pass
    
    @abstractmethod
    def get_state_bounds(self):
        pass


class DifferentialDriveMotors(SystemModel):
    """
    Motor dynamics for differential drive robot
    
    State: [ω_L, ω_R] - actual motor angular velocities
    Control: [V_L, V_R] - motor voltages (PWM)
    
    Motor dynamics (first-order):
    dω/dt = (V/K_v - ω) / τ
    
    Where:
        K_v = voltage to velocity constant
        τ = motor time constant
    """
    
    def __init__(self, wheel_radius=0.05, wheelbase=0.3, 
                 K_v=10.0, tau=0.1, max_voltage=12.0, max_rpm=300):
        """
        Args:
            wheel_radius: Radius of wheel [m]
            wheelbase: Distance between wheels [m]
            K_v: Motor constant [rad/s per Volt]
            tau: Motor time constant [s]
            max_voltage: Maximum motor voltage [V]
            max_rpm: Maximum motor RPM
        """
        self.R = wheel_radius
        self.L = wheelbase
        self.K_v = K_v
        self.tau = tau
        self.V_max = max_voltage
        self.omega_max = (max_rpm * 2 * np.pi) / 60  # Convert RPM to rad/s
    
    def get_state_size(self):
        return 2  # [ω_L, ω_R]
    
    def get_control_size(self):
        return 2  # [V_L, V_R]
    
    def dynamics(self, state, control, dt):
        """
        First-order motor dynamics:
        ω_next = ω + (V*K_v - ω)/τ * dt
        """
        omega_L, omega_R = state
        V_L, V_R = control
        
        # Motor dynamics (exponential response to voltage)
        d_omega_L = ((V_L * self.K_v) - omega_L) / self.tau
        d_omega_R = ((V_R * self.K_v) - omega_R) / self.tau
        
        omega_L_next = omega_L + d_omega_L * dt
        omega_R_next = omega_R + d_omega_R * dt
        
        return np.array([omega_L_next, omega_R_next])
    
    def get_control_bounds(self):
        """Motor voltage limits"""
        V_min = np.array([-self.V_max, -self.V_max])
        V_max = np.array([self.V_max, self.V_max])
        return V_min, V_max
    
    def get_state_bounds(self):
        """Motor speed limits"""
        omega_min = np.array([-self.omega_max, -self.omega_max])
        omega_max = np.array([self.omega_max, self.omega_max])
        return omega_min, omega_max
    
    def cmd_vel_to_wheel_speeds(self, v, omega):
        """
        Convert cmd_vel to desired wheel speeds
        
        Args:
            v: linear velocity [m/s]
            omega: angular velocity [rad/s]
        
        Returns:
            [ω_L, ω_R]: desired wheel angular velocities [rad/s]
        """
        # Differential drive inverse kinematics
        omega_L = (v - omega * self.L / 2) / self.R
        omega_R = (v + omega * self.L / 2) / self.R
        
        return np.array([omega_L, omega_R])
    
    def wheel_speeds_to_cmd_vel(self, omega_L, omega_R):
        """
        Convert wheel speeds to robot velocity
        
        Args:
            omega_L: left wheel angular velocity [rad/s]
            omega_R: right wheel angular velocity [rad/s]
        
        Returns:
            [v, omega]: linear and angular velocity
        """
        v = self.R * (omega_L + omega_R) / 2
        omega = self.R * (omega_R - omega_L) / self.L
        
        return v, omega


# ============================================
# MPC CONTROLLER
# ============================================
class MPC_MotorController:
    """MPC for motor control"""
    
    def __init__(self, system_model, dt, horizon=10):
        self.model = system_model
        self.dt = dt
        self.N = horizon
        
        # Cost weights
        self.Q = 100.0  # Tracking weight
        self.R = 0.1    # Control effort weight
        
        # Get system properties
        self.n_states = system_model.get_state_size()
        self.n_controls = system_model.get_control_size()
        self.u_min, self.u_max = system_model.get_control_bounds()
        self.x_min, self.x_max = system_model.get_state_bounds()
        
        # Control sampling
        self.n_samples = 20
    
    def set_weights(self, Q, R):
        self.Q = Q
        self.R = R
    
    def compute_control(self, current_state, target_state):
        """Compute optimal motor voltages using MPC"""
        
        # Generate voltage candidates
        voltage_candidates = self._generate_voltage_grid()
        
        best_voltage = np.zeros(self.n_controls)
        best_cost = float('inf')
        
        # Evaluate each voltage combination
        for voltage in voltage_candidates:
            cost = self._evaluate_voltage(current_state, target_state, voltage)
            
            if cost < best_cost:
                best_cost = cost
                best_voltage = voltage.copy()
        
        return best_voltage
    
    def _generate_voltage_grid(self):
        """Generate voltage candidates for both motors"""
        V_L_samples = np.linspace(self.u_min[0], self.u_max[0], self.n_samples)
        V_R_samples = np.linspace(self.u_min[1], self.u_max[1], self.n_samples)
        
        candidates = []
        for V_L in V_L_samples:
            for V_R in V_R_samples:
                candidates.append(np.array([V_L, V_R]))
        
        return candidates
    
    def _evaluate_voltage(self, initial_state, target_state, voltage):
        """Evaluate cost of applying voltage over prediction horizon"""
        total_cost = 0.0
        state = initial_state.copy()
        
        # Predict N steps ahead
        for step in range(self.N):
            # Predict next state
            state = self.model.dynamics(state, voltage, self.dt)
            
            # Calculate cost
            state_error = state - target_state
            state_cost = self.Q * np.sum(state_error**2)
            control_cost = self.R * np.sum(voltage**2)
            
            # Constraint penalties
            constraint_penalty = 0.0
            for i in range(self.n_states):
                if state[i] < self.x_min[i]:
                    constraint_penalty += 1000 * (self.x_min[i] - state[i])**2
                if state[i] > self.x_max[i]:
                    constraint_penalty += 1000 * (state[i] - self.x_max[i])**2
            
            total_cost += state_cost + control_cost + constraint_penalty
        
        return total_cost


# ============================================
# ROS NODE: MPC MOTOR CONTROLLER
# ============================================
class MPCMotorControllerNode:
    """
    ROS node that:
    1. Subscribes to /cmd_vel from DWA planner
    2. Uses MPC to compute motor voltages
    3. Publishes motor commands
    """
    
    def __init__(self):
        rospy.init_node('mpc_motor_controller', anonymous=True)
        
        # Robot parameters (CHANGE THESE TO MATCH YOUR ROBOT!)
        wheel_radius = rospy.get_param('~wheel_radius', 0.05)  # 5cm wheels
        wheelbase = rospy.get_param('~wheelbase', 0.3)         # 30cm between wheels
        K_v = rospy.get_param('~motor_constant', 10.0)         # Motor constant
        tau = rospy.get_param('~motor_tau', 0.1)               # Motor time constant
        max_voltage = rospy.get_param('~max_voltage', 12.0)    # 12V max
        max_rpm = rospy.get_param('~max_rpm', 300)             # 300 RPM max
        
        # MPC parameters
        dt = rospy.get_param('~mpc_dt', 0.05)                  # 50ms control loop
        horizon = rospy.get_param('~mpc_horizon', 10)          # 10 step horizon
        Q = rospy.get_param('~mpc_Q', 100.0)                   # Tracking weight
        R = rospy.get_param('~mpc_R', 0.1)                     # Control weight
        
        # Create system model
        self.motor_model = DifferentialDriveMotors(
            wheel_radius=wheel_radius,
            wheelbase=wheelbase,
            K_v=K_v,
            tau=tau,
            max_voltage=max_voltage,
            max_rpm=max_rpm
        )
        
        # Create MPC controller
        self.mpc = MPC_MotorController(self.motor_model, dt=dt, horizon=horizon)
        self.mpc.set_weights(Q=Q, R=R)
        
        # Current state (motor speeds)
        self.current_motor_speeds = np.array([0.0, 0.0])  # [ω_L, ω_R]
        
        # Target from DWA
        self.target_v = 0.0
        self.target_omega = 0.0
        
        # Subscribers
        self.cmd_vel_sub = rospy.Subscriber(
            '/cmd_vel', 
            Twist, 
            self.cmd_vel_callback
        )
        
        # Publishers
        self.motor_cmd_pub = rospy.Publisher(
            '/motor_commands',
            Float64MultiArray,
            queue_size=10
        )
        
        # Diagnostics publisher (optional)
        self.diagnostics_pub = rospy.Publisher(
            '/mpc_diagnostics',
            Float64MultiArray,
            queue_size=10
        )
        
        # Control loop rate
        self.rate = rospy.Rate(1.0 / dt)
        
        rospy.loginfo("MPC Motor Controller initialized!")
        rospy.loginfo(f"Wheel radius: {wheel_radius}m, Wheelbase: {wheelbase}m")
        rospy.loginfo(f"MPC: dt={dt}s, horizon={horizon}, Q={Q}, R={R}")
    
    def cmd_vel_callback(self, msg):
        """Callback for /cmd_vel from DWA planner"""
        self.target_v = msg.linear.x
        self.target_omega = msg.angular.z
    
    def run(self):
        """Main control loop"""
        rospy.loginfo("Starting MPC motor control loop...")
        
        while not rospy.is_shutdown():
            # Convert cmd_vel to target wheel speeds
            target_wheel_speeds = self.motor_model.cmd_vel_to_wheel_speeds(
                self.target_v, 
                self.target_omega
            )
            
            # Compute optimal motor voltages using MPC
            motor_voltages = self.mpc.compute_control(
                self.current_motor_speeds,
                target_wheel_speeds
            )
            
            # Update motor state (simulate or get from encoders)
            self.current_motor_speeds = self.motor_model.dynamics(
                self.current_motor_speeds,
                motor_voltages,
                self.mpc.dt
            )
            
            # Publish motor commands
            motor_msg = Float64MultiArray()
            motor_msg.data = motor_voltages.tolist()
            self.motor_cmd_pub.publish(motor_msg)
            
            # Publish diagnostics
            diag_msg = Float64MultiArray()
            actual_v, actual_omega = self.motor_model.wheel_speeds_to_cmd_vel(
                self.current_motor_speeds[0],
                self.current_motor_speeds[1]
            )
            diag_msg.data = [
                self.target_v, actual_v,                    # Linear velocity
                self.target_omega, actual_omega,            # Angular velocity
                self.current_motor_speeds[0],               # Left wheel speed
                self.current_motor_speeds[1],               # Right wheel speed
                motor_voltages[0], motor_voltages[1]        # Motor voltages
            ]
            self.diagnostics_pub.publish(diag_msg)
            
            # Log
            rospy.loginfo_throttle(1.0, 
                f"Target: v={self.target_v:.3f}, ω={self.target_omega:.3f} | "
                f"Actual: v={actual_v:.3f}, ω={actual_omega:.3f} | "
                f"Voltage: [{motor_voltages[0]:.2f}, {motor_voltages[1]:.2f}]V | "
                f"Motors: [{self.current_motor_speeds[0]:.2f}, {self.current_motor_speeds[1]:.2f}] rad/s"
            )
            
            self.rate.sleep()


# ============================================
# STANDALONE TEST (NO ROS)
# ============================================
def test_without_ros():
    """Test MPC motor controller without ROS"""
    print("\n" + "="*70)
    print("MPC MOTOR CONTROLLER TEST (NO ROS)")
    print("="*70)
    
    # Create system
    motor_model = DifferentialDriveMotors(
        wheel_radius=0.05,    # 5cm
        wheelbase=0.3,        # 30cm
        K_v=10.0,
        tau=0.1,
        max_voltage=12.0,
        max_rpm=300
    )
    
    # Create MPC
    mpc = MPC_MotorController(motor_model, dt=0.05, horizon=10)
    mpc.set_weights(Q=100.0, R=0.1)
    
    # Test scenarios
    scenarios = [
        ("Forward", 0.5, 0.0),      # v=0.5m/s, ω=0
        ("Turn left", 0.3, 0.5),    # v=0.3m/s, ω=0.5rad/s
        ("Spin", 0.0, 1.0),         # v=0, ω=1.0rad/s
        ("Stop", 0.0, 0.0)          # v=0, ω=0
    ]
    
    current_state = np.array([0.0, 0.0])  # Start from rest
    
    for name, target_v, target_omega in scenarios:
        print(f"\n{name}: v={target_v}m/s, ω={target_omega}rad/s")
        print("-"*70)
        print(f"{'Time[s]':<8} {'ωL[rad/s]':<12} {'ωR[rad/s]':<12} "
              f"{'VL[V]':<10} {'VR[V]':<10} {'v[m/s]':<10} {'ω[rad/s]':<10}")
        print("-"*70)
        
        # Convert to target wheel speeds
        target_state = motor_model.cmd_vel_to_wheel_speeds(target_v, target_omega)
        
        # Simulate for 2 seconds
        for step in range(40):
            time = step * 0.05
            
            # MPC control
            voltage = mpc.compute_control(current_state, target_state)
            
            # Update state
            current_state = motor_model.dynamics(current_state, voltage, 0.05)
            
            # Calculate actual robot velocity
            actual_v, actual_omega = motor_model.wheel_speeds_to_cmd_vel(
                current_state[0], current_state[1]
            )
            
            # Print every 0.2s
            if step % 4 == 0:
                print(f"{time:<8.2f} {current_state[0]:<12.3f} {current_state[1]:<12.3f} "
                      f"{voltage[0]:<10.2f} {voltage[1]:<10.2f} "
                      f"{actual_v:<10.3f} {actual_omega:<10.3f}")
            
            # Check convergence
            if np.linalg.norm(current_state - target_state) < 0.5:
                print(f"✓ Converged at t={time:.2f}s")
                break
    
    print("\n" + "="*70)
    print("TEST COMPLETE")
    print("="*70)


# ============================================
# MAIN
# ============================================
if __name__ == '__main__':
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == '--test':
        # Run standalone test
        test_without_ros()
    else:
        # Run ROS node
        if not ROS_AVAILABLE:
            print("ERROR: ROS not available!")
            print("Run with --test flag for standalone testing:")
            print("  python mpc_motor_controller_ros.py --test")
            sys.exit(1)
        
        try:
            node = MPCMotorControllerNode()
            node.run()
        except rospy.ROSInterruptException:
            pass
