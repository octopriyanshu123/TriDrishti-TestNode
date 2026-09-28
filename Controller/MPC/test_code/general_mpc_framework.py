"""
GENERAL MPC CONTROLLER FRAMEWORK
A flexible MPC implementation that can adapt to any system

STEP-BY-STEP GUIDE TO ADAPT MPC TO NEW SITUATIONS
"""

import numpy as np
from abc import ABC, abstractmethod

# ============================================
# STEP 1: DEFINE YOUR SYSTEM DYNAMICS
# ============================================
"""
To adapt MPC to a new system, you need to define:
1. State variables (what describes your system?)
2. Control inputs (what can you control?)
3. System dynamics (how does state change with control?)
4. Constraints (what are the limits?)
"""

class SystemModel(ABC):
    """
    Abstract base class for any dynamic system
    You must implement these methods for your specific system
    """
    
    @abstractmethod
    def get_state_size(self):
        """Return number of state variables"""
        pass
    
    @abstractmethod
    def get_control_size(self):
        """Return number of control inputs"""
        pass
    
    @abstractmethod
    def dynamics(self, state, control, dt):
        """
        Define how state evolves with control
        
        Args:
            state: current state vector
            control: control input vector
            dt: time step
            
        Returns:
            next_state: state after one time step
        """
        pass
    
    @abstractmethod
    def get_control_bounds(self):
        """
        Return min and max values for each control input
        
        Returns:
            control_min: array of minimum control values
            control_max: array of maximum control values
        """
        pass
    
    @abstractmethod
    def get_state_bounds(self):
        """
        Return min and max values for each state (optional, can be None)
        
        Returns:
            state_min: array of minimum state values (or None)
            state_max: array of maximum state values (or None)
        """
        pass


# ============================================
# STEP 2: GENERAL MPC CONTROLLER
# ============================================
class MPC_Controller:
    """
    General MPC Controller that works with ANY system
    Just plug in your SystemModel!
    """
    
    def __init__(self, system_model, dt, horizon=10):
        """
        Args:
            system_model: Instance of SystemModel (your system)
            dt: Time step for prediction [s]
            horizon: Prediction horizon (number of steps to look ahead)
        """
        self.model = system_model
        self.dt = dt
        self.N = horizon
        
        # Cost function weights (tune these!)
        self.Q = 100.0  # State error weight
        self.R = 0.1    # Control effort weight
        
        # Get system properties
        self.n_states = system_model.get_state_size()
        self.n_controls = system_model.get_control_size()
        self.u_min, self.u_max = system_model.get_control_bounds()
        self.x_min, self.x_max = system_model.get_state_bounds()
        
        # Control sampling
        self.n_samples = 15  # Number of control values to try per dimension
        
    def set_weights(self, Q, R):
        """
        Tune cost function weights
        
        Args:
            Q: State tracking weight (higher = track target more aggressively)
            R: Control effort weight (higher = smoother control)
        """
        self.Q = Q
        self.R = R
    
    def compute_control(self, current_state, target_state, reference_control=None):
        """
        Compute optimal control using MPC
        
        Args:
            current_state: Current state vector
            target_state: Desired state vector
            reference_control: Reference control (e.g., equilibrium), optional
            
        Returns:
            optimal_control: Best control vector to apply
        """
        if reference_control is None:
            reference_control = np.zeros(self.n_controls)
        
        # Generate control candidates
        control_candidates = self._generate_control_candidates()
        
        best_control = reference_control.copy()
        best_cost = float('inf')
        
        # Evaluate each control candidate
        for control in control_candidates:
            cost = self._evaluate_control(current_state, target_state, 
                                         control, reference_control)
            
            if cost < best_cost:
                best_cost = cost
                best_control = control.copy()
        
        return best_control
    
    def _generate_control_candidates(self):
        """Generate candidate control inputs to test"""
        candidates = []
        
        if self.n_controls == 1:
            # 1D control: simple linspace
            u_samples = np.linspace(self.u_min[0], self.u_max[0], self.n_samples)
            candidates = [[u] for u in u_samples]
        
        elif self.n_controls == 2:
            # 2D control: grid sampling
            u1_samples = np.linspace(self.u_min[0], self.u_max[0], self.n_samples)
            u2_samples = np.linspace(self.u_min[1], self.u_max[1], self.n_samples)
            for u1 in u1_samples:
                for u2 in u2_samples:
                    candidates.append([u1, u2])
        
        else:
            # Multi-dimensional: random sampling for efficiency
            n_random_samples = 100
            for _ in range(n_random_samples):
                control = np.random.uniform(self.u_min, self.u_max, self.n_controls)
                candidates.append(control)
        
        return [np.array(c) for c in candidates]
    
    def _evaluate_control(self, initial_state, target_state, control, ref_control):
        """
        Evaluate cost of applying a control over prediction horizon
        
        This is the core MPC prediction step!
        """
        total_cost = 0.0
        state = initial_state.copy()
        
        # Simulate forward for N steps
        for step in range(self.N):
            # Predict next state
            state = self.model.dynamics(state, control, self.dt)
            
            # Calculate cost components
            state_error = state - target_state
            control_error = control - ref_control
            
            # Quadratic cost
            state_cost = self.Q * np.sum(state_error**2)
            control_cost = self.R * np.sum(control_error**2)
            
            # Add constraint penalties
            constraint_penalty = self._compute_constraint_penalty(state, control)
            
            # Total step cost
            step_cost = state_cost + control_cost + constraint_penalty
            total_cost += step_cost
        
        return total_cost
    
    def _compute_constraint_penalty(self, state, control):
        """Add penalty for constraint violations"""
        penalty = 0.0
        
        # State constraints
        if self.x_min is not None and self.x_max is not None:
            for i in range(self.n_states):
                if state[i] < self.x_min[i]:
                    penalty += 1000 * (self.x_min[i] - state[i])**2
                if state[i] > self.x_max[i]:
                    penalty += 1000 * (state[i] - self.x_max[i])**2
        
        # Control constraints (already enforced in sampling, but double-check)
        for i in range(self.n_controls):
            if control[i] < self.u_min[i]:
                penalty += 1000 * (self.u_min[i] - control[i])**2
            if control[i] > self.u_max[i]:
                penalty += 1000 * (control[i] - self.u_max[i])**2
        
        return penalty


# ============================================
# STEP 3: EXAMPLE - DRONE Z-AXIS SYSTEM
# ============================================
class DroneZAxis(SystemModel):
    """
    Example: 1D vertical drone
    State: [z, vz] (position, velocity)
    Control: [F] (thrust)
    """
    
    def __init__(self, mass=1.0, gravity=9.81):
        self.m = mass
        self.g = gravity
    
    def get_state_size(self):
        return 2  # [z, vz]
    
    def get_control_size(self):
        return 1  # [F]
    
    def dynamics(self, state, control, dt):
        """
        Physics: 
        az = F/m - g
        vz_next = vz + az*dt
        z_next = z + vz*dt
        """
        z, vz = state
        F = control[0]
        
        # Acceleration
        az = F / self.m - self.g
        
        # Update velocity and position
        vz_next = vz + az * dt
        z_next = z + vz * dt
        
        return np.array([z_next, vz_next])
    
    def get_control_bounds(self):
        # Thrust limits [N]
        F_min = np.array([0.0])
        F_max = np.array([25.0])
        return F_min, F_max
    
    def get_state_bounds(self):
        # Position and velocity limits
        state_min = np.array([0.0, -5.0])    # [z_min, vz_min]
        state_max = np.array([30.0, 5.0])    # [z_max, vz_max]
        return state_min, state_max


# ============================================
# STEP 4: EXAMPLE - CAR SYSTEM
# ============================================
class Car2D(SystemModel):
    """
    Example: 2D car with simple kinematics
    State: [x, y, vx, vy] (position and velocity)
    Control: [ax, ay] (accelerations)
    """
    
    def get_state_size(self):
        return 4  # [x, y, vx, vy]
    
    def get_control_size(self):
        return 2  # [ax, ay]
    
    def dynamics(self, state, control, dt):
        """
        Simple kinematics:
        vx_next = vx + ax*dt
        vy_next = vy + ay*dt
        x_next = x + vx*dt
        y_next = y + vy*dt
        """
        x, y, vx, vy = state
        ax, ay = control
        
        # Update velocities
        vx_next = vx + ax * dt
        vy_next = vy + ay * dt
        
        # Update positions
        x_next = x + vx * dt
        y_next = y + vy * dt
        
        return np.array([x_next, y_next, vx_next, vy_next])
    
    def get_control_bounds(self):
        # Acceleration limits [m/s²]
        a_min = np.array([-3.0, -3.0])
        a_max = np.array([3.0, 3.0])
        return a_min, a_max
    
    def get_state_bounds(self):
        # Position and velocity limits
        state_min = np.array([-50.0, -50.0, -10.0, -10.0])
        state_max = np.array([50.0, 50.0, 10.0, 10.0])
        return state_min, state_max


# ============================================
# STEP 5: EXAMPLE - TEMPERATURE CONTROL
# ============================================
class TemperatureSystem(SystemModel):
    """
    Example: Room temperature control
    State: [T] (temperature)
    Control: [P] (heater power)
    """
    
    def __init__(self, thermal_mass=1.0, heat_loss=0.1, ambient_temp=20.0):
        self.C = thermal_mass      # Thermal capacitance
        self.k = heat_loss         # Heat loss coefficient
        self.T_ambient = ambient_temp
    
    def get_state_size(self):
        return 1  # [T]
    
    def get_control_size(self):
        return 1  # [P]
    
    def dynamics(self, state, control, dt):
        """
        Heat equation:
        dT/dt = (P - k*(T - T_ambient)) / C
        """
        T = state[0]
        P = control[0]
        
        # Temperature change rate
        dT_dt = (P - self.k * (T - self.T_ambient)) / self.C
        
        # Update temperature
        T_next = T + dT_dt * dt
        
        return np.array([T_next])
    
    def get_control_bounds(self):
        # Heater power limits [W]
        P_min = np.array([0.0])
        P_max = np.array([1000.0])
        return P_min, P_max
    
    def get_state_bounds(self):
        # Temperature limits [°C]
        T_min = np.array([15.0])
        T_max = np.array([30.0])
        return T_min, T_max


# ============================================
# STEP 6: USAGE EXAMPLES
# ============================================

def example_drone():
    """Example: Control drone altitude"""
    print("\n" + "="*70)
    print("EXAMPLE 1: DRONE Z-AXIS CONTROL")
    print("="*70)
    
    # Create system and controller
    drone = DroneZAxis(mass=1.0, gravity=9.81)
    mpc = MPC_Controller(drone, dt=0.1, horizon=10)
    mpc.set_weights(Q=100.0, R=0.1)
    
    # Initial state and target
    state = np.array([0.0, 0.0])  # [z=0m, vz=0m/s]
    target = np.array([10.0, 0.0])  # [z=10m, vz=0m/s]
    
    print(f"\nInitial: z={state[0]:.2f}m, vz={state[1]:.2f}m/s")
    print(f"Target:  z={target[0]:.2f}m, vz={target[1]:.2f}m/s")
    print(f"\n{'Time[s]':<8} {'Z[m]':<10} {'Vz[m/s]':<10} {'Thrust[N]':<12}")
    print("-"*40)
    
    # Simulation
    for step in range(50):
        time = step * 0.1
        
        # Compute optimal control
        control = mpc.compute_control(state, target, 
                                     reference_control=np.array([drone.m * drone.g]))
        
        # Print
        if step % 5 == 0:
            print(f"{time:<8.1f} {state[0]:<10.3f} {state[1]:<10.3f} {control[0]:<12.3f}")
        
        # Update state
        state = drone.dynamics(state, control, 0.1)
        
        # Check if reached target
        if np.linalg.norm(state - target) < 0.5:
            print(f"\n✓ Reached target at t={time:.1f}s")
            break


def example_car():
    """Example: Control car position"""
    print("\n" + "="*70)
    print("EXAMPLE 2: 2D CAR CONTROL")
    print("="*70)
    
    # Create system and controller
    car = Car2D()
    mpc = MPC_Controller(car, dt=0.1, horizon=10)
    mpc.set_weights(Q=50.0, R=0.5)
    
    # Initial state and target
    state = np.array([0.0, 0.0, 0.0, 0.0])  # [x, y, vx, vy] all zero
    target = np.array([10.0, 5.0, 0.0, 0.0])  # Go to (10, 5)
    
    print(f"\nInitial: x={state[0]:.2f}m, y={state[1]:.2f}m")
    print(f"Target:  x={target[0]:.2f}m, y={target[1]:.2f}m")
    print(f"\n{'Time[s]':<8} {'X[m]':<10} {'Y[m]':<10} {'Vx[m/s]':<10} {'Vy[m/s]':<10}")
    print("-"*50)
    
    # Simulation
    for step in range(100):
        time = step * 0.1
        
        # Compute optimal control
        control = mpc.compute_control(state, target)
        
        # Print
        if step % 10 == 0:
            print(f"{time:<8.1f} {state[0]:<10.3f} {state[1]:<10.3f} "
                  f"{state[2]:<10.3f} {state[3]:<10.3f}")
        
        # Update state
        state = car.dynamics(state, control, 0.1)
        
        # Check if reached target
        if np.linalg.norm(state[:2] - target[:2]) < 0.3:
            print(f"\n✓ Reached target at t={time:.1f}s")
            break


def example_temperature():
    """Example: Control room temperature"""
    print("\n" + "="*70)
    print("EXAMPLE 3: TEMPERATURE CONTROL")
    print("="*70)
    
    # Create system and controller
    temp_system = TemperatureSystem(thermal_mass=100.0, heat_loss=5.0, ambient_temp=15.0)
    mpc = MPC_Controller(temp_system, dt=1.0, horizon=20)
    mpc.set_weights(Q=10.0, R=0.01)
    
    # Initial state and target
    state = np.array([15.0])  # Room at ambient temp
    target = np.array([22.0])  # Want 22°C
    
    print(f"\nInitial: T={state[0]:.2f}°C")
    print(f"Target:  T={target[0]:.2f}°C")
    print(f"Ambient: T={temp_system.T_ambient:.2f}°C")
    print(f"\n{'Time[s]':<10} {'Temp[°C]':<12} {'Power[W]':<12}")
    print("-"*35)
    
    # Simulation
    for step in range(100):
        time = step * 1.0
        
        # Compute optimal control
        control = mpc.compute_control(state, target)
        
        # Print
        if step % 5 == 0:
            print(f"{time:<10.0f} {state[0]:<12.2f} {control[0]:<12.1f}")
        
        # Update state
        state = temp_system.dynamics(state, control, 1.0)
        
        # Check if reached target
        if abs(state[0] - target[0]) < 0.5:
            print(f"\n✓ Reached target at t={time:.0f}s")
            break


# ============================================
# STEP 7: ADAPTATION GUIDE
# ============================================
def print_adaptation_guide():
    """Print step-by-step guide for adapting to new systems"""
    print("\n" + "="*70)
    print("HOW TO ADAPT MPC TO YOUR NEW SYSTEM - STEP BY STEP")
    print("="*70)
    
    guide = """
STEP 1: IDENTIFY YOUR SYSTEM
────────────────────────────
Ask yourself:
1. What are the STATE variables? (What describes the system?)
   Examples: position, velocity, temperature, angle, pressure
   
2. What are the CONTROL inputs? (What can you change?)
   Examples: force, power, voltage, flow rate
   
3. What are the DYNAMICS? (How does state change?)
   Write the equations: state_next = f(state, control, dt)


STEP 2: CREATE YOUR SYSTEM CLASS
─────────────────────────────────
class MySystem(SystemModel):
    
    def get_state_size(self):
        return N  # Number of state variables
    
    def get_control_size(self):
        return M  # Number of control inputs
    
    def dynamics(self, state, control, dt):
        # YOUR PHYSICS EQUATIONS HERE
        # Example: x_next = x + v*dt
        #          v_next = v + (F/m)*dt
        return next_state
    
    def get_control_bounds(self):
        u_min = np.array([...])  # Minimum control values
        u_max = np.array([...])  # Maximum control values
        return u_min, u_max
    
    def get_state_bounds(self):
        x_min = np.array([...])  # Minimum state values
        x_max = np.array([...])  # Maximum state values
        return x_min, x_max


STEP 3: CREATE MPC CONTROLLER
──────────────────────────────
my_system = MySystem()
mpc = MPC_Controller(my_system, dt=0.1, horizon=10)


STEP 4: TUNE THE WEIGHTS
─────────────────────────
mpc.set_weights(Q=100.0, R=0.1)

Q (State weight):
  - Higher Q → Track target more aggressively
  - Lower Q → More gradual approach
  
R (Control weight):
  - Higher R → Smoother control (less aggressive)
  - Lower R → More aggressive control


STEP 5: RUN THE CONTROLLER
───────────────────────────
current_state = np.array([...])   # Your current state
target_state = np.array([...])    # Where you want to be

optimal_control = mpc.compute_control(current_state, target_state)

# Apply the control to your real system
# Then repeat!


COMMON SYSTEMS AND THEIR MODELS
────────────────────────────────

1. MECHANICAL SYSTEMS (Robot, Drone, Car)
   State: [position, velocity]
   Control: [force, torque]
   Dynamics: F = ma

2. THERMAL SYSTEMS (Heater, Oven, HVAC)
   State: [temperature]
   Control: [power]
   Dynamics: Heat equation

3. ELECTRICAL SYSTEMS (Motor, Battery)
   State: [current, voltage]
   Control: [input voltage]
   Dynamics: Circuit equations

4. CHEMICAL SYSTEMS (Reactor, Mixer)
   State: [concentration, temperature]
   Control: [flow rate, heat]
   Dynamics: Mass/energy balance

5. ECONOMIC SYSTEMS (Portfolio, Inventory)
   State: [value, stock level]
   Control: [buy/sell amount]
   Dynamics: Supply/demand equations


KEY INSIGHTS
────────────
✓ MPC works for ANY system with predictable dynamics
✓ You just need to define: states, controls, and how they relate
✓ The MPC_Controller handles the optimization automatically
✓ Tune Q and R to get desired behavior
✓ Start with simple dynamics, add complexity gradually
"""
    print(guide)
    
    print("\n" + "="*70)
    print("TRY THE EXAMPLES BELOW!")
    print("="*70)


# ============================================
# MAIN
# ============================================
if __name__ == "__main__":
    # Print guide
    print_adaptation_guide()
    
    # Run examples
    example_drone()
    example_car()
    example_temperature()
    
    print("\n" + "="*70)
    print("✓ ALL EXAMPLES COMPLETE!")
    print("="*70)
    print("\nNow adapt this framework to YOUR system!")
    print("Just create a new class inheriting from SystemModel")
    print("="*70)
