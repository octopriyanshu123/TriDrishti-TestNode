# MPC ADAPTATION QUICK REFERENCE CARD

## What Changes When You Adapt MPC to a New System?

### The 5 Things You MUST Define:

```
┌─────────────────────────────────────────────────────────┐
│  1. STATE VARIABLES      → What describes your system?  │
│  2. CONTROL INPUTS       → What can you control?        │
│  3. DYNAMICS EQUATION    → How does state change?       │
│  4. CONTROL BOUNDS       → Min/max control limits       │
│  5. STATE BOUNDS         → Min/max state limits         │
└─────────────────────────────────────────────────────────┘
```

### What STAYS THE SAME:
- ✓ MPC algorithm (predict → evaluate → select)
- ✓ Cost function structure (Q·error² + R·control²)
- ✓ Optimization logic
- ✓ Receding horizon concept

---

## 📋 Step-by-Step Checklist

### STEP 1: Identify Your System
```
[ ] What is the STATE? (position? temperature? angle?)
[ ] What is the CONTROL? (force? power? voltage?)
[ ] What are the PHYSICS? (F=ma? heat equation? other?)
```

### STEP 2: Define System Class
```python
class MySystem(SystemModel):
    
    def get_state_size(self):
        return ___  # How many state variables?
    
    def get_control_size(self):
        return ___  # How many control inputs?
    
    def dynamics(self, state, control, dt):
        # Write your physics equations here!
        # state_next = ???
        return state_next
    
    def get_control_bounds(self):
        u_min = np.array([___])
        u_max = np.array([___])
        return u_min, u_max
    
    def get_state_bounds(self):
        x_min = np.array([___])
        x_max = np.array([___])
        return x_min, x_max
```

### STEP 3: Create & Run Controller
```python
# Create system
system = MySystem()

# Create MPC controller
mpc = MPC_Controller(system, dt=0.1, horizon=10)

# Tune weights
mpc.set_weights(Q=100.0, R=0.1)

# Control loop
while True:
    control = mpc.compute_control(current_state, target_state)
    # Apply control to real system
    # Update current_state
```

---

## 🔧 Common Systems - Copy & Modify

### 1️⃣ DRONE (Vertical)
```python
State:    [z, vz]           # Position, velocity
Control:  [F]               # Thrust
Dynamics: az = F/m - g
          vz_next = vz + az*dt
          z_next = z + vz*dt
```

### 2️⃣ CAR (2D)
```python
State:    [x, y, vx, vy]    # Position, velocity
Control:  [ax, ay]          # Accelerations
Dynamics: vx_next = vx + ax*dt
          vy_next = vy + ay*dt
          x_next = x + vx*dt
          y_next = y + vy*dt
```

### 3️⃣ TEMPERATURE
```python
State:    [T]               # Temperature
Control:  [P]               # Heater power
Dynamics: dT/dt = (P - k*(T-T_amb))/C
          T_next = T + dT/dt*dt
```

### 4️⃣ PENDULUM
```python
State:    [θ, ω]            # Angle, angular velocity
Control:  [τ]               # Torque
Dynamics: α = τ/I - (g/L)*sin(θ)
          ω_next = ω + α*dt
          θ_next = θ + ω*dt
```

### 5️⃣ TANK LEVEL
```python
State:    [h]               # Water height
Control:  [Q_in]            # Inflow rate
Dynamics: dh/dt = (Q_in - Q_out)/A
          h_next = h + dh/dt*dt
```

### 6️⃣ ROBOT ARM (1 joint)
```python
State:    [θ, ω]            # Joint angle, velocity
Control:  [τ]               # Motor torque
Dynamics: α = (τ - b*ω - k*θ)/I
          ω_next = ω + α*dt
          θ_next = θ + ω*dt
```

---

## ⚙️ Tuning Guide

### Weight Selection:

| Want more... | Adjust |
|-------------|--------|
| Aggressive tracking | ↑ Q |
| Smooth control | ↑ R |
| Faster response | ↓ R |
| Less overshoot | ↑ R, ↑ Q_vel |

### Typical Values:
```
Q (tracking):    10 - 1000
R (smoothness):  0.01 - 10
Horizon N:       5 - 20 steps
```

---

## 🚨 Common Mistakes

❌ **Mistake**: Wrong sign in dynamics equation
✅ **Fix**: Double-check physics (az = F/m - g, NOT + g)

❌ **Mistake**: Unrealistic control bounds
✅ **Fix**: Use actual hardware limits

❌ **Mistake**: Forgetting to update state
✅ **Fix**: state = system.dynamics(state, control, dt)

❌ **Mistake**: Too high Q and R both
✅ **Fix**: They compete! Balance them.

❌ **Mistake**: Prediction horizon too short
✅ **Fix**: N should cover ~1-2 seconds of motion

---

## 💡 Quick Examples

### Example 1: Heater Control
```python
class Heater(SystemModel):
    def __init__(self):
        self.C = 100  # Heat capacity
        self.k = 5    # Heat loss
        
    def get_state_size(self): return 1  # [T]
    def get_control_size(self): return 1  # [P]
    
    def dynamics(self, state, control, dt):
        T = state[0]
        P = control[0]
        dT = (P - self.k*(T-20))/self.C * dt
        return np.array([T + dT])
    
    def get_control_bounds(self):
        return np.array([0]), np.array([1000])  # 0-1000W
    
    def get_state_bounds(self):
        return np.array([15]), np.array([30])  # 15-30°C
```

### Example 2: Tank Filling
```python
class Tank(SystemModel):
    def __init__(self):
        self.A = 1.0   # Cross-section area [m²]
        self.Q_out = 0.1  # Outflow [m³/s]
    
    def get_state_size(self): return 1  # [h]
    def get_control_size(self): return 1  # [Q_in]
    
    def dynamics(self, state, control, dt):
        h = state[0]
        Q_in = control[0]
        dh = (Q_in - self.Q_out) / self.A * dt
        return np.array([max(0, h + dh)])
    
    def get_control_bounds(self):
        return np.array([0]), np.array([0.5])  # 0-0.5 m³/s
    
    def get_state_bounds(self):
        return np.array([0]), np.array([2])  # 0-2m height
```

---

## 🎓 Learning Path

1. ✅ Understand basic MPC (predict → cost → select)
2. ✅ Run the drone example
3. ✅ Modify drone mass or gravity
4. ✅ Try the car or temperature example
5. ✅ Create your own simple system (1-2 states)
6. ✅ Add complexity gradually

---

## 📚 Resources

**Files:**
- `general_mpc_framework.py` - Full framework with examples
- `z_axis_mpc_drone.py` - Simple working example
- This file - Quick reference

**Key Concepts:**
- System dynamics = how state evolves
- Cost function = what you want to minimize
- Constraints = physical limits
- Receding horizon = recompute every step

---

## ✨ Template for New System

```python
# 1. Define your system
class MyNewSystem(SystemModel):
    def __init__(self, param1, param2):
        self.p1 = param1
        self.p2 = param2
    
    def get_state_size(self):
        return N  # Number of states
    
    def get_control_size(self):
        return M  # Number of controls
    
    def dynamics(self, state, control, dt):
        # Extract states
        x1 = state[0]
        x2 = state[1]
        # ... etc
        
        # Extract controls
        u1 = control[0]
        # ... etc
        
        # Compute derivatives/updates
        dx1 = ... # your equation
        dx2 = ... # your equation
        
        # Return next state
        return np.array([x1 + dx1*dt, x2 + dx2*dt, ...])
    
    def get_control_bounds(self):
        return np.array([u_min, ...]), np.array([u_max, ...])
    
    def get_state_bounds(self):
        return np.array([x_min, ...]), np.array([x_max, ...])

# 2. Create and use
system = MyNewSystem(param1=..., param2=...)
mpc = MPC_Controller(system, dt=0.1, horizon=10)
mpc.set_weights(Q=100, R=0.1)

# 3. Control loop
state = np.array([...])  # initial state
target = np.array([...])  # target state

for step in range(100):
    control = mpc.compute_control(state, target)
    state = system.dynamics(state, control, dt)
    print(f"State: {state}, Control: {control}")
```

---

**Remember**: MPC is just predict → evaluate → choose. 
The physics changes, the algorithm doesn't! 🚀
