# C++ MPC Framework

## 📦 Complete MPC Implementation in C++

A production-ready Model Predictive Control framework in C++ with examples.

---

## 🚀 Quick Start

### Compile & Run
```bash
g++ -std=c++11 mpc_framework.cpp -o mpc_framework
./mpc_framework
```

### Output
```
✓ Temperature control simulation
✓ Drone altitude control simulation
```

---

## 📁 File Structure

```
mpc_framework.cpp
├── SystemModel (base class)
├── TemperatureSystem (example)
├── DroneZAxis (example)
├── MPC_Controller (main controller)
└── main() (simulations)
```

---

## 🎯 How To Use

### Step 1: Create Your System Class

```cpp
class MySystem : public SystemModel {
private:
    // Your parameters
    double param1, param2;
    
public:
    MySystem(double p1, double p2) : param1(p1), param2(p2) {}
    
    int get_state_size() const override {
        return 2;  // Example: [position, velocity]
    }
    
    int get_control_size() const override {
        return 1;  // Example: [force]
    }
    
    vector<double> dynamics(const vector<double>& state,
                           const vector<double>& control,
                           double dt) override {
        // Implement your physics here
        double x = state[0];
        double v = state[1];
        double F = control[0];
        
        double a = F / mass;
        double v_next = v + a * dt;
        double x_next = x + v * dt;
        
        return vector<double>{x_next, v_next};
    }
    
    void get_control_bounds(vector<double>& u_min,
                           vector<double>& u_max) const override {
        u_min = vector<double>{-100.0};  // Min force
        u_max = vector<double>{100.0};   // Max force
    }
    
    void get_state_bounds(vector<double>& x_min,
                         vector<double>& x_max) const override {
        x_min = vector<double>{-10.0, -5.0};  // [pos_min, vel_min]
        x_max = vector<double>{10.0, 5.0};    // [pos_max, vel_max]
    }
};
```

### Step 2: Create MPC Controller

```cpp
// Create system
auto my_system = make_shared<MySystem>(param1, param2);

// Create MPC controller
MPC_Controller mpc(my_system, dt, horizon);
//                            ↑    ↑
//                         0.1s   10 steps

// Tune weights
mpc.set_weights(Q, R);
//              ↑  ↑
//            100  0.1
```

### Step 3: Control Loop

```cpp
vector<double> state = {0.0, 0.0};      // Initial state
vector<double> target = {10.0, 0.0};    // Target state

for (int step = 0; step < 100; step++) {
    // Compute optimal control
    vector<double> control = mpc.compute_control(state, target);
    
    // Apply to real system (or simulate)
    state = my_system->dynamics(state, control, dt);
    
    // Print status
    cout << "State: " << state[0] << ", " << state[1] << endl;
    cout << "Control: " << control[0] << endl;
}
```

---

## 🔧 Class Reference

### SystemModel (Base Class)

**Pure Virtual Methods (must implement):**

```cpp
int get_state_size() const
```
- Returns number of state variables
- Example: `return 2;` for [position, velocity]

```cpp
int get_control_size() const
```
- Returns number of control inputs
- Example: `return 1;` for [force]

```cpp
vector<double> dynamics(state, control, dt)
```
- Implements system physics
- Returns next state after one time step

```cpp
void get_control_bounds(u_min, u_max)
```
- Sets minimum and maximum control values
- Example: `u_min = {0.0}; u_max = {100.0};`

```cpp
void get_state_bounds(x_min, x_max)
```
- Sets minimum and maximum state values
- Example: `x_min = {-10, -5}; x_max = {10, 5};`

---

### MPC_Controller

**Constructor:**
```cpp
MPC_Controller(shared_ptr<SystemModel> model, double dt, int horizon)
```
- `model`: Your system model
- `dt`: Time step [seconds]
- `horizon`: Prediction horizon [steps]

**Methods:**

```cpp
void set_weights(double Q, double R)
```
- Set cost function weights
- `Q`: State tracking weight (100-1000 typical)
- `R`: Control effort weight (0.01-10 typical)

```cpp
vector<double> compute_control(state, target)
```
- Main MPC computation
- Returns optimal control vector
- Call this every control cycle

---

## 📊 Examples Included

### 1. Temperature Control

**System:**
- State: Temperature [°C]
- Control: Heater power [W]
- Goal: Heat room from 15°C to 22°C

**Physics:**
```
dT/dt = (P - k*(T - T_ambient)) / C
```

**Usage:**
```cpp
auto temp = make_shared<TemperatureSystem>(100.0, 5.0, 15.0);
MPC_Controller mpc(temp, 1.0, 20);
```

### 2. Drone Altitude

**System:**
- State: [altitude, velocity]
- Control: Thrust force [N]
- Goal: Reach 10m altitude

**Physics:**
```
az = F/m - g
vz = vz + az*dt
z = z + vz*dt
```

**Usage:**
```cpp
auto drone = make_shared<DroneZAxis>(1.0, 9.81);
MPC_Controller mpc(drone, 0.1, 10);
```

---

## ⚙️ Tuning Guide

### Time Step (dt)

| System Type | Typical dt |
|-------------|-----------|
| Fast motors | 0.01 - 0.05s |
| Robots | 0.05 - 0.1s |
| Temperature | 1 - 10s |
| Chemical process | 10 - 60s |

### Horizon (N)

```
Rule: N * dt should cover 1-3× settling time

Example:
  Settling time = 5 seconds
  dt = 0.1s
  N ≥ 50 steps (but 10-20 often sufficient)
```

### Weights (Q, R)

**Q (State Tracking):**
- Higher Q → More aggressive tracking
- Typical: 10 - 1000

**R (Control Smoothness):**
- Higher R → Smoother control
- Typical: 0.01 - 10

**Start with:**
```cpp
mpc.set_weights(100.0, 0.1);
```

**If oscillating:**
```cpp
mpc.set_weights(50.0, 1.0);  // Less aggressive
```

**If too slow:**
```cpp
mpc.set_weights(200.0, 0.01);  // More aggressive
```

---

## 🔍 Advanced Features

### Custom Control Sampling

Modify `n_samples` in MPC_Controller:

```cpp
// In MPC_Controller constructor
n_samples = 50;  // More samples = better but slower
```

### Constraint Penalties

Automatically handled in `compute_constraint_penalty()`:

```cpp
// Penalty weight = 1000.0 (adjustable)
if (state[i] < x_min[i])
    penalty += 1000.0 * (x_min[i] - state[i])^2
```

### Multi-Dimensional Systems

Works automatically! Just set:

```cpp
int get_state_size() const override {
    return 6;  // [x, y, z, vx, vy, vz]
}

int get_control_size() const override {
    return 3;  // [Fx, Fy, Fz]
}
```

---

## 📝 Adding Your Own System

### Template:

```cpp
class MyRobot : public SystemModel {
private:
    // Your parameters here
    double mass, length, damping;
    
public:
    MyRobot(double m, double l, double d) 
        : mass(m), length(l), damping(d) {}
    
    int get_state_size() const override {
        return 4;  // [x, y, vx, vy]
    }
    
    int get_control_size() const override {
        return 2;  // [Fx, Fy]
    }
    
    vector<double> dynamics(const vector<double>& state,
                           const vector<double>& control,
                           double dt) override {
        // Extract states
        double x = state[0], y = state[1];
        double vx = state[2], vy = state[3];
        double Fx = control[0], Fy = control[1];
        
        // Physics equations
        double ax = Fx/mass - damping*vx;
        double ay = Fy/mass - damping*vy;
        
        // Update
        double vx_next = vx + ax*dt;
        double vy_next = vy + ay*dt;
        double x_next = x + vx*dt;
        double y_next = y + vy*dt;
        
        return {x_next, y_next, vx_next, vy_next};
    }
    
    void get_control_bounds(vector<double>& u_min,
                           vector<double>& u_max) const override {
        u_min = {-100.0, -100.0};
        u_max = {100.0, 100.0};
    }
    
    void get_state_bounds(vector<double>& x_min,
                         vector<double>& x_max) const override {
        x_min = {-10.0, -10.0, -5.0, -5.0};
        x_max = {10.0, 10.0, 5.0, 5.0};
    }
};
```

### Use it:

```cpp
auto robot = make_shared<MyRobot>(1.0, 0.5, 0.1);
MPC_Controller mpc(robot, 0.05, 15);
mpc.set_weights(100.0, 0.1);

// Control loop...
```

---

## 🐛 Troubleshooting

### Compilation Errors

**Error:** `'make_shared' is not a member of 'std'`
**Fix:** Add `#include <memory>` and use `std::make_shared`

**Error:** `'vector' does not name a type`
**Fix:** Add `using namespace std;` or prefix with `std::`

### Runtime Issues

**Problem:** MPC too slow
**Solution:** 
- Reduce `n_samples` (line ~155)
- Reduce horizon `N`
- Optimize with `-O3` flag: `g++ -std=c++11 -O3 ...`

**Problem:** Oscillations
**Solution:**
- Increase R (smoother control)
- Decrease Q (less aggressive)

**Problem:** Not reaching target
**Solution:**
- Increase Q (more aggressive)
- Check if constraints too tight
- Verify dynamics() is correct

---

## 🚀 Performance Tips

### Optimization Flags
```bash
g++ -std=c++11 -O3 -march=native mpc_framework.cpp -o mpc_framework
```

### Profiling
```bash
g++ -std=c++11 -pg mpc_framework.cpp -o mpc_framework
./mpc_framework
gprof mpc_framework gmon.out > analysis.txt
```

### For Real-Time
- Use fixed-size arrays instead of vectors
- Pre-allocate memory
- Consider using Eigen library for linear algebra
- Implement warm-starting (use previous solution as initial guess)

---

## 📚 Further Reading

- **Python Version:** See `general_mpc_framework.py`
- **Interview Prep:** See `MPC_INTERVIEW_QA.md`
- **Applications:** See `MPC_DWA_INTEGRATION_GUIDE.md`

---

## ✅ What You Can Do Now

1. ✓ Run the examples
2. ✓ Modify parameters (Q, R, N, dt)
3. ✓ Create your own system class
4. ✓ Integrate with real hardware
5. ✓ Port to embedded systems (Arduino, STM32, etc.)

---

## 🎓 Key Concepts in Code

**MPC Algorithm:**
```cpp
for each candidate control:
    1. Predict N steps ahead
    2. Calculate cost (tracking + effort + constraints)
    3. Keep track of best

Apply best control (receding horizon)
```

**Cost Function:**
```cpp
cost = Q * (state - target)^2  // Want to reach target
     + R * control^2           // But smoothly
     + penalties               // While respecting limits
```

---

**Now you have MPC in C++! 🎉**

Compile it, run it, modify it, deploy it!
