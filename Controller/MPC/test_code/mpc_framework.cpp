#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <memory>


using namespace std;

class SystemModel {
public:
    virtual ~SystemModel() {}
    
    // Pure virtual functions (must be implemented by derived classes)
    virtual int get_state_size() const = 0;
    virtual int get_control_size() const = 0;
    
    virtual vector<double> dynamics(const vector<double>& state, 
                                    const vector<double>& control, 
                                    double dt) = 0;
    
    virtual void get_control_bounds(vector<double>& u_min, 
                                    vector<double>& u_max) const = 0;
    
    virtual void get_state_bounds(vector<double>& x_min, 
                                  vector<double>& x_max) const = 0;
};


// ============================================
// DERIVED CLASS: TemperatureSystem
// ============================================
class TemperatureSystem : public SystemModel {
private:
    double C;           // Thermal capacitance
    double k;           // Heat loss coefficient
    double T_ambient;   // Ambient temperature
    
public:
    TemperatureSystem(double thermal_mass = 1.0, 
                     double heat_loss = 0.1, 
                     double ambient_temp = 20.0)
        : C(thermal_mass), k(heat_loss), T_ambient(ambient_temp) {
        cout << "TemperatureSystem initialized:" << endl;
        cout << "  Thermal mass: " << C << endl;
        cout << "  Heat loss: " << k << endl;
        cout << "  Ambient temp: " << T_ambient << "°C" << endl;
    }
    
    ~TemperatureSystem() {}
    
    int get_state_size() const override {
        return 1;  // [T]
    }
    
    int get_control_size() const override {
        return 1;  // [P]
    }
    
    vector<double> dynamics(const vector<double>& state, 
                           const vector<double>& control, 
                           double dt) override {
        /*
         * Heat equation:
         * dT/dt = (P - k*(T - T_ambient)) / C
         */
        double T = state[0];
        double P = control[0];
        
        // Temperature change rate
        double dT_dt = (P - k * (T - T_ambient)) / C;
        
        // Update temperature
        double T_next = T + dT_dt * dt;
        
        return vector<double>{T_next};
    }
    
    void get_control_bounds(vector<double>& u_min, 
                           vector<double>& u_max) const override {
        // Heater power limits [W]
        u_min = vector<double>{0.0};
        u_max = vector<double>{1000.0};
    }
    
    void get_state_bounds(vector<double>& x_min, 
                         vector<double>& x_max) const override {
        // Temperature limits [°C]
        x_min = vector<double>{15.0};
        x_max = vector<double>{30.0};
    }
};


// ============================================
// MPC CONTROLLER
// ============================================
class MPC_Controller {
private:
    shared_ptr<SystemModel> model;
    double dt;
    int N;  // Prediction horizon
    
    // Cost function weights
    double Q;  // State tracking weight
    double R;  // Control effort weight
    
    // System properties
    int n_states;
    int n_controls;
    vector<double> u_min;
    vector<double> u_max;
    vector<double> x_min;
    vector<double> x_max;
    
    // Control sampling
    int n_samples;
    
public:
    MPC_Controller(shared_ptr<SystemModel> system_model, 
                   double timestep, 
                   int horizon = 10)
        : model(system_model), dt(timestep), N(horizon) {
        
        // Default weights
        Q = 100.0;
        R = 0.1;
        
        // Get system properties
        n_states = model->get_state_size();
        n_controls = model->get_control_size();
        model->get_control_bounds(u_min, u_max);
        model->get_state_bounds(x_min, x_max);
        
        // Control sampling
        n_samples = 20;
        
        cout << "MPC_Controller initialized:" << endl;
        cout << "  States: " << n_states << endl;
        cout << "  Controls: " << n_controls << endl;
        cout << "  Horizon: " << N << " steps" << endl;
        cout << "  dt: " << dt << "s" << endl;
    }
    
    ~MPC_Controller() {}
    
    void set_weights(double Q_weight, double R_weight) {
        Q = Q_weight;
        R = R_weight;
        cout << "Weights updated: Q=" << Q << ", R=" << R << endl;
    }
    
    vector<double> compute_control(const vector<double>& current_state, 
                                   const vector<double>& target_state) {
        /*
         * Compute optimal control using MPC
         */
        
        // Generate control candidates
        vector<vector<double>> candidates = generate_control_candidates();
        
        vector<double> best_control(n_controls, 0.0);
        double best_cost = numeric_limits<double>::infinity();
        
        // Evaluate each control candidate
        for (const auto& control : candidates) {
            double cost = evaluate_control(current_state, target_state, control);
            
            if (cost < best_cost) {
                best_cost = cost;
                best_control = control;
            }
        }
        
        return best_control;
    }
    
private:
    vector<vector<double>> generate_control_candidates() {
        /*
         * Generate candidate control inputs to test
         */
        vector<vector<double>> candidates;
        
        if (n_controls == 1) {
            // 1D control: simple linspace
            for (int i = 0; i < n_samples; i++) {
                double u = u_min[0] + (u_max[0] - u_min[0]) * i / (n_samples - 1);
                candidates.push_back(vector<double>{u});
            }
        } 
        else if (n_controls == 2) {
            // 2D control: grid sampling
            int samples_per_dim = (int)sqrt(n_samples);
            for (int i = 0; i < samples_per_dim; i++) {
                double u1 = u_min[0] + (u_max[0] - u_min[0]) * i / (samples_per_dim - 1);
                for (int j = 0; j < samples_per_dim; j++) {
                    double u2 = u_min[1] + (u_max[1] - u_min[1]) * j / (samples_per_dim - 1);
                    candidates.push_back(vector<double>{u1, u2});
                }
            }
        }
        else {
            // Multi-dimensional: uniform sampling (simplified)
            for (int i = 0; i < n_samples; i++) {
                vector<double> control(n_controls);
                for (int j = 0; j < n_controls; j++) {
                    control[j] = u_min[j] + (u_max[j] - u_min[j]) * i / (n_samples - 1);
                }
                candidates.push_back(control);
            }
        }
        
        return candidates;
    }
    
    double evaluate_control(const vector<double>& initial_state,
                           const vector<double>& target_state,
                           const vector<double>& control) {
        /*
         * Evaluate cost of applying a control over prediction horizon
         */
        double total_cost = 0.0;
        vector<double> state = initial_state;
        
        // Simulate forward for N steps
        for (int step = 0; step < N; step++) {
            // Predict next state
            state = model->dynamics(state, control, dt);
            
            // Calculate cost components
            double state_cost = 0.0;
            for (int i = 0; i < n_states; i++) {
                double error = state[i] - target_state[i];
                state_cost += Q * error * error;
            }
            
            double control_cost = 0.0;
            for (int i = 0; i < n_controls; i++) {
                control_cost += R * control[i] * control[i];
            }
            
            // Add constraint penalties
            double constraint_penalty = compute_constraint_penalty(state, control);
            
            // Total step cost
            double step_cost = state_cost + control_cost + constraint_penalty;
            total_cost += step_cost;
        }
        
        return total_cost;
    }
    
    double compute_constraint_penalty(const vector<double>& state,
                                     const vector<double>& control) {
        /*
         * Add penalty for constraint violations
         */
        double penalty = 0.0;
        
        // State constraints
        for (int i = 0; i < n_states; i++) {
            if (state[i] < x_min[i]) {
                double violation = x_min[i] - state[i];
                penalty += 1000.0 * violation * violation;
            }
            if (state[i] > x_max[i]) {
                double violation = state[i] - x_max[i];
                penalty += 1000.0 * violation * violation;
            }
        }
        
        // Control constraints
        for (int i = 0; i < n_controls; i++) {
            if (control[i] < u_min[i]) {
                double violation = u_min[i] - control[i];
                penalty += 1000.0 * violation * violation;
            }
            if (control[i] > u_max[i]) {
                double violation = control[i] - u_max[i];
                penalty += 1000.0 * violation * violation;
            }
        }
        
        return penalty;
    }
};


// ============================================
// SIMULATION FUNCTIONS
// ============================================
void run_temperature_control_simulation() {
    cout << "\n" << string(70, '=') << endl;
    cout << "TEMPERATURE CONTROL SIMULATION" << endl;
    cout << string(70, '=') << endl << endl;
    
    // Create system and controller
    auto temp_system = make_shared<TemperatureSystem>(100.0, 5.0, 15.0);
    MPC_Controller mpc(temp_system, 1.0, 20);  // dt=1s, horizon=20
    mpc.set_weights(10.0, 0.01);
    
    // Initial state and target
    vector<double> state = {15.0};  // Room at ambient temp
    vector<double> target = {22.0};  // Want 22°C
    
    cout << "\nInitial: T=" << state[0] << "°C" << endl;
    cout << "Target:  T=" << target[0] << "°C" << endl;
    cout << "Ambient: T=15.0°C" << endl;
    cout << "\n" << string(70, '-') << endl;
    printf("%-10s %-12s %-12s %-10s\n", "Time[s]", "Temp[°C]", "Power[W]", "Error[°C]");
    cout << string(70, '-') << endl;
    
    // Simulation
    for (int step = 0; step < 100; step++) {
        double time = step * 1.0;
        
        // Compute optimal control
        vector<double> control = mpc.compute_control(state, target);
        
        // Print every 5 seconds
        if (step % 5 == 0) {
            double error = abs(state[0] - target[0]);
            printf("%-10.0f %-12.2f %-12.1f %-10.3f\n", 
                   time, state[0], control[0], error);
        }
        
        // Update state
        state = temp_system->dynamics(state, control, 1.0);
        
        // Check if reached target
        if (abs(state[0] - target[0]) < 0.5) {
            cout << "\n✓ Reached target at t=" << time << "s" << endl;
            printf("  Final temperature: %.2f°C\n", state[0]);
            printf("  Final power: %.1fW\n", control[0]);
            break;
        }
    }
    
    cout << string(70, '=') << endl;
}


// ============================================
// ADDITIONAL EXAMPLE: DroneZAxis
// ============================================
class DroneZAxis : public SystemModel {
private:
    double m;  // mass
    double g;  // gravity
    
public:
    DroneZAxis(double mass = 1.0, double gravity = 9.81)
        : m(mass), g(gravity) {
        cout << "DroneZAxis initialized:" << endl;
        cout << "  Mass: " << m << " kg" << endl;
        cout << "  Gravity: " << g << " m/s²" << endl;
    }
    
    ~DroneZAxis() {}
    
    int get_state_size() const override {
        return 2;  // [z, vz]
    }
    
    int get_control_size() const override {
        return 1;  // [F]
    }
    
    vector<double> dynamics(const vector<double>& state,
                           const vector<double>& control,
                           double dt) override {
        /*
         * Physics:
         * az = F/m - g
         * vz_next = vz + az*dt
         * z_next = z + vz*dt
         */
        double z = state[0];
        double vz = state[1];
        double F = control[0];
        
        // Acceleration
        double az = F / m - g;
        
        // Update velocity and position
        double vz_next = vz + az * dt;
        double z_next = z + vz * dt;
        
        return vector<double>{z_next, vz_next};
    }
    
    void get_control_bounds(vector<double>& u_min,
                           vector<double>& u_max) const override {
        // Thrust limits [N]
        u_min = vector<double>{0.0};
        u_max = vector<double>{25.0};
    }
    
    void get_state_bounds(vector<double>& x_min,
                         vector<double>& x_max) const override {
        // Position and velocity limits
        x_min = vector<double>{0.0, -5.0};    // [z_min, vz_min]
        x_max = vector<double>{30.0, 5.0};    // [z_max, vz_max]
    }
};


void run_drone_simulation() {
    cout << "\n" << string(70, '=') << endl;
    cout << "DRONE Z-AXIS CONTROL SIMULATION" << endl;
    cout << string(70, '=') << endl << endl;
    
    // Create system and controller
    auto drone = make_shared<DroneZAxis>(1.0, 9.81);
    MPC_Controller mpc(drone, 0.1, 10);  // dt=0.1s, horizon=10
    mpc.set_weights(100.0, 0.1);
    
    // Initial state and target
    vector<double> state = {0.0, 0.0};  // [z=0m, vz=0m/s]
    vector<double> target = {10.0, 0.0};  // [z=10m, vz=0m/s]
    
    cout << "\nInitial: z=" << state[0] << "m, vz=" << state[1] << "m/s" << endl;
    cout << "Target:  z=" << target[0] << "m, vz=" << target[1] << "m/s" << endl;
    cout << "\n" << string(70, '-') << endl;
    printf("%-10s %-12s %-12s %-12s\n", "Time[s]", "Z[m]", "Vz[m/s]", "Thrust[N]");
    cout << string(70, '-') << endl;
    
    // Simulation
    for (int step = 0; step < 100; step++) {
        double time = step * 0.1;
        
        // Compute optimal control
        vector<double> control = mpc.compute_control(state, target);
        
        // Print every 0.5s
        if (step % 5 == 0) {
            printf("%-10.1f %-12.3f %-12.3f %-12.3f\n",
                   time, state[0], state[1], control[0]);
        }
        
        // Update state
        state = drone->dynamics(state, control, 0.1);
        
        // Check if reached target
        double pos_error = abs(state[0] - target[0]);
        double vel_error = abs(state[1] - target[1]);
        if (pos_error < 0.1 && vel_error < 0.1) {
            cout << "\n✓ Reached target at t=" << time << "s" << endl;
            printf("  Final position: %.3fm\n", state[0]);
            printf("  Final velocity: %.3fm/s\n", state[1]);
            break;
        }
    }
    
    cout << string(70, '=') << endl;
}


// ============================================
// MAIN
// ============================================
int main() {
    run_temperature_control_simulation();
    run_drone_simulation();
    return 0;
}
