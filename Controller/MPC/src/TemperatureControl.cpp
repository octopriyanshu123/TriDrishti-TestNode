#include "SystemModel.cpp"
#include "MPC_Controller.cpp"
#include <iostream>
using namespace std;

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
        // cout << "TemperatureSystem initialized:" << endl;
        // cout << "  Thermal mass: " << C << endl;
        // cout << "  Heat loss: " << k << endl;
        // cout << "  Ambient temp: " << T_ambient << "°C" << endl;
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



void run_temperature_control_simulation() {
    // cout << "\n" << string(70, '=') << endl;
    // cout << "TEMPERATURE CONTROL SIMULATION" << endl;
    // cout << string(70, '=') << endl << endl;
    
    // Create system and controller
    auto temp_system = make_shared<TemperatureSystem>(100.0, 5.0, 15.0);
    MPC_Controller mpc(temp_system, 1.0, 20);  // dt=1s, horizon=20
    mpc.set_weights(10.0, 0.01);
    
    // Initial state and target
    vector<double> state = {15.0};  // Room at ambient temp
    vector<double> target = {22.0};  // Want 22°CController/MPC/src/DroneZAxis.cpp
    
    // cout << "\nInitial: T=" << state[0] << "°C" << endl;
    // cout << "Target:  T=" << target[0] << "°C" << endl;
    // cout << "Ambient: T=15.0°C" << endl;
    // cout << "\n" << string(70, '-') << endl;
    // printf("%-10s %-12s %-12s %-10s\n", "Time[s]", "Temp[°C]", "Power[W]", "Error[°C]");
    // cout << string(70, '-') << endl;
    
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
            // cout << "\n✓ Reached target at t=" << time << "s" << endl;
            // printf("  Final temperature: %.2f°C\n", state[0]);
            // printf("  Final power: %.1fW\n", control[0]);
            break;
        }
    }
    
    // cout << string(70, '=') << endl;
}