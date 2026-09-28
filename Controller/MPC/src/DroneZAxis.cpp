#include "SystemModel.cpp"
#include "MPC_Controller.cpp"
#include <iostream>
using namespace std;


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
        return 2;  // [z (hight in z ), vz (velocity in z )]
    }
    
    int get_control_size() const override {
        return 1;  // [F force]
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
