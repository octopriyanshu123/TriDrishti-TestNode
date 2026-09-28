
#include <memory>
#include <cmath>
#include <vector>

#pragma once

class MPC_Controller {
private:
    std::shared_ptr<SystemModel> model;
    double dt;
    int N;  // Prediction horizon
    
    // Cost function weights
    double Q;  // State tracking weight
    double R;  // Control effort weight
    
    // System properties
    int n_states;
    int n_controls;
    std::vector<double> u_min;
    std::vector<double> u_max;
    std::vector<double> x_min;
    std::vector<double> x_max;
    
    // Control sampling
    int n_samples;
    
public:
    MPC_Controller(std::shared_ptr<SystemModel> system_model, 
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
        
        // cout << "MPC_Controller initialized:" << endl;
        // cout << "  States: " << n_states << endl;
        // cout << "  Controls: " << n_controls << endl;
        // cout << "  Horizon: " << N << " steps" << endl;
        // cout << "  dt: " << dt << "s" << endl;
    }
    
    ~MPC_Controller() {}
    
    void set_weights(double Q_weight, double R_weight) {
        Q = Q_weight;
        R = R_weight;
        // cout << "Weights updated: Q=" << Q << ", R=" << R << endl;
    }
    
    std::vector<double> compute_control(const std::vector<double>& current_state, 
                                   const std::vector<double>& target_state) {
        /*
         * Compute optimal control using MPC
         */
        
        // Generate control candidates
        std::vector<std::vector<double>> candidates = generate_control_candidates();
        
        std::vector<double> best_control(n_controls, 0.0);
        double best_cost = std::numeric_limits<double>::infinity();
        
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
    std::vector<std::vector<double>> generate_control_candidates() {
        /*
         * Generate candidate control inputs to test
         */
        std::vector<std::vector<double>> candidates;
        
        if (n_controls == 1) {
            // 1D control: simple linspace
            for (int i = 0; i < n_samples; i++) {
                double u = u_min[0] + (u_max[0] - u_min[0]) * i / (n_samples - 1);
                candidates.push_back(std::vector<double>{u});
            }
        } 
        else if (n_controls == 2) {
            // 2D control: grid sampling
            int samples_per_dim = (int)sqrt(n_samples);
            for (int i = 0; i < samples_per_dim; i++) {
                double u1 = u_min[0] + (u_max[0] - u_min[0]) * i / (samples_per_dim - 1);
                for (int j = 0; j < samples_per_dim; j++) {
                    double u2 = u_min[1] + (u_max[1] - u_min[1]) * j / (samples_per_dim - 1);
                    candidates.push_back(std::vector<double>{u1, u2});
                }
            }
        }
        else {
            // Multi-dimensional: uniform sampling (simplified)
            for (int i = 0; i < n_samples; i++) {
                std::vector<double> control(n_controls);
                for (int j = 0; j < n_controls; j++) {
                    control[j] = u_min[j] + (u_max[j] - u_min[j]) * i / (n_samples - 1);
                }
                candidates.push_back(control);
            }
        }
        
        return candidates;
    }
    
    double evaluate_control(const std::vector<double>& initial_state,
                           const std::vector<double>& target_state,
                           const std::vector<double>& control) {
        /*
         * Evaluate cost of applying a control over prediction horizon
         */
        double total_cost = 0.0;
        std::vector<double> state = initial_state;
        
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
    
    double compute_constraint_penalty(const std::vector<double>& state,
                                     const std::vector<double>& control) {
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