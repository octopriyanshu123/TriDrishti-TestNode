#include <vector>
#pragma once

class SystemModel {
public:
    virtual ~SystemModel() {}
    
    // Pure virtual functions (must be implemented by derived classes)
    virtual int get_state_size() const = 0;
    virtual int get_control_size() const = 0;
    
    virtual std::vector<double> dynamics(const std::vector<double>& state, 
                                    const std::vector<double>& control, 
                                    double dt) = 0;
    
    virtual void get_control_bounds(std::vector<double>& u_min, 
                                    std::vector<double>& u_max) const = 0;
    
    virtual void get_state_bounds(std::vector<double>& x_min, 
                                  std::vector<double>& x_max) const = 0;
};