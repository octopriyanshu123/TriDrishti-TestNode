#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>

struct PIDConfig
{
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;

    // Limits on the final controller output (actuator saturation).
    double out_min = -std::numeric_limits<double>::infinity();
    double out_max =  std::numeric_limits<double>::infinity();

    // Limits on the integral *contribution* (ki * integral), for anti-windup.
    double i_min = -std::numeric_limits<double>::infinity();
    double i_max =  std::numeric_limits<double>::infinity();

    // Derivative low-pass filter time constant in seconds (0 = no filtering).
    double d_filter_tau = 0.0;
};

class PID
{
public:
    explicit PID(const PIDConfig& cfg) : cfg_(cfg)
    {
        if (cfg_.out_min > cfg_.out_max)
            throw std::invalid_argument("PID: out_min must be <= out_max");
        if (cfg_.i_min > cfg_.i_max)
            throw std::invalid_argument("PID: i_min must be <= i_max");
        if (cfg_.d_filter_tau < 0.0)
            throw std::invalid_argument("PID: d_filter_tau must be >= 0");
    }

    double compute(double setpoint, double measurement, double dt)
    {
        if (dt <= 0.0)
            throw std::invalid_argument("PID: dt must be > 0");

        const double error = setpoint - measurement;

        // Proportional
        const double p_term = cfg_.kp * error;

        // Derivative on measurement (avoids "derivative kick" on setpoint changes),
        // with an optional first-order low-pass filter to suppress noise.
        double d_term = 0.0;
        if (initialized_)
        {
            const double raw_d = -(measurement - prev_measurement_) / dt;
            const double alpha = (cfg_.d_filter_tau > 0.0)
                                     ? dt / (cfg_.d_filter_tau + dt)
                                     : 1.0;
            d_filtered_ += alpha * (raw_d - d_filtered_);
            d_term = cfg_.kd * d_filtered_;
        }
        prev_measurement_ = measurement;
        initialized_ = true;

        // Integral with conditional-integration anti-windup:
        // only accumulate if it won't push an already saturated output further.
        const double candidate_i = std::clamp(i_term_ + cfg_.ki * error * dt,
                                              cfg_.i_min, cfg_.i_max);
        const double unsat = p_term + candidate_i + d_term;

        const bool saturating_high = unsat > cfg_.out_max && error > 0.0;
        const bool saturating_low  = unsat < cfg_.out_min && error < 0.0;
        if (!saturating_high && !saturating_low)
            i_term_ = candidate_i;

        const double output = p_term + i_term_ + d_term;
        return std::clamp(output, cfg_.out_min, cfg_.out_max);
    }

    void reset()
    {
        i_term_ = 0.0;
        d_filtered_ = 0.0;
        prev_measurement_ = 0.0;
        initialized_ = false;
    }

    double integralTerm() const { return i_term_; }

private:
    PIDConfig cfg_;
    double i_term_ = 0.0;           // Stored as ki * integral, so gain changes don't cause jumps
    double d_filtered_ = 0.0;
    double prev_measurement_ = 0.0;
    bool initialized_ = false;
};




int main()
{
    PIDConfig cfg;
    cfg.kp = 0.50;
    cfg.ki = 0.1;
    cfg.kd = 0.1;

    cfg.out_min = -500.0;
    cfg.out_max =  500.0;
    cfg.i_min = -10.0;
    cfg.i_max =  10.0;
    cfg.d_filter_tau = 0.02;

    PID pid(cfg);

    const double setpoint = 0.0;
    double measurement = 1000.0;
    const double dt = 0.01;
    const int steps = 1000;           // 10 seconds of simulated time

    std::cout << std::fixed << std::setprecision(4);
    std::cout << std::setw(8) << "t [s]"
              << std::setw(14) << "measurement"
              << std::setw(14) << "control" << '\n';

    for (int i = 0; i <= steps; ++i)
    {
        const double control = pid.compute(setpoint, measurement, dt);

        if (i % 50 == 0)  // print every 0.5 s
        {
            std::cout << std::setw(8) << i * dt
                      << std::setw(14) << measurement
                      << std::setw(14) << control << '\n';
        }

        // Simple integrating plant: control drives the rate of change.
        measurement += control * dt;
    }
}