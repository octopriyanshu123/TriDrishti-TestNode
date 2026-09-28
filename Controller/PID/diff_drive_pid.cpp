#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

constexpr double PI = 3.14159265358979323846;

inline double deg2rad(double d) { return d * PI / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / PI; }

// Wrap angle to [-pi, pi]
inline double wrapAngle(double a)
{
    return std::atan2(std::sin(a), std::cos(a));
}

// ============================================================
// PID controller
// ============================================================
struct PIDConfig
{
    double kp = 0.0, ki = 0.0, kd = 0.0;
    double out_min = -std::numeric_limits<double>::infinity();
    double out_max =  std::numeric_limits<double>::infinity();
    double i_min   = -std::numeric_limits<double>::infinity();
    double i_max   =  std::numeric_limits<double>::infinity();
    double d_filter_tau = 0.0;   // derivative low-pass filter [s]
    bool   angular = false;      // wrap errors to [-pi, pi]
};

class PID
{
public:
    explicit PID(const PIDConfig& cfg) : cfg_(cfg) {}

    double compute(double setpoint, double measurement, double dt)
    {
        if (dt <= 0.0) throw std::invalid_argument("PID: dt must be > 0");

        double error = setpoint - measurement;
        if (cfg_.angular) error = wrapAngle(error);

        // P
        const double p = cfg_.kp * error;

        // D (on measurement, filtered)
        double d = 0.0;
        if (initialized_)
        {
            double dm = measurement - prev_meas_;
            if (cfg_.angular) dm = wrapAngle(dm);
            const double raw = -dm / dt;
            const double alpha = cfg_.d_filter_tau > 0.0
                                     ? dt / (cfg_.d_filter_tau + dt) : 1.0;
            d_filt_ += alpha * (raw - d_filt_);
            d = cfg_.kd * d_filt_;
        }
        prev_meas_ = measurement;
        initialized_ = true;

        // I (clamped + conditional integration anti-windup)
        const double cand = std::clamp(i_ + cfg_.ki * error * dt,
                                       cfg_.i_min, cfg_.i_max);
        const double unsat = p + cand + d;
        if (!((unsat > cfg_.out_max && error > 0) ||
              (unsat < cfg_.out_min && error < 0)))
            i_ = cand;

        return std::clamp(p + i_ + d, cfg_.out_min, cfg_.out_max);
    }

    void reset() { i_ = d_filt_ = prev_meas_ = 0.0; initialized_ = false; }

private:
    PIDConfig cfg_;
    double i_ = 0.0, d_filt_ = 0.0, prev_meas_ = 0.0;
    bool initialized_ = false;
};

// ============================================================
// Differential-drive robot environment
// ============================================================
struct RobotParams
{
    double wheel_radius     = 0.05;  // R [m]
    double wheel_separation = 0.30;  // L [m]
    double max_wheel_speed  = 10.0;  // [rad/s]
    double motor_tau        = 0.05;  // motor response time constant [s]
};

struct RobotState
{
    double x = 0.0, y = 0.0, theta = 0.0;  // pose [m, m, rad]
    double wl = 0.0, wr = 0.0;             // actual wheel speeds [rad/s]
};

class DiffDriveRobot
{
public:
    explicit DiffDriveRobot(const RobotParams& p) : p_(p) {}

    // Commands are wheel angular velocities [rad/s]
    void step(double wl_cmd, double wr_cmd, double dt)
    {
        wl_cmd = std::clamp(wl_cmd, -p_.max_wheel_speed, p_.max_wheel_speed);
        wr_cmd = std::clamp(wr_cmd, -p_.max_wheel_speed, p_.max_wheel_speed);

        // First-order motor dynamics
        const double a = dt / (p_.motor_tau + dt);
        s_.wl += a * (wl_cmd - s_.wl);
        s_.wr += a * (wr_cmd - s_.wr);

        // Forward kinematics
        const double v = p_.wheel_radius * (s_.wr + s_.wl) / 2.0;
        const double w = p_.wheel_radius * (s_.wr - s_.wl) / p_.wheel_separation;

        s_.x += v * std::cos(s_.theta) * dt;
        s_.y += v * std::sin(s_.theta) * dt;
        s_.theta = wrapAngle(s_.theta + w * dt);
    }

    // Inverse kinematics: body (v, w) -> wheel speeds
    void inverse(double v, double w, double& wl, double& wr) const
    {
        wl = (v - w * p_.wheel_separation / 2.0) / p_.wheel_radius;
        wr = (v + w * p_.wheel_separation / 2.0) / p_.wheel_radius;
    }

    // Max achievable yaw rate when turning in place
    double maxYawRate() const
    {
        return 2.0 * p_.wheel_radius * p_.max_wheel_speed / p_.wheel_separation;
    }

    const RobotState& state() const { return s_; }

private:
    RobotParams p_;
    RobotState s_;
};

// ============================================================
// Main: reach target, then ask user for next target
// ============================================================
int main()
{
    RobotParams rp;
    rp.wheel_radius     = 0.05;   // 5 cm
    rp.wheel_separation = 0.30;   // 30 cm
    DiffDriveRobot robot(rp);

    PIDConfig cfg;
    cfg.kp = 4.0;
    cfg.ki = 0.0;    // heading is already an integrator; raise (e.g. 0.1) only if you have steady disturbances
    cfg.kd = 0.3;
    cfg.out_min = -robot.maxYawRate();   // output = desired yaw rate [rad/s]
    cfg.out_max =  robot.maxYawRate();
    cfg.i_min = -0.3;
    cfg.i_max =  0.3;
    cfg.d_filter_tau = 0.02;
    cfg.angular = true;
    PID pid(cfg);

    const double v  = 0.0;               // forward speed (0 = turn in place)
    const double dt = 0.01;              // 100 Hz

    // "Reached" = within tolerance and nearly stopped, held for settle_time
    const double tol_deg     = 0.5;
    const double rate_tol    = 0.02;     // [rad/s]
    const double settle_time = 0.3;      // [s]
    const double timeout     = 10.0;     // [s] give up waiting per target

    double target_deg = 90.0;            // first target
    double t = 0.0;

    std::cout << std::fixed << std::setprecision(3);

    while (true)
    {
        const double target = wrapAngle(deg2rad(target_deg));
        std::cout << "\n--- Target: " << target_deg << " deg ---\n"
                  << std::setw(8)  << "t[s]"
                  << std::setw(12) << "heading[d]"
                  << std::setw(12) << "error[d]"
                  << std::setw(10) << "w[r/s]" << '\n';

        double settled = 0.0, elapsed = 0.0;
        int step = 0;

        while (settled < settle_time && elapsed < timeout)
        {
            const double heading = robot.state().theta;          // measurement
            const double w_cmd   = pid.compute(target, heading, dt);

            double wl, wr;
            robot.inverse(v, w_cmd, wl, wr);
            robot.step(wl, wr, dt);

            const double err_deg = rad2deg(wrapAngle(target - heading));
            if (std::abs(err_deg) < tol_deg && std::abs(w_cmd) < rate_tol)
                settled += dt;
            else
                settled = 0.0;

            if (step % 10 == 0)   // print every 0.1 s
                std::cout << std::setw(8)  << t
                          << std::setw(12) << rad2deg(heading)
                          << std::setw(12) << err_deg
                          << std::setw(10) << w_cmd << '\n';

            t += dt; elapsed += dt; ++step;
        }

        if (settled >= settle_time)
            std::cout << "Reached " << rad2deg(robot.state().theta) << " deg\n";
        else
            std::cout << "Timeout at " << rad2deg(robot.state().theta) << " deg\n";

        // Ask for next target
        std::cout << "\nEnter new target heading in degrees (q to quit): ";
        std::string input;
        if (!(std::cin >> input) || input == "q" || input == "Q") break;

        try
        {
            target_deg = std::stod(input);
        }
        catch (const std::exception&)
        {
            std::cout << "Invalid number, keeping " << target_deg << " deg\n";
            continue;   // re-runs the same target (already reached, exits quickly)
        }
    }

    std::cout << "Bye.\n";
}