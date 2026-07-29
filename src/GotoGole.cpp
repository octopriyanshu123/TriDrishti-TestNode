#include "i2w/impl.hpp"
#include "crawler_i2w_msgs/ui/joy.hpp"
#include "crawler_i2w_msgs/robot/cmd_vel.hpp"
#include "crawler_i2w_msgs/diff_drive_odometry.hpp"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <linux/joystick.h>
#include <utility>
#include <unistd.h>
#include <iostream>
#include <thread>
#include <atomic>
#include <cmath>
#include <algorithm>

std::atomic_bool running{true};
std::atomic<bool> shutdownRequested{false};

// Constants for control
// const double K_LINEAR = 5;
// const double K_ANGULAR = 10;
// const double MAX_LINEAR = 10;
// const double MAX_ANGULAR = 5;
// const double POSITION_TOLERANCE = 0.1;
// const double ANGLE_TOLERANCE = 0.1;

struct Pose
{
    double x;     // meters
    double y;     // meters
    double theta; // radians (orientation)
};

Pose goal_pose;

void Stop(int)
{
    running.store(false);
}

crawler_i2w_msgs::cmd_vel cmd_vel_;

float normalize(int16_t value, float max_output)
{
    return (static_cast<float>(value) / 32767.0f) * max_output;
}

class GoTOGole final : public i2w::SystemBase
{
public:
    std::atomic_bool goal_reached_{false};
    explicit GoTOGole(i2w::Config config)
        : i2w::SystemBase(std::move(config))
    {

        goal_reached_.store(false); // Initialize
    }

    i2w::Publisher<crawler_i2w_msgs::cmd_vel> publisher_{};

    bool goleReached = true;

    double prev_linear_cmd_ = 0.0;
    double prev_angular_cmd_ = 0.0;
    const double SMOOTHING_FACTOR = 1; // 0-1, higher = more smoothing

    void PublishVelocity(double linear, double angular)
    {
        crawler_i2w_msgs::cmd_vel cmd;
        cmd.linearVelocity = linear;
        cmd.angularVelocity = angular;
        publisher_.publish(cmd, static_cast<std::uint64_t>(runtime().clock().now().ns));
    }

    void StopRobot()
    {
        PublishVelocity(0.0, 0.0);
        std::cout << "Robot stopped!" << std::endl;
    }
    Pose current;
    Pose initPose;

    bool isMoving = false;
    std::thread motionThread;
    std::atomic<bool> motionRunning{false};

private:
    // Control constants
    const double K_LINEAR = 2.0;
    const double K_ANGULAR = 10.0;
    const double MAX_LINEAR = 10;
    const double MAX_ANGULAR = 5;
    const double POSITION_TOLERANCE = 0.05;
    const double ANGLE_TOLERANCE = 0.05;

    void MoveTo(double distance)
{
    Pose start = current;                 // Initial position
    bool forward = true;

    while (!shutdownRequested.load())
    {
        Pose pose = current;              // Local snapshot

        double target = start.x + (forward ? distance : -distance);

        if (forward)
        {
            if (pose.x < target)
            {
                std::cout << "Forward  Target: " << target
                          << " Current: " << pose.x << std::endl;

                PublishVelocity(10, 0.0);
            }
            else
            {
                StopRobot();

                forward = false;
                start = pose;             // New starting point
            }
        }
        else
        {
            if (pose.x > target)
            {
                std::cout << "Backward Target: " << target
                          << " Current: " << pose.x << std::endl;

                PublishVelocity(-10, 0.0);
            }
            else
            {
                StopRobot();

                forward = true;
                start = pose;             // New starting point
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    StopRobot();
    std::cout << "Thread Closed\n";
}

    void AdvancedGoToGoal(const Pose &current, const Pose &goal, crawler_i2w_msgs::cmd_vel &cmd)
    {
        // Calculate errors
        double dx = goal.x - current.x;
        double dy = goal.y - current.y;
        double distance = sqrt(dx * dx + dy * dy);
        double desired_theta = atan2(dy, dx);
        double angle_diff = desired_theta - current.theta;

        // Normalize angle to [-PI, PI]
        while (angle_diff > M_PI)
            angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI)
            angle_diff += 2 * M_PI;

        // Goal check
        if (distance < POSITION_TOLERANCE && fabs(angle_diff) < ANGLE_TOLERANCE)
        {
            cmd.linearVelocity = 0.0;
            cmd.angularVelocity = 0.0;
            goal_reached_.store(true);
            return;
        }

        // --- Advanced Control Strategy ---

        // 1. Pure pursuit: Look ahead distance
        double lookahead = distance * 0.3 + 0.1;
        if (lookahead > 0.5)
            lookahead = 0.5;

        double curvature = 0.0;
        if (distance > 0.01)
        {
            double alpha = atan2(dy, dx) - current.theta;
            curvature = 2.0 * sin(alpha) / lookahead;
        }

        // 2. Compute desired velocities
        double linear_desired = K_LINEAR * distance;
        double angular_desired = K_ANGULAR * angle_diff;

        // 3. Velocity scaling with smooth transitions
        double angle_scale = 1.0 - (fabs(angle_diff) / M_PI) * 0.5;
        if (angle_scale < 0.3)
            angle_scale = 0.3;
        if (angle_scale > 1.0)
            angle_scale = 1.0;

        double distance_scale = distance / 0.3;
        if (distance_scale > 1.0)
            distance_scale = 1.0;
        if (distance_scale < 0.1)
            distance_scale = 0.1;

        // 4. Apply scaling
        linear_desired *= angle_scale * distance_scale;

        // 5. Add feedforward term for better tracking
        if (distance > 0.2 && fabs(angle_diff) < 0.3)
        {
            double feedforward = 0.1 * cos(angle_diff);
            linear_desired += feedforward;
        }

        // 6. Manual Clamp linear velocity
        if (linear_desired < 0.0)
        {
            linear_desired = 0.0;
        }
        else if (linear_desired > MAX_LINEAR)
        {
            linear_desired = MAX_LINEAR;
        }

        // 7. Manual Clamp angular velocity
        if (angular_desired > MAX_ANGULAR)
        {
            angular_desired = MAX_ANGULAR;
        }
        else if (angular_desired < -MAX_ANGULAR)
        {
            angular_desired = -MAX_ANGULAR;
        }

        // 8. Apply smoothing (low-pass filter)
        cmd.linearVelocity = prev_linear_cmd_ + SMOOTHING_FACTOR * (linear_desired - prev_linear_cmd_);
        cmd.angularVelocity = prev_angular_cmd_ + SMOOTHING_FACTOR * (angular_desired - prev_angular_cmd_);

        // 9. Final clamping after smoothing
        if (cmd.linearVelocity < 0.0)
        {
            cmd.linearVelocity = 0.0;
        }
        else if (cmd.linearVelocity > MAX_LINEAR)
        {
            cmd.linearVelocity = MAX_LINEAR;
        }

        if (cmd.angularVelocity > MAX_ANGULAR)
        {
            cmd.angularVelocity = MAX_ANGULAR;
        }
        else if (cmd.angularVelocity < -MAX_ANGULAR)
        {
            cmd.angularVelocity = -MAX_ANGULAR;
        }

        // Store for next iteration
        prev_linear_cmd_ = cmd.linearVelocity;
        prev_angular_cmd_ = cmd.angularVelocity;

        // 10. Special handling: final alignment
        if (distance < 0.15)
        {
            double final_angle_diff = goal.theta - current.theta;
            while (final_angle_diff > M_PI)
                final_angle_diff -= 2 * M_PI;
            while (final_angle_diff < -M_PI)
                final_angle_diff += 2 * M_PI;

            if (fabs(final_angle_diff) > ANGLE_TOLERANCE)
            {
                cmd.linearVelocity = 0.0;

                double final_angular = K_ANGULAR * final_angle_diff;
                if (final_angular > MAX_ANGULAR)
                {
                    cmd.angularVelocity = MAX_ANGULAR;
                }
                else if (final_angular < -MAX_ANGULAR)
                {
                    cmd.angularVelocity = -MAX_ANGULAR;
                }
                else
                {
                    cmd.angularVelocity = final_angular;
                }
            }
            else
            {
                cmd.linearVelocity = 0.0;
                cmd.angularVelocity = 0.0;
                goal_reached_.store(true);
            }
        }
    }
   
    i2w::LifecycleResult OnSetup() override
    {
        // Subscribe to odometry
        i2w::SubscriptionOptions opts;
        opts.plane = i2w::EndpointPlane::Local;
        opts.reliability = i2w::Reliability::BestEffort;
        opts.queue_depth = 32;
        opts.overflow_policy = i2w::OverflowPolicy::DropOldest;

        auto subscription = runtime().subscribe<crawler_i2w_msgs::I2wDiffDriveOdometry>(
            "diff_drive/odometry",
            [this](const i2w::Sample<crawler_i2w_msgs::I2wDiffDriveOdometry> &sample)
            {
                // Get current pose from odometry

                current.x = sample.value.x_m;
                current.y = sample.value.y_m;
                current.theta = sample.value.yaw_rad; // Assuming theta is available

                // std::cout << std::fixed << std::setprecision(4)
                //           << "Current x: " << current.x
                //           << " y: " << current.y
                //           << " theta: " << current.theta << std::endl;

                // Calculate control command
                // crawler_i2w_msgs::cmd_vel cmd;

                // // AdvancedGoToGoal(current, goal_pose, cmd);
                // void MoveTo()

                // std::cout << "Cmd: linear=" << cmd.linearVelocity
                //           << " angular=" << cmd.angularVelocity << std::endl;

                // // Publish command
                // publisher_.publish(cmd, static_cast<std::uint64_t>(runtime().clock().now().ns));
            },
            opts);
        sub_ = std::move(subscription.value());

        // Setup publisher
        i2w::PublisherOptions cmdVelPubOpt;
        cmdVelPubOpt.plane = i2w::EndpointPlane::Local;
        auto publisher = runtime().advertise<crawler_i2w_msgs::cmd_vel>("/cmd_vel", cmdVelPubOpt);
        publisher_ = std::move(publisher.value());

        return i2w::Ok();
    }

    i2w::LifecycleResult OnTick() override
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        if (!isMoving)
        {
            isMoving = true;

            motionThread = std::thread(&GoTOGole::MoveTo, this, 0.5);
        }

        return i2w::Ok();
    }

    // void OnDispose() override;

    i2w::Subscription<crawler_i2w_msgs::I2wDiffDriveOdometry> sub_{};
};

#include <csignal>
bool firstGole = true;

void signalHandler(int)
{
    shutdownRequested.store(true, std::memory_order_relaxed);
}

int main()
{
    std::signal(SIGINT, signalHandler);
    i2w::Config joySubConfig;
    joySubConfig.node_name = "GOtoGole";
    joySubConfig.ns = "robot";
    joySubConfig.transport.network_profile_file = "/home/octo/TriDristi-ws/src/joyToCmdVel/config/config.json";

    GoTOGole gtg(joySubConfig);

    gtg.Setup();

    while (!shutdownRequested.load(std::memory_order_relaxed))
    {
        gtg.Tick();

        // if (firstGole || gtg.goleReached)
        // {
        //     gtg.goleReached = false;
        //     firstGole = false;
        //     std::cout << "Set Gole x" << std::endl;
        //     std::cin >> goal_pose.x;
        //     std::cout << "Set Gole y" << std::endl;
        //     std::cin >> goal_pose.y;
        //     std::cout << "Set Gole thera" << std::endl;
        //     std::cin >> goal_pose.theta;
        // }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    gtg.StopRobot();
    gtg.motionThread.join();
    return 0;
}