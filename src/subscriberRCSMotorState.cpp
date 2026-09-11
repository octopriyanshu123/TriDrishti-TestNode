#include "i2w/impl.hpp"

#include "crawler_i2w_msgs/robot/motor_status.hpp"
#include "crawler_i2w_msgs/robot/robot_coordinate_system.hpp"

#include <atomic>
#include <cstdint>
#include <iostream>
#include <thread>
#include <chrono>
#include <utility>
#include <signal.h>


std::atomic_bool running{true};

void Stop(int)
{
    running.store(false);
}

class RobotDataSubscriber final : public i2w::SystemBase
{
public:
    explicit RobotDataSubscriber(i2w::Config config)
        : i2w::SystemBase(std::move(config))
    {
    }

private:

    i2w::LifecycleResult OnSetup()
    {
        // ============================================================
        // MotorState subscriber
        // ============================================================

        i2w::SubscriptionOptions motorOpts;

        motorOpts.plane = i2w::EndpointPlane::Local;
        motorOpts.reliability = i2w::Reliability::BestEffort;
        motorOpts.queue_depth = 32;
        motorOpts.overflow_policy =
            i2w::OverflowPolicy::DropOldest;

        auto motorSubscription =
            runtime().subscribe<crawler_i2w_msgs::MotorState>(
                "/motor_status",

                [this](
                    const i2w::Sample<crawler_i2w_msgs::MotorState>& sample)
                {
                    const auto& motorState = sample.value;

                    std::cout
                        << "\n========== MOTOR STATE ==========\n"
                        << "Timestamp       : "
                        << motorState.timestamp_ns << '\n'

                        << "Left Velocity    : "
                        << motorState.left.velocity_rad_s
                        << " rad/s\n"

                        << "Left Torque      : "
                        << motorState.left.torque_
                        << '\n'

                        << "Left Temperature : "
                        << motorState.left.temperature_c
                        << " C\n"

                        << "Left Current     : "
                        << motorState.left.current_a
                        << " A\n"

                        << "Left Brake       : "
                        << motorState.left.is_brake_released
                        << '\n'

                        << "Left Enabled     : "
                        << motorState.left.enabled
                        << '\n'

                        << "Left Fault       : 0x"
                        << std::hex
                        << static_cast<std::uint16_t>(
                               motorState.left.fault)
                        << std::dec
                        << '\n'

                        << "Right Velocity   : "
                        << motorState.right.velocity_rad_s
                        << " rad/s\n"

                        << "Right Torque     : "
                        << motorState.right.torque_
                        << '\n'

                        << "Right Temperature: "
                        << motorState.right.temperature_c
                        << " C\n"

                        << "Right Current    : "
                        << motorState.right.current_a
                        << " A\n"

                        << "Right Brake      : "
                        << motorState.right.is_brake_released
                        << '\n'

                        << "Right Enabled    : "
                        << motorState.right.enabled
                        << '\n'

                        << "Right Fault      : 0x"
                        << std::hex
                        << static_cast<std::uint16_t>(
                               motorState.right.fault)
                        << std::dec
                        << '\n'

                        << "=================================\n";
                },

                motorOpts);

        if (!motorSubscription.has_value())
        {
            std::cerr
                << "Failed to subscribe to /motor_status\n";

            return i2w::Fail();
        }

        motorSub_ = std::move(motorSubscription.value());


        // ============================================================
        // Robot Coordinate System subscriber
        // ============================================================

        i2w::SubscriptionOptions rcsOpts;

        rcsOpts.plane = i2w::EndpointPlane::Local;
        rcsOpts.reliability = i2w::Reliability::BestEffort;
        rcsOpts.queue_depth = 32;
        rcsOpts.overflow_policy =
            i2w::OverflowPolicy::DropOldest;

        auto rcsSubscription =
            runtime().subscribe<
                crawler_i2w_msgs::RobotCoordinateSystem>(
                "/rcs",

                [this](
                    const i2w::Sample<
                        crawler_i2w_msgs::RobotCoordinateSystem>& sample)
                {
                    const auto& rcs = sample.value;

                    std::cout
                        << "\n============ RCS ================\n"
                        << "Timestamp : "
                        << rcs.timestamp
                        << '\n'

                        << "Radial    : "
                        << rcs.radial
                        << " deg\n"

                        << "Altitude  : "
                        << rcs.altitude
                        << '\n'

                        << "Bearing   : "
                        << rcs.bearing
                        << " deg\n"

                        << "=================================\n";
                },

                rcsOpts);

        if (!rcsSubscription.has_value())
        {
            std::cerr
                << "Failed to subscribe to /rcs\n";

            return i2w::Fail();
        }

        rcsSub_ = std::move(rcsSubscription.value());

        std::cout
            << "RobotDataSubscriber setup successful\n"
            << "Subscribed to:\n"
            << "  /motor_status\n"
            << "  /rcs\n";

        return i2w::Ok();
    }


    i2w::LifecycleResult OnTick()
    {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(1));

        return i2w::Ok();
    }


private:

    i2w::Subscription<crawler_i2w_msgs::MotorState>
        motorSub_{};

    i2w::Subscription<crawler_i2w_msgs::RobotCoordinateSystem>
        rcsSub_{};
};

#include <signal.h>
int main()
{
    signal(SIGINT, Stop);
    signal(SIGTERM, Stop);

    i2w::Config config;

    config.node_name = "robot_data_subscriber";
    config.ns = "";

    RobotDataSubscriber subscriber(config);

    auto setupResult = subscriber.Setup();

    // if (!setupResult)
    // {
    //     std::cerr
    //         << "Failed to setup RobotDataSubscriber\n";

    //     return 1;
    // }

    while (running.load())
    {
        subscriber.Tick();

        std::this_thread::sleep_for(
            std::chrono::milliseconds(1));
    }

    std::cout
        << "RobotDataSubscriber stopped\n";

    return 0;
}