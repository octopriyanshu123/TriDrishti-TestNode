#include "i2w/impl.hpp"
#include "crawler_i2w_msgs/robot/motor_status.hpp"

#include <cstdint>
#include <iostream>
#include <random>
#include <thread>
#include <chrono>
#include <atomic>
#include <csignal>
#include <utility>

std::atomic_bool running{true};

void Stop(int)
{
    running.store(false);
}

class RandomMotorStatePublisher final : public i2w::SystemBase
{
public:
    explicit RandomMotorStatePublisher(i2w::Config config)
        : i2w::SystemBase(std::move(config))
    {
    }

private:
    i2w::LifecycleResult OnSetup()
    {
        i2w::PublisherOptions publisherOptions;

        publisherOptions.plane = i2w::EndpointPlane::Local;

        auto publisher =
            runtime().advertise<crawler_i2w_msgs::MotorState>(
                "/motor_status",
                publisherOptions);

        if (!publisher.has_value())
        {
            std::cerr << "[MotorStatePublisher] Failed to create publisher\n";
            return i2w::Fail();
        }

        motorStatePublisher_ = std::move(publisher.value());

        std::cout
            << "[MotorStatePublisher] Publisher created: /motor_status\n";

        return i2w::Ok();
    }

    i2w::LifecycleResult OnTick()
    {
        publishRandomMotorState();

        // 100 Hz
        std::this_thread::sleep_for(
            std::chrono::milliseconds(10));

        return i2w::Ok();
    }

    void publishRandomMotorState()
    {
        static std::random_device rd;
        static std::mt19937 gen(rd());

        // Random values
        static std::uniform_real_distribution<double>
            velocityDist(-10.0, 10.0);

        static std::uniform_real_distribution<double>
            torqueDist(0.0, 5.0);

        static std::uniform_real_distribution<double>
            temperatureDist(25.0, 80.0);

        static std::uniform_real_distribution<double>
            currentDist(0.0, 15.0);

        static std::bernoulli_distribution
            boolDist(0.5);

        // Generate number from 0 to 9
        static std::uniform_int_distribution<int>
            errorDist(0, 9);

        crawler_i2w_msgs::MotorState motorState{};

        // Timestamp
        motorState.timestamp_ns =
            static_cast<std::uint64_t>(
                runtime().clock().now().ns);

        // ------------------------------------------------
        // LEFT MOTOR
        // ------------------------------------------------

        motorState.left.velocity_rad_s =
            velocityDist(gen);

        motorState.left.torque_ =
            torqueDist(gen);

        motorState.left.temperature_c =
            temperatureDist(gen);

        motorState.left.current_a =
            currentDist(gen);

        motorState.left.is_brake_released =
            boolDist(gen);

        motorState.left.enabled =
            boolDist(gen);

        // ------------------------------------------------
        // RIGHT MOTOR
        // ------------------------------------------------

        motorState.right.velocity_rad_s =
            velocityDist(gen);

        motorState.right.torque_ =
            torqueDist(gen);

        motorState.right.temperature_c =
            temperatureDist(gen);

        motorState.right.current_a =
            currentDist(gen);

        motorState.right.is_brake_released =
            boolDist(gen);

        motorState.right.enabled =
            boolDist(gen);

        // ------------------------------------------------
        // MOTOR ERRORS
        // ------------------------------------------------

        motorState.left.fault =
            crawler_i2w_msgs::ErrorCode::NO_ERROR;

        motorState.right.fault =
            crawler_i2w_msgs::ErrorCode::NO_ERROR;

        // Occasionally generate an error
        if (errorDist(gen) == 0)
        {
            motorState.left.fault =
                crawler_i2w_msgs::ErrorCode::OVERTEMPERATURE;
        }

        if (errorDist(gen) == 0)
        {
            motorState.right.fault =
                crawler_i2w_msgs::ErrorCode::OVERCURRENT;
        }

        // ------------------------------------------------
        // PUBLISH
        // ------------------------------------------------

        (void)motorStatePublisher_.publish(
            motorState,
            static_cast<std::int64_t>(
                motorState.timestamp_ns));

        // Optional terminal output
        std::cout
            << "[MotorState]"
            << " L_Vel=" << motorState.left.velocity_rad_s
            << " R_Vel=" << motorState.right.velocity_rad_s
            << " L_Temp=" << motorState.left.temperature_c
            << " R_Temp=" << motorState.right.temperature_c
            << " L_Current=" << motorState.left.current_a
            << " R_Current=" << motorState.right.current_a
            << '\n';
    }

private:
    i2w::Publisher<crawler_i2w_msgs::MotorState>
        motorStatePublisher_{};
};

int main()
{
    // --------------------------------------------
    // Signal handling
    // --------------------------------------------

    std::signal(SIGINT, Stop);
    std::signal(SIGTERM, Stop);

    // --------------------------------------------
    // i2w configuration
    // --------------------------------------------

    i2w::Config motorStateConfig;

    motorStateConfig.node_name = "random_motor_state";
    motorStateConfig.ns = "";

    // If network transport is required:
    //
    // motorStateConfig.transport.network_profile_file =
    //     "./config/ecal-network-udp.yaml";

    // --------------------------------------------
    // Create node
    // --------------------------------------------

    RandomMotorStatePublisher motorStatePublisher(
        motorStateConfig);

    // --------------------------------------------
    // Setup
    // --------------------------------------------

   motorStatePublisher.Setup();

    // if (!setupResult.ok())
    // {
    //     std::cerr
    //         << "[MotorStatePublisher] Setup failed\n";

    //     return 1;
    // }

    std::cout   << "[MotorStatePublisher] Running...\n";

    // --------------------------------------------
    // Main loop
    // --------------------------------------------

    while (running.load())
    {
        motorStatePublisher.Tick();

        // Small sleep to avoid spinning unnecessarily.
        // Actual publishing rate is controlled inside OnTick().
        std::this_thread::sleep_for(
            std::chrono::milliseconds(1));
    }

    std::cout
        << "[MotorStatePublisher] Shutting down...\n";

    return 0;
}