// #include <iostream>
// #include <i2w/impl.hpp>
// #include <csignal>

// #include "crawler_i2w_services/channelSwitching.hpp"
// #include "crawler_i2w_services/moveRobot.hpp"
// #include "crawler_i2w_services/rasterManualControl.hpp"
// #include "crawler_i2w_services/uirobotconnectioncheck.hpp"

// std::atomic_bool running{true};

// void Stop(int)
// {
//     running.store(false);
// }

// class ServiceCaller final : public i2w::SystemBase
// {

// public:
//     explicit ServiceCaller(i2w::Config config)
//         : i2w::SystemBase(std::move(config))
//     {
//         std::cout << "JoyNode Constructor " << std::endl;
//     }

// private:
//     i2w::LifecycleResult OnSetup()
//     {
//         std::cout << "ServiceCaller OnSetup " << std::endl;

//         return i2w::Ok();
//     }

//     i2w::LifecycleResult OnTick() noexcept
//     {
//         std::cout << "ServiceCaller OnTick " << std::endl;

//         return i2w::Ok();
//     }

//     void OnDispose()
//     {
//     }
// };

// int main()
// {

//     std::signal(SIGINT, Stop);
//     std::signal(SIGTERM, Stop);
//     i2w::Config i2wServiceCallerConfig;
//     i2wServiceCallerConfig.node_name = "Joy_pub";
//     i2wServiceCallerConfig.ns = "";

//     auto networkProfileFilePath = std::string(CONFIG_DIR) + "/ecal-network-udp.yaml";

//     std::cout << "Network Profile File Path " << networkProfileFilePath << std::endl;

//     i2wServiceCallerConfig.transport.network_profile_file = networkProfileFilePath;

//     ServiceCaller serviceCallerNode(i2wServiceCallerConfig);

//     serviceCallerNode.Setup();

//     while (running.load())
//     {

//         serviceCallerNode.Tick();
//     }

//     serviceCallerNode.Dispose();

//     return 0;
// }

#include <iostream>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <string>
#include <thread>
#include <utility>

#include <i2w/impl.hpp>

#include "crawler_i2w_services/channelSwitching.hpp"
#include "crawler_i2w_services/moveRobot.hpp"
#include "crawler_i2w_services/rasterManualControl.hpp"
#include "crawler_i2w_services/uirobotconnectioncheck.hpp"

// =============================================================================
// Global running flag
// =============================================================================

std::atomic_bool running{true};

// =============================================================================
// Signal handler
// =============================================================================

void Stop(int)
{
    running.store(false);
}

// =============================================================================
// Service IDs
// =============================================================================

enum class ServiceType
{
    ChannelSwitching = 1,
    MoveRobot = 2,
    RasterManualControl = 3,
    UiRobotConnectionCheck = 4
};

// =============================================================================
// Service Caller
// =============================================================================

class ServiceCaller final : public i2w::SystemBase
{
public:
    explicit ServiceCaller(i2w::Config config)
        : i2w::SystemBase(std::move(config))
    {
        std::cout << "ServiceCaller Constructor" << std::endl;
    }

private:
    ServiceType AskService()
    {
        int choice{};

        std::cout << "\nSelect service:\n"
                  << "1. Channel Switching\n"
                  << "2. Move Robot\n"
                  << "3. Raster Manual Control\n"
                  << "4. UI Robot Connection Check\n"
                  << "0. Exit\n"
                  << "Enter choice: ";

        std::cin >> choice;

        switch (choice)
        {
        case 1:
            return ServiceType::ChannelSwitching;

        case 2:
            return ServiceType::MoveRobot;

        case 3:
            return ServiceType::RasterManualControl;

        case 4:
            return ServiceType::UiRobotConnectionCheck;

        default:
            return static_cast<ServiceType>(0);
        }
        std::cout << "=====================================================" << std::endl;

        std::cout << std::endl;
    }

    // =========================================================================
    // Setup
    // =========================================================================

    i2w::LifecycleResult OnSetup()
    {
        std::cout << "ServiceCaller OnSetup" << std::endl;

        return i2w::Ok();
    }

    // =========================================================================
    // Tick
    // =========================================================================

    i2w::LifecycleResult OnTick() noexcept
    {
        // Example:
        //

        const ServiceType service = AskService();

        if (service == static_cast<ServiceType>(0))
        {
            running.store(false);
            return i2w::Ok();
        }

        CallService(service);

        return i2w::Ok();
    }

    // =========================================================================
    // Generic service dispatcher
    // =========================================================================

    void CallService(ServiceType service)
    {
        switch (service)
        {
            // =================================================================
            // Channel Switching
            // =================================================================

        case ServiceType::ChannelSwitching:
        {
            std::cout
                << "Calling ChannelSwitching service"
                << std::endl;

            // using Request =
            //     crawler_i2w_services::ChannelSwitchingRequest;

            // using Response =
            //     crawler_i2w_services::ChannelSwitchingResponse;

            // auto client =
            //     setupClient<Request, Response>(
            //         runtime(),
            //         "/channel_switching",
            //         i2w::EndpointPlane::Network,
            //         16);

            // if (!client)
            // {
            //     std::cerr
            //         << "Failed to create ChannelSwitching client"
            //         << std::endl;

            //     return;
            // }

            // Request request{};

            // // Fill request here.
            // //
            // // request.channel = ...;

            // auto result =
            //     client->call(request);

            // if (!result)
            // {
            //     std::cerr
            //         << "ChannelSwitching call failed"
            //         << std::endl;

            //     return;
            // }

            // std::cout
            //     << "ChannelSwitching call successful"
            //     << std::endl;

            // break;
        }

            // =================================================================
            // Move Robot
            // =================================================================

        case ServiceType::MoveRobot:
        {
            std::cout
                << "Calling MoveRobot service"
                << std::endl;

            // using Request =
            //     crawler_i2w_services::MoveRobotRequest;

            // using Response =
            //     crawler_i2w_services::MoveRobotResponse;

            // auto client =
            //     setupClient<Request, Response>(
            //         runtime(),
            //         "/move_robot",
            //         i2w::EndpointPlane::Network,
            //         16);

            // if (!client)
            // {
            //     std::cerr
            //         << "Failed to create MoveRobot client"
            //         << std::endl;

            //     return;
            // }

            // Request request{};

            // // Fill request here.
            // //
            // // request.xxx = ...;

            // auto result =
            //     client->call(request);

            // if (!result)
            // {
            //     std::cerr
            //         << "MoveRobot call failed"
            //         << std::endl;

            //     return;
            // }

            // std::cout
            //     << "MoveRobot call successful"
            //     << std::endl;

            // break;
        }

            // =================================================================
            // Raster Manual Control
            // =================================================================

        case ServiceType::RasterManualControl:
        {
            std::cout
                << "Calling RasterManualControl service"
                << std::endl;

            // using Request =
            //     crawler_i2w_services::RasterManualControlRequest;

            // using Response =
            //     crawler_i2w_services::RasterManualControlResponse;

            // auto client =
            //     setupClient<Request, Response>(
            //         runtime(),
            //         "/raster_manual_control",
            //         i2w::EndpointPlane::Network,
            //         16);

            // if (!client)
            // {
            //     std::cerr
            //         << "Failed to create RasterManualControl client"
            //         << std::endl;

            //     return;
            // }

            // Request request{};

            // // Fill request here.
            // //
            // // request.xxx = ...;

            // auto result =
            //     client->call(request);

            // if (!result)
            // {
            //     std::cerr
            //         << "RasterManualControl call failed"
            //         << std::endl;

            //     return;
            // }

            // std::cout
            //     << "RasterManualControl call successful"
            //     << std::endl;

            // break;
        }

            // =================================================================
            // UI Robot Connection Check
            // =================================================================

        case ServiceType::UiRobotConnectionCheck:
        {
            std::cout
                << "Calling UiRobotConnectionCheck service"
                << std::endl;

            // using Request =
            //     crawler_i2w_services::UiRobotConnectionCheckRequest;

            // using Response =
            //     crawler_i2w_services::UiRobotConnectionCheckReponse;

            // auto client =
            //     setupClient<Request, Response>(
            //         runtime(),
            //         "/ui_robot_connection_check",
            //         i2w::EndpointPlane::Network,
            //         16);

            // if (!client)
            // {
            //     std::cerr
            //         << "Failed to create UiRobotConnectionCheck client"
            //         << std::endl;

            //     return;
            // }

            // Request request{};

            // auto result =
            //     client->call(request);

            // if (!result)
            // {
            //     std::cerr
            //         << "UiRobotConnectionCheck call failed"
            //         << std::endl;

            //     return;
            // }

            // std::cout
            //     << "UiRobotConnectionCheck call successful"
            //     << std::endl;

            // break;
        }

            // =================================================================
            // Invalid service
            // =================================================================

        default:
        {
            std::cerr
                << "Unknown service"
                << std::endl;

            break;
        }
        }
    }

    // =========================================================================
    // Dispose
    // =========================================================================

    void OnDispose()
    {
        std::cout << "ServiceCaller OnDispose" << std::endl;
    }
};

// =============================================================================
// Main
// =============================================================================

int main()
{
    // -------------------------------------------------------------------------
    // Signal handling
    // -------------------------------------------------------------------------

    std::signal(SIGINT, Stop);
    std::signal(SIGTERM, Stop);

    // -------------------------------------------------------------------------
    // i2w configuration
    // -------------------------------------------------------------------------

    i2w::Config config;

    config.node_name = "ServiceCaller";
    config.ns = "";

    // -------------------------------------------------------------------------
    // Network profile
    // -------------------------------------------------------------------------

    const auto networkProfileFilePath = std::string(CONFIG_DIR) + "/ecal-network-udp.yaml";

    std::cout
        << "Network Profile File Path: "
        << networkProfileFilePath
        << std::endl;

    config.transport.network_profile_file = networkProfileFilePath;

    // -------------------------------------------------------------------------
    // Create node
    // -------------------------------------------------------------------------

    ServiceCaller serviceCallerNode(config);

    // -------------------------------------------------------------------------
    // Setup
    // -------------------------------------------------------------------------

    if (!serviceCallerNode.Setup().ok)
    {
        std::cerr
            << "ServiceCaller setup failed"
            << std::endl;

        return 1;
    }

    std::cout
        << "ServiceCaller setup successful"
        << std::endl;

    // -------------------------------------------------------------------------
    // Main loop
    // -------------------------------------------------------------------------

    while (running.load())
    {
        if (!serviceCallerNode.Tick().ok)
        {
            std::cerr
                << "ServiceCaller tick failed"
                << std::endl;

            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // -------------------------------------------------------------------------
    // Cleanup
    // -------------------------------------------------------------------------

    serviceCallerNode.Dispose();

    std::cout
        << "ServiceCaller stopped"
        << std::endl;

    return 0;
}