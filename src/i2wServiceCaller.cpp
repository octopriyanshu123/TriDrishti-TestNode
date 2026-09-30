#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include <utility>

#include <i2w/impl.hpp>

#include "crawler_i2w_services/channelSwitching.hpp"
#include "crawler_i2w_services/moveRobot.hpp"
#include "crawler_i2w_services/rasterManualControl.hpp"
#include "crawler_i2w_services/uirobotconnectioncheck.hpp" // check exact filename case on Linux

std::atomic_bool running{true};
static_assert(std::atomic_bool::is_always_lock_free,
              "atomic_bool must be lock-free to be safe in a signal handler");

void Stop(int)
{
    running.store(false);
}

enum class ServiceType
{
    Exit = 0,
    ChannelSwitching = 1,
    MoveRobot = 2,
    RasterManualControl = 3,
    UiRobotConnectionCheck = 4,
    MoveProbToPosition = 5,
    MoveLAToPosition = 6,
    GetProbPosition = 7,
    GetLAPosition = 8,
    Invalid = -1
};

class ServiceCaller final : public i2w::SystemBase
{
public:
    explicit ServiceCaller(i2w::Config config)
        : i2w::SystemBase(std::move(config))
    {
        std::cout << "ServiceCaller Constructor" << std::endl;
    }

private:
    // ------------------------------------------------------------------
    // Input
    // ------------------------------------------------------------------
    ServiceType AskService()
    {
        std::cout << "\n=====================================================\n"
                  << "Select service:\n"
                  << "1. Channel Switching\n"
                  << "2. Move Robot\n"
                  << "3. Raster Manual Control\n"
                  << "4. UI Robot Connection Check\n"
                  << "5. Move Prob To Position\n"
                  << "6. Move LA To Position\n"
                  << "7. Get Prob Position\n"
                  << "8. Get LA Position\n"
                  << "0. Exit\n"
                  << "Enter choice: " << std::flush;

        int choice{};
        if (!(std::cin >> choice))
        {
            if (std::cin.eof() || !running.load())
            {
                return ServiceType::Exit; // Ctrl+D, or interrupted by a signal
            }
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            return ServiceType::Invalid; // non-numeric input
        }

        if (choice >= 0 && choice <= 8)
        {
            return static_cast<ServiceType>(choice);
        }
        return ServiceType::Invalid;
    }
    // Clients
    // abs

    i2w::Client<crawler_i2w_services::RasterProbMoveTORequest, crawler_i2w_services::RasterProbMoveToResponse> RasterProbMoveToClient_{};
    i2w::Client<crawler_i2w_services::RasterLinearActuatorMoveToRequest, crawler_i2w_services::RasterLinearActuatorMoveToResponse> RasterLAMoveToClient_{};

    // getter

    i2w::Client<crawler_i2w_services::GetRasterProbPositionRequest, crawler_i2w_services::GetRasterProbPositionResponse> GetRasterProbPositionClient_{};
    i2w::Client<crawler_i2w_services::GetRasterLAPositionRequest, crawler_i2w_services::GetRasterLPPositionResponse> GetRasterLPPositionClient_{};
    // ------------------------------------------------------------------
    // Lifecycle
    // `override` makes the compiler verify these really override the base
    // class hooks. If compilation fails here, the signature doesn't match
    // i2w::SystemBase, which means these were never being called.
    // ------------------------------------------------------------------

    i2w::LifecycleResult OnSetup() override
    {

        i2w::ServiceOptions common_service_option;
        common_service_option.plane = i2w::EndpointPlane::Local;
        common_service_option.max_outstanding_calls = 16;
        common_service_option.call_timeout_ms = 2000;

        auto RasterProbMoveToClient = runtime().create_client<crawler_i2w_services::RasterProbMoveTORequest, crawler_i2w_services::RasterProbMoveToResponse>("/raster_prob_move_to_service", common_service_option);
        if (!RasterProbMoveToClient)
        {
            std::printf("create_client failed: %s\n", i2w::to_string(RasterProbMoveToClient.error()));
            return i2w::Fail();
        }

        RasterProbMoveToClient_ = std::move(RasterProbMoveToClient.value());

        auto RasterLAMoveToClient = runtime().create_client<crawler_i2w_services::RasterLinearActuatorMoveToRequest, crawler_i2w_services::RasterLinearActuatorMoveToResponse>("/raster_linearActuator_move_to_service", common_service_option);
        if (!RasterLAMoveToClient)
        {
            std::printf("create_client failed: %s\n", i2w::to_string(RasterLAMoveToClient.error()));
            return i2w::Fail();
        }

        RasterLAMoveToClient_ = std::move(RasterLAMoveToClient.value());

        auto GetRasterProbPositionClient = runtime().create_client<crawler_i2w_services::GetRasterProbPositionRequest, crawler_i2w_services::GetRasterProbPositionResponse>("/get_raster_prob_position_service", common_service_option);
        if (!RasterLAMoveToClient)
        {
            std::printf("create_client failed: %s\n", i2w::to_string(GetRasterProbPositionClient.error()));
            return i2w::Fail();
        }

        GetRasterProbPositionClient_ = std::move(GetRasterProbPositionClient.value());

        auto GetRasterLPPositionClient = runtime().create_client<crawler_i2w_services::GetRasterLAPositionRequest, crawler_i2w_services::GetRasterLPPositionResponse>("/get_raster_la_position_service", common_service_option);
        if (!GetRasterLPPositionClient)
        {
            std::printf("create_client failed: %s\n", i2w::to_string(GetRasterLPPositionClient.error()));
            return i2w::Fail();
        }

        GetRasterLPPositionClient_ = std::move(GetRasterLPPositionClient.value());

        std::cout << "ServiceCaller OnSetup" << std::endl;

        return i2w::Ok();
    }

    i2w::LifecycleResult OnTick() noexcept override
    {
        const ServiceType service = AskService();

        if (service == ServiceType::Exit)
        {
            running.store(false);
            return i2w::Ok();
        }

        if (service == ServiceType::Invalid)
        {
            std::cerr << "Invalid choice, please try again." << std::endl;
            return i2w::Ok();
        }

        CallService(service);
        return i2w::Ok();
    }

    void OnDispose() override
    {
        std::cout << "ServiceCaller OnDispose" << std::endl;
    }

    // ------------------------------------------------------------------
    // Dispatch: one method per service, and every case has a break
    // ------------------------------------------------------------------
    void CallService(ServiceType service)
    {
        switch (service)
        {
        case ServiceType::ChannelSwitching:
            CallChannelSwitching();
            break;
        case ServiceType::MoveRobot:
            CallMoveRobot();
            break;
        case ServiceType::RasterManualControl:
            CallRasterManualControl();
            break;
        case ServiceType::UiRobotConnectionCheck:
            CallUiRobotConnectionCheck();
            break;
        case ServiceType::MoveProbToPosition:
            CallMoveProbToPosition();
            break;
        case ServiceType::MoveLAToPosition:
            CallMoveLAToPosition();
            break;
        case ServiceType::GetProbPosition:
            CallGetProbPosition();
            break;
        case ServiceType::GetLAPosition:
            CallGetLAPosition();
            break;
        default:
            std::cerr << "Unknown service" << std::endl;
            break;
        }
    }

    // ------------------------------------------------------------------
    // Service calls
    // ------------------------------------------------------------------
    void CallChannelSwitching()
    {
        std::cout << "Calling ChannelSwitching service" << std::endl;

        // using Request  = crawler_i2w_services::ChannelSwitchingRequest;
        // using Response = crawler_i2w_services::ChannelSwitchingResponse;
        //
        // auto client = setupClient<Request, Response>(
        //     runtime(), "/channel_switching", i2w::EndpointPlane::Network, 16);
        // if (!client)
        // {
        //     std::cerr << "Failed to create ChannelSwitching client" << std::endl;
        //     return;
        // }
        //
        // Request request{};
        // // request.channel = ...;
        //
        // auto result = client->call(request);
        // if (!result)
        // {
        //     std::cerr << "ChannelSwitching call failed" << std::endl;
        //     return;
        // }
        // std::cout << "ChannelSwitching call successful" << std::endl;
    }

    void CallMoveRobot()
    {
        std::cout << "Calling MoveRobot service" << std::endl;

        // using Request  = crawler_i2w_services::MoveRobotRequest;
        // using Response = crawler_i2w_services::MoveRobotResponse;
        //
        // auto client = setupClient<Request, Response>(
        //     runtime(), "/move_robot", i2w::EndpointPlane::Network, 16);
        // if (!client)
        // {
        //     std::cerr << "Failed to create MoveRobot client" << std::endl;
        //     return;
        // }
        //
        // Request request{};
        // // request.xxx = ...;
        //
        // auto result = client->call(request);
        // if (!result)
        // {
        //     std::cerr << "MoveRobot call failed" << std::endl;
        //     return;
        // }
        // std::cout << "MoveRobot call successful" << std::endl;
    }

    void CallRasterManualControl()
    {
        std::cout << "Calling RasterManualControl service" << std::endl;

        // using Request  = crawler_i2w_services::RasterManualControlRequest;
        // using Response = crawler_i2w_services::RasterManualControlResponse;
        //
        // auto client = setupClient<Request, Response>(
        //     runtime(), "/raster_manual_control", i2w::EndpointPlane::Network, 16);
        // if (!client)
        // {
        //     std::cerr << "Failed to create RasterManualControl client" << std::endl;
        //     return;
        // }
        //
        // Request request{};
        // // request.xxx = ...;
        //
        // auto result = client->call(request);
        // if (!result)
        // {
        //     std::cerr << "RasterManualControl call failed" << std::endl;
        //     return;
        // }
        // std::cout << "RasterManualControl call successful" << std::endl;
    }

    void CallUiRobotConnectionCheck()
    {
        std::cout << "Calling UiRobotConnectionCheck service" << std::endl;

        // using Request  = crawler_i2w_services::UiRobotConnectionCheckRequest;
        // using Response = crawler_i2w_services::UiRobotConnectionCheckReponse; // "Reponse"? check header
        //
        // auto client = setupClient<Request, Response>(
        //     runtime(), "/ui_robot_connection_check", i2w::EndpointPlane::Network, 16);
        // if (!client)
        // {
        //     std::cerr << "Failed to create UiRobotConnectionCheck client" << std::endl;
        //     return;
        // }
        //
        // Request request{};
        //
        // auto result = client->call(request);
        // if (!result)
        // {
        //     std::cerr << "UiRobotConnectionCheck call failed" << std::endl;
        //     return;
        // }
        // std::cout << "UiRobotConnectionCheck call successful" << std::endl;
    }

    void CallMoveProbToPosition()
    {
        std::cout << " MoveProbToPosition service" << std::endl;
        std::cout << " Enter the Prob Position " << std::endl;
        int targetPosition = 0;
        std::cin >> targetPosition;

        crawler_i2w_services::RasterProbMoveTORequest request;
        request.id = 1;
        request.position = targetPosition;

        // std::cout << " id " << request.id << " position " << request.position << std::endl;

        const auto result = RasterProbMoveToClient_.call(request, runtime().clock().now().ns, [](const i2w::Sample<crawler_i2w_services::RasterProbMoveToResponse> &sample)
                                                         { std::cout << " Status of the Service Call " << sample.value.status << std::endl; });
    }

    void CallMoveLAToPosition()
    {
        std::cout << "Calling MoveLAToPosition service" << std::endl;

        std::cout << " Enter the LA id " << std::endl;
        int targetLAid = 0;
        std::cin >> targetLAid;

        std::cout << " Enter the LA Position " << std::endl;
        int targetPosition = 0;
        std::cin >> targetPosition;

        crawler_i2w_services::RasterLinearActuatorMoveToRequest request;
        request.id = targetLAid;
        request.position = targetPosition;

        const auto result = RasterLAMoveToClient_.call(request, runtime().clock().now().ns, [](const i2w::Sample<crawler_i2w_services::RasterLinearActuatorMoveToResponse> &sample)
                                                       { std::cout << " Status of the Service Call " << sample.value.status << std::endl; });

        // TODO: implement
    }

    void CallGetProbPosition()
    {
        std::cout << "Call GetProbPosition" << std::endl;

        crawler_i2w_services::GetRasterProbPositionRequest request;
        request.id = 1;

        const auto result = GetRasterProbPositionClient_.call(request, runtime().clock().now().ns, [](const i2w::Sample<crawler_i2w_services::GetRasterProbPositionResponse> &sample)
                                                              { std::cout << "Prob Position is " << sample.value.position << std::endl; });
    }

    void CallGetLAPosition()
    {
        std::cout << "Call GetLAPosition" << std::endl;

        std::cout << " Enter the LA id " << std::endl;
        int targetLAid = 0;
        std::cin >> targetLAid;

        crawler_i2w_services::GetRasterLAPositionRequest request;
        request.id = targetLAid;

        const auto result = GetRasterLPPositionClient_.call(request, runtime().clock().now().ns, [](const i2w::Sample<crawler_i2w_services::GetRasterLPPositionResponse> &sample)
                                                            { std::cout << "LA Position is " << sample.value.position << std::endl; });
    }
};

int main()
{
    std::signal(SIGINT, Stop);
    std::signal(SIGTERM, Stop);

    i2w::Config config;
    config.node_name = "ServiceCaller";
    config.ns = "";

    // CONFIG_DIR must be defined by the build system, e.g. in CMake:
    // target_compile_definitions(service_caller PRIVATE CONFIG_DIR="${CMAKE_SOURCE_DIR}/config")
    const auto networkProfileFilePath = std::string(CONFIG_DIR) + "/ecal-network-udp.yaml";
    std::cout << "Network Profile File Path: " << networkProfileFilePath << std::endl;
    config.transport.network_profile_file = networkProfileFilePath;

    ServiceCaller serviceCallerNode(config);

    if (!serviceCallerNode.Setup().ok)
    {
        std::cerr << "ServiceCaller setup failed" << std::endl;
        return 1;
    }
    std::cout << "ServiceCaller setup successful" << std::endl;

    // Note: std::cin blocks, so Ctrl+C takes effect once the read is
    // interrupted or the next input arrives.
    while (running.load())
    {
        if (!serviceCallerNode.Tick().ok)
        {
            std::cerr << "ServiceCaller tick failed" << std::endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    serviceCallerNode.Dispose();
    std::cout << "ServiceCaller stopped" << std::endl;
    return 0;
}