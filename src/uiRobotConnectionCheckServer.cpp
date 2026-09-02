#include <chrono>
#include <cstdio>
#include <iostream>
#include <thread>
#include <utility>

#include "i2w/impl.hpp"
#include "crawler_i2w_services/uirobotconnectioncheck.hpp"

class UiRobotConnectionCheckServer final : public i2w::SystemBase
{
public:
    explicit UiRobotConnectionCheckServer(i2w::Config config)
        : i2w::SystemBase(std::move(config))
    {
    }

private:
    i2w::LifecycleResult OnSetup() override
    {
        i2w::ServiceOptions server_options;
        server_options.plane = plane_;
        server_options.max_outstanding_calls = 1;
        server_options.call_timeout_ms = 2000;

        auto server = runtime().advertise_service<crawler_i2w_services::UiRobotConnectionCheckRequest, crawler_i2w_services::UiRobotConnectionCheckReponse>(
            service_name_,
            [this](const crawler_i2w_services::UiRobotConnectionCheckRequest &request, const i2w::Header &header, crawler_i2w_services::UiRobotConnectionCheckReponse &response)
            {
                std::cout << "Request received: ping = "
                          << static_cast<int>(request.ping)
                          << std::endl;
                response.pong = 1;
                response.timestamp = request.timestamp;
            },
            server_options);

        if (!server)
        {
            std::printf(
                "create_client failed: %s\n",
                i2w::to_string(server.error()));

            return i2w::Fail();
        }

        server_ = std::move(server.value());

        return i2w::Ok();
    }

    i2w::LifecycleResult OnTick() override
    {
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));

        return i2w::Ok();
    }

private:
    std::string service_name_{"/ui_robot_connection_check"};

    i2w::EndpointPlane plane_{i2w::EndpointPlane::Local};

    i2w::Server<crawler_i2w_services::UiRobotConnectionCheckRequest, crawler_i2w_services::UiRobotConnectionCheckReponse> server_{};
    bool waiting_for_response_{false};
    bool is_ui_live_{false};
    std::chrono::steady_clock::time_point next_call_{};
    std::chrono::steady_clock::time_point response_deadline_{};
};

int main()
{
    i2w::Config config;

    config.node_name = "UiRobotConnectionCheckClient";
    config.ns = "";

    UiRobotConnectionCheckServer node(config);

    if (!node.Setup().ok)
    {
        std::cerr << "Setup failed\n";
        return 1;
    }

    while (true)
    {
        node.Tick();

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return 0;
}