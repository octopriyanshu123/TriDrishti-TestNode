#include <chrono>
#include <cstdio>
#include <iostream>
#include <thread>
#include <utility>

#include "i2w/impl.hpp"
#include "crawler_i2w_services/uirobotconnectioncheck.hpp"

class UiRobotConnectionCheck final : public i2w::SystemBase
{
public:
    explicit UiRobotConnectionCheck(i2w::Config config)
        : i2w::SystemBase(std::move(config))
    {
    }

private:

    i2w::LifecycleResult OnSetup() override
    {
        i2w::ServiceOptions options;

        options.plane = plane_;
        options.max_outstanding_calls = 16;
        options.call_timeout_ms = 500;

        auto client = runtime().create_client<crawler_i2w_services::UiRobotConnectionCheckRequest,crawler_i2w_services::UiRobotConnectionCheckReponse>(service_name_,options);

        if (!client)
        {
            std::printf(
                "create_client failed: %s\n",
                i2w::to_string(client.error())
            );

            return i2w::Fail();
        }

        client_ = std::move(client.value());
        next_call_ = std::chrono::steady_clock::now() + std::chrono::seconds(1);

        return i2w::Ok();
    }

    i2w::LifecycleResult OnTick() override
    {
        const auto now = std::chrono::steady_clock::now();

        // If we're waiting on a response and it's taken too long, mark the UI as not live.
        if (waiting_for_response_ && now >= response_deadline_)
        {
            waiting_for_response_ = false;
            is_ui_live_ = false;
        }

        // Rate-limit outgoing calls to once per second.
        if (now >= next_call_)
        {
            crawler_i2w_services::UiRobotConnectionCheckRequest request;

            request.ping = 1;
            request.timestamp = static_cast<std::uint64_t>(runtime().clock().now().ns);

            const auto result = client_.call(request, runtime().clock().now().ns,
                [this](const i2w::Sample<crawler_i2w_services::UiRobotConnectionCheckReponse>& sample)
                {
                    is_ui_live_ = true;
                    waiting_for_response_ = false;

                    // std::cout << "Response received: pong = "
                    //           << static_cast<int>(sample.value.pong)
                    //           << std::endl;
                });

            waiting_for_response_ = true;
            response_deadline_ = now + std::chrono::milliseconds(500);
            next_call_ = now + std::chrono::milliseconds(500);
        }

        // Print current liveness state every tick.
        std::cout << "UI live: " << (is_ui_live_ ? "true" : "false") << std::endl;

        return i2w::Ok();
    }

private:

    std::string service_name_{"/ui_robot_connection_check"};

    i2w::EndpointPlane plane_{i2w::EndpointPlane::Local};

    i2w::Client<crawler_i2w_services::UiRobotConnectionCheckRequest,crawler_i2w_services::UiRobotConnectionCheckReponse> client_{};
    bool waiting_for_response_{false};
    bool is_ui_live_{false};
    std::chrono::steady_clock::time_point next_call_{};
    std::chrono::steady_clock::time_point response_deadline_{};
};

int main()
{
    i2w::Config config;

    config.node_name = "UiRobotConnectionCheck";
    config.ns = "";

    UiRobotConnectionCheck node(config);

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