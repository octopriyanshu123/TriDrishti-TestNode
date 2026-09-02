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
        options.call_timeout_ms = 2000;

        auto client =
            runtime().create_client<crawler_i2w_services::UiRobotConnectionCheckRequest,crawler_i2w_services::UiRobotConnectionCheckReponse>(service_name_,options);

        if (!client)
        {
            std::printf(
                "create_client failed: %s\n",
                i2w::to_string(client.error())
            );

            return i2w::Fail();
        }

        client_ = std::move(client.value());

        return i2w::Ok();
    }

    i2w::LifecycleResult OnTick() override
    {
        const auto now = std::chrono::steady_clock::now();

        if (now < next_call_)
        {
            return i2w::Ok();
        }

        next_call_ = now + std::chrono::milliseconds(500);

        std::cout << "Input the Channel id: ";

        if (!(std::cin >> deviceId))
        {
            std::cerr << "Invalid channel ID\n";

            // Clear bad input
            std::cin.clear();
            std::cin.ignore(
                std::numeric_limits<std::streamsize>::max(),
                '\n'
            );

            return i2w::Fail();
        }

        crawler_i2w_services::UiRobotConnectionCheckRequest request;

        request.ping = 1;

        request.timestamp =
            static_cast<std::uint64_t>(
                runtime().clock().now().ns
            );

        // If your request has this field:
        // request.device_id = deviceId;

        const auto result =
            client_.call(
                request,
                runtime().clock().now().ns,
                [](const i2w::Sample<
                       crawler_i2w_services::UiRobotConnectionCheckReponse
                   >& sample)
                {
                    std::cout
                        << "Response received: pong = "
                        << static_cast<int>(sample.value.pong)
                        << std::endl;
                }
            );

        if (!result)
        {
            std::printf(
                "call failed: %s\n",
                i2w::to_string(result.error())
            );

            return i2w::Fail();
        }

        std::cout << "Request sent for channel: "
                  << deviceId
                  << std::endl;

        return i2w::Ok();
    }

private:

    std::string service_name_{"/channel_switching_service"};

    i2w::EndpointPlane plane_{i2w::EndpointPlane::Local};

    i2w::Client<
        crawler_i2w_services::UiRobotConnectionCheckRequest,
        crawler_i2w_services::UiRobotConnectionCheckReponse
    > client_{};

    std::chrono::steady_clock::time_point next_call_{};

    int deviceId{-1};
};

int main()
{
    i2w::Config config;

    config.node_name = "UiRobotConnectionCheck";
    config.ns = "robot";

    UiRobotConnectionCheck node(config);

    if (!node.Setup().ok)
    {
        std::cerr << "Setup failed\n";
        return 1;
    }

    while (true)
    {
        node.Tick();

        std::this_thread::sleep_for(
            std::chrono::milliseconds(20)
        );
    }

    return 0;
}