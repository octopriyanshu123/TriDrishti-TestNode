#include <chrono>
#include <cstdio>
#include <thread>
#include <utility>
#include <iostream>

#include "i2w/impl.hpp"

#include "crawler_i2w_services/channelSwitching.hpp"

bool running{true};

class ChannelSwitchingClient final : public i2w::SystemBase
{
public:
    ChannelSwitchingClient(i2w::Config config) : i2w::SystemBase(std::move(config)) {}

private:
    i2w::LifecycleResult OnSetup() override
    {
        i2w::ServiceOptions options;
        options.plane = plane_;
        options.max_outstanding_calls = 16;
        options.call_timeout_ms = 2000;

        auto client = runtime().create_client<crawler_i2w_services::ChannelSwitchingRequest, crawler_i2w_services::ChannelSwitchingResponse>(service_name_, options);
        if (!client)
        {
            std::printf("create_client failed: %s\n", i2w::to_string(client.error()));
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

        std::cout << "Input the Chnnel id:" << std::endl;
        std::cin >> deviceId;

        crawler_i2w_services::ChannelSwitchingRequest request;
        request.device_id = deviceId;

        const auto result = client_.call(request, runtime().clock().now().ns, [](const i2w::Sample<crawler_i2w_services::ChannelSwitchingResponse> &sample)
                                         { 
                                            std::cout << "Channel Switching device_id = " << static_cast<int>(sample.value.device_id )<< std::endl; 
                                            std::cout << "Response received: result = "  <<  sample.value.result << std::endl; 
                                             std::cout <<std::endl; 
                                            std::terminate();});

        if (!result)
        {
            std::printf("call failed: %s\n", i2w::to_string(result.error()));
            return i2w::Fail();
        }

        return i2w::Ok();
    }

    std::string service_name_{"/channel_switching_service"};
    i2w::EndpointPlane plane_{i2w::EndpointPlane::Local};
    i2w::Client<crawler_i2w_services::ChannelSwitchingRequest, crawler_i2w_services::ChannelSwitchingResponse> client_{};
    std::uint64_t sent_{0};
    std::chrono::steady_clock::time_point next_call_{};
    int deviceId{-1};
};
int main()
{
    i2w::Config config;
    config.node_name = "ChannelSwitchingClient";
    config.ns = "robot";

    ChannelSwitchingClient node(config);

    if (!node.Setup().ok)
    {
        std::cout << "Setup failed\n";
        return 1;
    }
    while (true)
    {
        node.Tick();

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }    

    return 0;
}