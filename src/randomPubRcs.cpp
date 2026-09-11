#include <random>

#include "i2w/impl.hpp"
#include "crawler_i2w_msgs/ui/joy.hpp"
#include "crawler_i2w_msgs/robot/cmd_vel.hpp"
#include "crawler_i2w_msgs/robot/imu.hpp"
#include "crawler_i2w_msgs/robot/diff_drive_odometry.hpp"
#include "crawler_i2w_msgs/robot/tank_meta_data.hpp"
#include "crawler_i2w_msgs/robot/robot_coordinate_system.hpp"

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
#include <signal.h>
std::atomic_bool running{true};

void Stop(int)
{
    running.store(false);
}
crawler_i2w_msgs::cmd_vel cmd_vel_g;
crawler_i2w_msgs::I2wDiffDriveOdometry odom_g;
crawler_i2w_msgs::imu imu_g;
crawler_i2w_msgs::tank_meta_data tankInfo_g;

float normalize(int16_t value, float max_output)
{

    return (static_cast<float>(value) / 32767.0f) * max_output;
}

class RobotCoordinateSystem final : public i2w::SystemBase
{
public:
    explicit RobotCoordinateSystem(i2w::Config config)
        : i2w::SystemBase(std::move(config))
    {
    }

private:
    i2w::LifecycleResult OnSetup()
    {

        // Subsciber

        i2w::SubscriptionOptions imuSubOpts;
        imuSubOpts.plane = i2w::EndpointPlane::Local;
        imuSubOpts.reliability = i2w::Reliability::BestEffort;
        imuSubOpts.queue_depth = 32;
        imuSubOpts.overflow_policy = i2w::OverflowPolicy::DropOldest;
        auto imuSubscription = runtime().subscribe<crawler_i2w_msgs::imu>(
            "/imu",
            [this](const i2w::Sample<crawler_i2w_msgs::imu> &sample)
            {
                imu_g = sample.value;
            },
            imuSubOpts);
        imuSubscription_ = std::move(imuSubscription.value());

        i2w::SubscriptionOptions odomSubOpts;
        odomSubOpts.plane = i2w::EndpointPlane::Local;
        odomSubOpts.reliability = i2w::Reliability::BestEffort;
        odomSubOpts.queue_depth = 32;
        odomSubOpts.overflow_policy = i2w::OverflowPolicy::DropOldest;
        auto odomSubscription = runtime().subscribe<crawler_i2w_msgs::I2wDiffDriveOdometry>(
            "/odom",
            [this](const i2w::Sample<crawler_i2w_msgs::I2wDiffDriveOdometry> &sample)
            {
                odom_g = sample.value;
            },
            odomSubOpts);
        odometrySubscription_ = std::move(odomSubscription.value());

        i2w::SubscriptionOptions tankInfoOpts;
        tankInfoOpts.plane = i2w::EndpointPlane::Local;
        tankInfoOpts.reliability = i2w::Reliability::BestEffort;
        tankInfoOpts.queue_depth = 32;
        tankInfoOpts.overflow_policy = i2w::OverflowPolicy::DropOldest;
        auto tankInfoSubscription = runtime().subscribe<crawler_i2w_msgs::tank_meta_data>(
            "/tank_info",
            [this](const i2w::Sample<crawler_i2w_msgs::tank_meta_data> &sample)
            {
                tankInfo_g = sample.value;
            },
            tankInfoOpts);
        tankInfoSubscription_ = std::move(tankInfoSubscription.value());

        // Publisher

        i2w::PublisherOptions rcsPubOpt;
        rcsPubOpt.plane = i2w::EndpointPlane::Local;
        auto rcsPublisher = runtime().advertise<crawler_i2w_msgs::RobotCoordinateSystem>("/rcs", rcsPubOpt);
        rcsPublisher_ = std::move(rcsPublisher.value());

        std::cout << "RobotCoordinateSystem OnSetup " << std::endl;

        return i2w::Ok();
    }
    i2w::LifecycleResult OnTick()
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        publishRCS();

        return i2w::Ok();
    }
    //   void OnDispose() override;

    void publishRCS()
    {
        static std::random_device rd;
        static std::mt19937 gen(rd());

        // Random RCS values
        static std::uniform_real_distribution<double> radialDist(0.0, 360.0);
        static std::uniform_real_distribution<double> altitudeDist(0.0, 20.0);
        static std::uniform_real_distribution<double> bearingDist(0.0, 360.0);

        crawler_i2w_msgs::RobotCoordinateSystem rcsData_;

        rcsData_.timestamp =
            static_cast<std::uint64_t>(runtime().clock().now().ns);

        rcsData_.radial = radialDist(gen);
        rcsData_.altitude = altitudeDist(gen);
        rcsData_.bearing = bearingDist(gen);

        (void)rcsPublisher_.publish(
            rcsData_,
            static_cast<std::int64_t>(rcsData_.timestamp));

        std::cout
            << "[RCS]"
            << " timestamp=" << rcsData_.timestamp
            << " radial=" << rcsData_.radial
            << " altitude=" << rcsData_.altitude
            << " bearing=" << rcsData_.bearing
            << std::endl;
    }

    i2w::Publisher<crawler_i2w_msgs::RobotCoordinateSystem> rcsPublisher_{};

    i2w::Subscription<crawler_i2w_msgs::I2wDiffDriveOdometry> odometrySubscription_{};
    i2w::Subscription<crawler_i2w_msgs::imu> imuSubscription_{};
    i2w::Subscription<crawler_i2w_msgs::tank_meta_data> tankInfoSubscription_{};
};

int main()
{

    i2w::Config rcsConfig;
    rcsConfig.node_name = "rcs";
    rcsConfig.ns = "";
    // rcsConfig.transport.network_profile_file = "/home/octobot/Github/TriDrishti-ws/src/i2w/examples/config/ecal-network-udp.yaml";

    RobotCoordinateSystem rcs(rcsConfig);

    rcs.Setup();

    while (running)
    {
        rcs.Tick();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::cout<<"Close the Program Gresefully";

    return 0;
}