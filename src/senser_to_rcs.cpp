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
        auto rcsPublisher = runtime().advertise<crawler_i2w_msgs::robot_coordinate_system>("/rcs", rcsPubOpt);
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

        // ── Cylindrical coordinates, from odom's dead-reckoning ────
        //  odom.x is arc-length travelled along the shell (from the
        //  update() call you're already making), so theta = x / R.
        //  odom.y is height, odom.theta is the odom's own integrated
        //  climb angle (psi).
        //
        //  Also prints the IMU's INDEPENDENT climb-angle estimate,
        //  psi_imu = atan2(accelX, accelY) — derived straight from a
        //  single accelerometer reading, no integration, so it never
        //  drifts. Printing both side by side is the whole point of
        //  "log raw, don't fuse yet": psi_odom vs. psi_imu is exactly
        //  the disagreement a future fusion step needs to correct.
        //
        //  tankRadius — the course's mean radius (mm or m, just be
        //  consistent with what you fed updateOdom's linear_vel in).

        double thetaRad = fmod(odom_g.x_m / tankInfo_g.tank_diameter_m / 2, 2.0 * M_PI);
        if (thetaRad < 0.0)
            thetaRad += 2.0 * M_PI;

        double thetaDeg = thetaRad * 180.0 / M_PI;

        double h = odom_g.y_m;
        int psiOdomDeg = odom_g.yaw_rad * 180.0 / M_PI;

        psiOdomDeg = psiOdomDeg % 360;
        double psiImuDeg = atan2(imu_g.accelX, imu_g.accelY) * 180.0 / M_PI;

        crawler_i2w_msgs::robot_coordinate_system rcsData_;
        rcsData_.timestamp = static_cast<std::uint64_t>(runtime().clock().now().ns);
        rcsData_.radial = thetaDeg;
        rcsData_.altitude = h;
        rcsData_.bearing = (psiOdomDeg + psiImuDeg) / 2;

        (void)rcsPublisher_.publish(rcsData_, static_cast<std::int64_t>(rcsData_.timestamp));

        // printf("[CYL] theta=%6.1fdeg  h=%7.3f  psi_odom=%6.1fdeg  psi_imu=%6.1fdeg\n",thetaDeg, h, psiOdomDeg, psiImuDeg);
    }

    i2w::Publisher<crawler_i2w_msgs::robot_coordinate_system> rcsPublisher_{};

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

    return 0;
}