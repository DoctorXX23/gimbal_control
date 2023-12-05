#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/info/info.h>
#include <mavsdk/plugins/gimbal/gimbal.h>

#include <chrono>
#include <functional>
#include <future>
#include <iostream>
#include <thread>
#include <string>
#include <utility>

#include <stdint.h>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/string.hpp"

namespace gimbal
{
    class GimbalControl : public rclcpp::Node
    {
    public:
        GimbalControl();
    private:
        std::unique_ptr<mavsdk::Mavsdk> _mavsdk;
        std::unique_ptr<mavsdk::Action> _action;
        std::shared_ptr<mavsdk::Telemetry> _telemetry;
        std::shared_ptr<mavsdk::Gimbal> _gimbal;

        void cbStartGimbalControl(const std::shared_ptr<std_srvs::srv::Trigger::Request> aRequest, const std::shared_ptr<std_srvs::srv::Trigger::Response> aResponse);
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvStartGimbalControl;

        std::shared_ptr<mavsdk::System> getSystem(mavsdk::Mavsdk& aMavsdk);
    };
}