#include "gimbal_control/gimbal_control.hpp"

using namespace std::placeholders;

namespace gimbal
{
    GimbalControl::GimbalControl() : Node("GimbalControl")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        declare_parameter("connection", "serial:///dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0:115200");

        _mavsdk = std::make_unique<mavsdk::Mavsdk>();

        auto connection = get_parameter("connection").as_string();

        mavsdk::ConnectionResult connectionResult = _mavsdk.get()->add_any_connection(connection);

        if(connectionResult != mavsdk::ConnectionResult::Success)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Connection failed");

            throw std::runtime_error("Connection failed");
        }

        auto system = getSystem(*_mavsdk);

        if(system == nullptr)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Timed out waiting for system");

            throw std::runtime_error("Timed out waiting for system");
        }

        _action = std::make_unique<mavsdk::Action>(system);
        _telemetry = std::make_shared<mavsdk::Telemetry>(system);
        _gimbal = std::make_shared<mavsdk::Gimbal>(system);

        // We want to listen to the camera/gimbal angle of the drone at 5 Hz.
        const mavsdk::Telemetry::Result set_rate_result = _telemetry.get()->set_rate_camera_attitude(5.0);
        if(set_rate_result != mavsdk::Telemetry::Result::Success)
        {
            std::cerr << "Setting rate failed:" << set_rate_result << '\n';
            throw std::runtime_error("Setting rate failed");
        }

        // Set up callback to monitor camera/gimbal angle
        _telemetry.get()->subscribe_camera_attitude_euler([](mavsdk::Telemetry::EulerAngle angle)
        {
            std::cout << "Gimbal angle pitch: " << angle.pitch_deg << " deg, yaw: " << angle.yaw_deg << " yaw\n";
        });

        std::cout << "Start controlling gimbal...\n";
        mavsdk::Gimbal::Result gimbal_result = _gimbal.get()->take_control(mavsdk::Gimbal::ControlMode::Primary);

        if(gimbal_result != mavsdk::Gimbal::Result::Success)
        {
            std::cerr << "Could not take gimbal control: " << gimbal_result << '\n';
            throw std::runtime_error("Could not take gimbal control");
        }

        _srvStartGimbalControl = this->create_service<std_srvs::srv::Trigger>("gimbal/start_control", std::bind(&GimbalControl::cbStartGimbalControl, this, _1, _2));
    }

    void GimbalControl::cbStartGimbalControl(const std::shared_ptr<std_srvs::srv::Trigger::Request> aRequest, const std::shared_ptr<std_srvs::srv::Trigger::Response> aResponse)
    {
        std::cout << "Set yaw mode to lock to a specific direction...\n";

        mavsdk::Gimbal::Result gimbal_result = _gimbal.get()->set_mode(mavsdk::Gimbal::GimbalMode::YawLock);

        if(gimbal_result != mavsdk::Gimbal::Result::Success)
        {
            std::cerr << "Could not set to lock mode: " << gimbal_result << '\n';
            throw std::runtime_error("Could not set to lock mode");
        }

        std::cout << "Look North...\n";
        _gimbal.get()->set_pitch_and_yaw(0.0f, 0.0f);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        std::cout << "Look East...\n";
        _gimbal.get()->set_pitch_and_yaw(0.0f, 90.0f);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        std::cout << "Look South...\n";
        _gimbal.get()->set_pitch_and_yaw(0.0f, 180.0f);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        std::cout << "Look West...\n";
        _gimbal.get()->set_pitch_and_yaw(0.0f, -90.0f);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        std::cout << "Set yaw mode to follow...\n";
        gimbal_result = _gimbal.get()->set_mode(mavsdk::Gimbal::GimbalMode::YawFollow);

        if(gimbal_result != mavsdk::Gimbal::Result::Success)
        {
            std::cerr << "Could not set to follow mode: " << gimbal_result << '\n';
            throw std::runtime_error("Could not set to follow mode");
        }

        std::cout << "And center first...\n";
        _gimbal.get()->set_pitch_and_yaw(0.0f, 0.0f);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        std::cout << "Tilt gimbal down...\n";
        _gimbal.get()->set_pitch_and_yaw(-90.0f, 0.0f);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        std::cout << "Tilt gimbal back up...\n";
        _gimbal.get()->set_pitch_and_yaw(0.0f, 0.0f);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        std::cout << "Slowly tilt up ...\n";
        _gimbal.get()->set_pitch_rate_and_yaw_rate(10.0f, 0.0f);
        std::this_thread::sleep_for(std::chrono::seconds(4));

        std::cout << "Back to horizontal...\n";
        _gimbal.get()->set_pitch_and_yaw(0.0f, 0.0f);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        std::cout << "Pan to the right...\n";
        _gimbal.get()->set_pitch_and_yaw(0.0f, 90.0f);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        std::cout << "Back to the center...\n";
        _gimbal.get()->set_pitch_and_yaw(0.0f, 0.0f);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        std::cout << "Pan slowly to the left...\n";
        _gimbal.get()->set_pitch_rate_and_yaw_rate(0.0f, -10.0f);
        std::this_thread::sleep_for(std::chrono::seconds(4));

        std::cout << "Back to the center...\n";
        _gimbal.get()->set_pitch_and_yaw(0.0f, 0.0f);
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    
    std::shared_ptr<mavsdk::System> GimbalControl::getSystem(mavsdk::Mavsdk& aMavsdk)
    {
        std::cout << "Waiting to discover system...\n";
        auto prom = std::promise<std::shared_ptr<mavsdk::System>>{};
        auto fut = prom.get_future();

        // We wait for new systems to be discovered, once we find one that has an
        // autopilot, we decide to use it.
        aMavsdk.subscribe_on_new_system([&aMavsdk, &prom]() {
            auto system = aMavsdk.systems().back();

            if(system->has_autopilot())
            {
                std::cout << "Discovered autopilot\n";

                // Unsubscribe again as we only want to find one system.
                aMavsdk.subscribe_on_new_system(nullptr);
                prom.set_value(system);
            }
        });

        // We usually receive heartbeats at 1Hz, therefore we should find a
        // system after around 3 seconds max, surely.
        if(fut.wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
        {
            std::cerr << "No autopilot found.\n";
            return nullptr;
        }

        // Get discovered system now.
        return fut.get();
    }
}

int main(int argc, char *argv[])
{
    std::cout << "Starting OffboardFlier node..." << std::endl;

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    try
    {
        rclcpp::spin(std::make_shared<gimbal::GimbalControl>());
    }
    catch(const std::runtime_error& e)
    {
        std::cout << "Exception: " << e.what() << std::endl;
    }
    catch(...)
    {

    }

    rclcpp::shutdown();
    
    return 0;
}