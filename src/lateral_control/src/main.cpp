#include "control.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.arguments(std::vector<std::string>(argv, argv + argc));
    std::string truck_id_arg = "--truck_id=";
    int truck_id = 0;
    for (int i = 0; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg.find(truck_id_arg) == 0)
        {
            std::string truck_id_str = arg.substr(truck_id_arg.length());
            truck_id = std::stoi(truck_id_str);
        }
    }

    auto control_node = std::make_shared<Control>(truck_id);
    rclcpp::spin(control_node);
    rclcpp::shutdown();

    return 0;
}