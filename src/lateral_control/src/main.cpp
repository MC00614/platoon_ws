#include "control.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    int truck_id = 0;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--truck_id") {
            if (i + 1 < argc) {
                truck_id = std::stoi(argv[i + 1]);
            } else {
                std::cerr << "No truck ID specified after --truck_id" << std::endl;
                return 1;
            }
        }
    }

    auto control_node = std::make_shared<Control>(truck_id);
    rclcpp::spin(control_node);
    rclcpp::shutdown();

    return 0;
}