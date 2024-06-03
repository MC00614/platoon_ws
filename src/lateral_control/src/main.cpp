#include "control.h"

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Control>());
	rclcpp::shutdown();
	return 0;
}