#include <amr_control/robot_hardware_interface.h>
// namesapce i2c_ros

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mobile_robot_hardware_interface");
    ros::NodeHandle nh;

    // ros::AsyncSpinner spinner(4);
    ros::MultiThreadedSpinner spinner(4); // Multiple threads for controller service callback and for the Service client callback used to get the feedback from ardiuno
    ROBOTHardwareInterface ROBOT(nh);
    // spinner.start();
    spinner.spin();
    // ros::spin();
    return 0;
}
