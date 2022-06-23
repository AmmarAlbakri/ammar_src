#include <mobile_robot_autonomous_navigation/robot_hardware_interface.h>

void ROBOTHardwareInterface::rightTicksCallback(const std_msgs::Float32 &msg)
{
    _Float32 x = msg.data;
    right_motor_pos += angles::from_degrees((double)x);
}

void ROBOTHardwareInterface::leftTicksCallback(const std_msgs::Float32 &msg)
{
    _Float32 x = msg.data;
    left_motor_pos += angles::from_degrees((double)x);
}

ROBOTHardwareInterface::ROBOTHardwareInterface(ros::NodeHandle &nh) : nh_(nh)
{
    init();

    right_speed_pub = nh.advertise<std_msgs::Int32>("right_vel", 1000);
    left_speed_pub = nh.advertise<std_msgs::Int32>("left_vel", 1000);

    right_sub = nh.subscribe("right_ticks", 1000, &ROBOTHardwareInterface::rightTicksCallback, this);
    left_sub = nh.subscribe("left_ticks", 1000, &ROBOTHardwareInterface::leftTicksCallback, this);

    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_ = 100;
    ros::Duration update_freq = ros::Duration(1.0 / loop_hz_);

    non_realtime_loop_ = nh_.createTimer(update_freq, &ROBOTHardwareInterface::update, this);
}

ROBOTHardwareInterface::~ROBOTHardwareInterface()
{
}

void ROBOTHardwareInterface::init()
{

    for (int i = 0; i < 2; i++)
    {
        // Create joint state interface
        hardware_interface::JointStateHandle jointStateHandle(joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
        joint_state_interface_.registerHandle(jointStateHandle);

        // Create velocity joint interface
        hardware_interface::JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_[i]);
        velocity_joint_interface_.registerHandle(jointVelocityHandle);

        // Create Joint Limit interface
        joint_limits_interface::JointLimits limits;
        joint_limits_interface::getJointLimits(joint_name_[i], nh_, limits);
        joint_limits_interface::VelocityJointSaturationHandle jointLimitsHandle(jointVelocityHandle, limits);
        velocityJointSaturationInterface.registerHandle(jointLimitsHandle);
    }

    // Register all joints interfaces
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&velocityJointSaturationInterface);
}

void ROBOTHardwareInterface::update(const ros::TimerEvent &e)
{
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void ROBOTHardwareInterface::read()
{
    joint_position_[0] = left_motor_pos;
    joint_position_[1] = right_motor_pos;

    // ROS_INFO("Reading from left motor %f", left_motor_pos);
    // ROS_INFO("Reading from right motor %f", right_motor_pos);
}

void ROBOTHardwareInterface::write(ros::Duration elapsed_time)
{

    velocityJointSaturationInterface.enforceLimits(elapsed_time);

    int velocity, result;

    velocity = (int)angles::to_degrees(joint_velocity_command_[0]);

    // ROS_INFO("joint_velocity_command_[0]=%.2f velocity=%d  B1=%d B2=%d", joint_velocity_command_[0],velocity,wbuff[0],wbuff[1]);

    // if (left_prev_cmd != velocity)
    // {
    // result = left_motor.writeData(wbuff,2);
    std_msgs::Int32 msg;
    msg.data = velocity;
    left_speed_pub.publish(msg);
    // ROS_INFO("Writen successfully to left motor: %d", velocity);
    //     left_prev_cmd = velocity;
    // }

    velocity = (int)angles::to_degrees(joint_velocity_command_[1]);

    // ROS_INFO("joint_velocity_command_[0]=%.2f velocity=%d  B1=%d B2=%d", joint_velocity_command_[0],velocity,wbuff[0],wbuff[1]);

    // if (right_prev_cmd != velocity)
    // {
    msg.data = velocity;
    // result = right_motor.writeData(wbuff,2);
    right_speed_pub.publish(msg);
    // ROS_INFO("Writen successfully to right motor: %d", velocity);
    //     right_prev_cmd = velocity;
    // }
}