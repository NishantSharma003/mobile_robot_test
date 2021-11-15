#include <mobile_robot_test/robot_hardware_interface.h>

ROBOTHardwareInterface::ROBOTHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {
    init();
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_=5;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
	
	pub = nh_.advertise<mobile_robot_test::custom_cmd>("/joints_to_aurdino",10);
        client = nh_.serviceClient<mobile_robot_test::custom_service_message>("/read_joint_state");
	
    non_realtime_loop_ = nh_.createTimer(update_freq, &ROBOTHardwareInterface::update, this);
}

ROBOTHardwareInterface::~ROBOTHardwareInterface() {
}

void ROBOTHardwareInterface::init() {

	for(int i=0; i<2; i++)
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
    


void ROBOTHardwareInterface::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}



void ROBOTHardwareInterface::read() {

	joint_read.request.req=1.0;
	
	if(client.call(joint_read))
	{
            left_motor_pos+= angles::from_degrees(joint_read.response.res[0]); 
            joint_position_[0]=left_motor_pos;    

	    
            right_motor_pos+= angles::from_degrees(joint_read.response.res[1]);
            joint_position_[1]=right_motor_pos;    
	}
	else
	{
	}
        

}

void ROBOTHardwareInterface::write(ros::Duration elapsed_time) {
   
    velocityJointSaturationInterface.enforceLimits(elapsed_time); 
     
       int velocity_left,velocity_right;	

        velocity_left=(int)angles::to_degrees(joint_velocity_command_[0]);
        joints_pub.l_vel_cmd = velocity_left;

       //if(left_prev_cmd!=velocity_left)
       //{   
       // joints_pub.l_vel_cmd = velocity_left;
       //}
       
       velocity_right=(int)angles::to_degrees(joint_velocity_command_[1]);
       joints_pub.r_vel_cmd = velocity_right;
       //if(right_prev_cmd!=velocity_right)
       //{   
       // joints_pub.r_vel_cmd = velocity_right;
       //}

       ROS_INFO("joint_velocity_command_[0]=%.2f velocity_left=%d", joint_velocity_command_[0],velocity_left);
       ROS_INFO("joint_velocity_command_[1]=%.2f velocity_right=%d", joint_velocity_command_[1],velocity_right);

       pub.publish(joints_pub);
		
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "mobile_robot_test_hardware_interface");
    ros::NodeHandle nh;
    //ros::AsyncSpinner spinner(4);  
    ros::MultiThreadedSpinner spinner(2); // Multiple threads for controller service callback and for the Service client callback used to get the feedback from ardiuno
    ROBOTHardwareInterface ROBOT(nh);
    //spinner.start();
    spinner.spin();
    //ros::spin();
    return 0;
}
