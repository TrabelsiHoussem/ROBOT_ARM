#include <moveo_moveit/robot_hardware_interface.h>
#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/transmission_interface.h>


ROBOTHardwareInterface::ROBOTHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {
    init();
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_=16;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
	
	pub = nh_.advertise<rospy_tutorials::Floats>("/joints_to_aurdino",1);
	client = nh_.serviceClient<moveo_moveit::Floats_array>("/read_joint_state");	
    non_realtime_loop_ = nh_.createTimer(update_freq, &ROBOTHardwareInterface::update, this);
}

ROBOTHardwareInterface::~ROBOTHardwareInterface() {
}

void ROBOTHardwareInterface::init() {
    
    num_joints_=7;
	joint_names_[0]="Joint_1";	
	joint_names_[1]="Joint_2";
	joint_names_[2]="Joint_3";
    joint_names_[3]="Joint_4";
	joint_names_[4]="Joint_5";
	joint_names_[5]="Gripper_Idol_Gear_Joint";
	joint_names_[6]="Gripper_Servo_Gear_Joint";
	

    for (int i = 0; i < num_joints_; ++i) {

         // Create joint state interface
        hardware_interface::JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
        joint_state_interface_.registerHandle(jointStateHandle);
        // Create position joint interface
        hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
        
        position_joint_interface_.registerHandle(jointPositionHandle);
   
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
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
	    
		joint_position_[0]=angles::from_degrees(joint_read.response.res[0]);
		joint_position_[1]=angles::from_degrees(joint_read.response.res[1]);
		joint_position_[2]=angles::from_degrees(joint_read.response.res[2]);
		joint_position_[3]=angles::from_degrees(joint_read.response.res[3]);
		joint_position_[4]=angles::from_degrees(joint_read.response.res[4]);
		joint_position_[5]=angles::from_degrees(joint_read.response.res[5]);
		joint_position_[6]=angles::from_degrees(joint_read.response.res[6]);
		
		ROS_INFO("Receiving  j1: %.2f, j2: %.2f, j3: %.2f, j4: %.2f,j5: %.2f,j6: %.2f   ",joint_read.response.res[0],joint_read.response.res[1], joint_read.response.res[2] , joint_read.response.res[3], joint_read.response.res[4]);
		
	}	
    else
    {
    	joint_position_[0]=0;
        joint_position_[1]=0;
        joint_position_[2]=0;
        joint_position_[3]=0;
        joint_position_[4]=0;
        joint_position_[5]=0;
        joint_position_[6]=0;
        ROS_INFO("Service not found ");
    }
        

}

void ROBOTHardwareInterface::write(ros::Duration elapsed_time) {
    
	joints_pub.data.clear();
	
    joints_pub.data.push_back((angles::to_degrees(joint_position_command_[0])));
	/*joints_pub.data.push_back((angles::to_degrees(joint_position_command_[1])));
	joints_pub.data.push_back((angles::to_degrees(joint_position_command_[2])));
	joints_pub.data.push_back((angles::to_degrees(joint_position_command_[3])));
	joints_pub.data.push_back((angles::to_degrees(joint_position_command_[4])));
	joints_pub.data.push_back((angles::to_degrees(joint_position_command_[5])));
	joints_pub.data.push_back((angles::to_degrees(joint_position_command_[6])));*/
    joints_pub.data.push_back(angles::to_degrees(0));
	joints_pub.data.push_back(angles::to_degrees(0));
	joints_pub.data.push_back(angles::to_degrees(0));
	joints_pub.data.push_back(angles::to_degrees(0));
	joints_pub.data.push_back(angles::to_degrees(0));
	joints_pub.data.push_back(angles::to_degrees(0));
	
    ROS_INFO("Publishing j1: %.2f, j2: %.2f, j3: %.2f",joints_pub.data[0],joints_pub.data[1],joints_pub.data[2]);
	pub.publish(joints_pub);	
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_hardware_interface");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2); // 2 threads for controller service and for the Service client used to get the feedback from ardiuno
    //spinner.start();
    ROBOTHardwareInterface ROBOT(nh);
    //ros::spin();
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
