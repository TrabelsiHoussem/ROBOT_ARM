#include <ros.h>
#include <rospy_tutorials/Floats.h>
#include <moveo_moveit/Floats_array.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>

#include <Servo.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

//Pinout for Ramps 1.4 with Arduino Mega

//Joint 1
#define E1_STEP_PIN        36
#define E1_DIR_PIN         34
#define E1_ENABLE_PIN      30

// Joint 2
#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19

// Joint 3
#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
#define Y_MIN_PIN          14
#define Y_MAX_PIN          15

// Joint 4
#define X_STEP_PIN         16//54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38

// Joint 5 
#define E0_STEP_PIN        26
#define E0_DIR_PIN         28
#define E0_ENABLE_PIN      24



//Accel Stepper for controlling joints in parallel mode
AccelStepper joint1(1,E1_STEP_PIN, E1_DIR_PIN);
AccelStepper joint2(1,Z_STEP_PIN, Z_DIR_PIN);
AccelStepper joint3(1,Y_STEP_PIN, Y_DIR_PIN);
AccelStepper joint4(1,X_STEP_PIN, X_DIR_PIN);
AccelStepper joint5(1, E0_STEP_PIN, E0_DIR_PIN);


//Servo Motor for the gripper 
Servo gripper;
//Stepper Motors
MultiStepper steppers;

//AS5600 Analogic encoderpin
int feedback_joint1_pin=A0;
int feedback_joint2_pin=A1;
int feedback_joint3_pin=A2;
int feedback_joint4_pin=A3;
int feedback_joint5_pin=A4;
int feedback_joint6_pin=A5;






//ros::NodeHandle_<ArduinoHardware, 2, 2, 512, 512> nh;

//creation of ros node
ros::NodeHandle nh;
//int cur_pos[3] = {0, 0, 0};

//revolutions=0
long revolutions_joint1 = 0;   // number of revolutions the encoder has made
long revolutions_joint2 = 0;
long revolutions_joint3 = 0;
long revolutions_joint4 = 0;
long revolutions_joint5 = 0;
long revolutions_joint6 = 0;


// raw value from AS5600



//output   // the calculated value the encoder is at
double feedback1;
double feedback2;
double feedback3;
double feedback4;
double feedback5;
double feedback6;
//last_output
long  last_feedback1;
long  last_feedback2;
long  last_feedback3;
long  last_feedback4;
long  last_feedback5;
long  last_feedback6;

//position
double joint_position1=0;
double joint_position2=0;
double joint_position3=0;
double joint_position4=0;
double joint_position5=0;
double joint_position6=0;

int joint_status = 0;
int joint_step[6];



//get angle from ros and covertit to stepper motor commands
void arm_cb( const rospy_tutorials::Floats& cmd_msg) {
  //nh.loginfo("Command Received");
    joint_status=1;
    joint_step[0] = cmd_msg.data[0]*0.03*1600;
    joint_step[1] = cmd_msg.data[1]*0.03*1600;
    joint_step[2] = cmd_msg.data[2]*0.03*1600;
    joint_step[3] = cmd_msg.data[3]*0.03*1600;
    joint_step[4] = cmd_msg.data[4]*0.03*1600;
    joint_step[5] = cmd_msg.data[5]*0.03*1600;
    // steps=-90*0.03*400
     
  
}
//get angle from ros and convert it to servo gripper command
void gripper_cb( const std_msgs::UInt16& cmd_msg){
  gripper.write(cmd_msg.data); // Set servo angle, should be from 0-180  
 // digitalWrite(13, HIGH-digitalRead(13));  // Toggle led  
}

//send data to ros to inform it with joint_position 
void feedback_cb(const moveo_moveit::Floats_array::Request & req, moveo_moveit::Floats_array::Response & res)
{
  // Simulate function running for a non-deterministic amount of time



  //res.res_length = 7;
  feedback1 = analogRead(feedback_joint1_pin)*1.46;


  if ((last_feedback1 - feedback1) > 500 )        // check if a full rotation has been made
    revolutions_joint1 ++;
  if ((last_feedback1 - feedback1) < -500 )
    revolutions_joint1 --;
  joint_position1 = revolutions_joint1 * 1024 +  feedback1;
  
  nh.loginfo(revolutions_joint1);
  //res.res[0]=(readservo1-100) * (180.0/325.0);
  //res.res[0]=position*0.033;
  res.res[0] = joint_position1*0.033;
  res.res[1] = 0;
  res.res[2] = 0;
  res.res[3] = 0;
  res.res[4] = 0;
  res.res[5]=0;
  res.res[6]=0;
  last_feedback1 = feedback1;
 
  return;

}









ros::Subscriber<std_msgs::UInt16> gripper_sub("gripper_angle", gripper_cb); //subscribes to gripper position

ros::Subscriber<rospy_tutorials::Floats> arm_sub("/joints_to_aurdino", arm_cb);
ros::ServiceServer<moveo_moveit::Floats_array::Request, moveo_moveit::Floats_array::Response> server("/read_joint_state", &feedback_cb);


void setup() {
  //nh.getHardware()->setBaud(57600);
  nh.initNode();
  //  nh.subscribe(sub);
  nh.advertiseService(server);

  nh.subscribe(arm_sub);
  //nh.subscribe(gripper_sub);

  feedback1 = analogRead(feedback_joint1_pin);
  last_feedback1 = feedback1;
  joint_position1 = feedback1;

  feedback2 = analogRead(feedback_joint2_pin);
  last_feedback2 = feedback2;
  joint_position2 = feedback2;


  feedback3 = analogRead(feedback_joint3_pin);
  last_feedback2 = feedback3;
  joint_position3 = feedback3;

  feedback4 = analogRead(feedback_joint4_pin);
  last_feedback2 = feedback4;
  joint_position4 = feedback4;

  feedback5= analogRead(feedback_joint5_pin);
  last_feedback2 = feedback5;
  joint_position5 = feedback5;

  feedback6= analogRead(feedback_joint6_pin);
  last_feedback2 = feedback6;
  joint_position6 = feedback6;
  /*
  joint1.setMaxSpeed(1500);
  joint2.setMaxSpeed(750);
  joint3.setMaxSpeed(2000);
  joint4.setMaxSpeed(500);
  joint5.setMaxSpeed(1000);
  */
  joint1.setMaxSpeed(400);
  joint2.setMaxSpeed(400);
  joint3.setMaxSpeed(400);
  joint4.setMaxSpeed(400);
  joint5.setMaxSpeed(400);
  steppers.addStepper(joint1);
  steppers.addStepper(joint2);
  steppers.addStepper(joint3);
  steppers.addStepper(joint4);
  steppers.addStepper(joint5);
  gripper.attach(11);



}

void loop() {
 if (joint_status == 1) // If command callback (arm_cb) is being called, execute stepper command
  { 
    long positions[5];  // Array of desired stepper positions must be long
    positions[0] = -joint_step[0]; // negated since the real robot rotates in the opposite direction as ROS
    positions[1] = -joint_step[1]; 
    positions[2] = joint_step[2]; 
    positions[3] = joint_step[3]; 
    positions[4] = -joint_step[4]; 

    // Publish back to ros to check if everything's correct
    //msg.data=positions[4];
    //steps.publish(&msg);

    steppers.moveTo(positions);
    nh.spinOnce();
    steppers.runSpeedToPosition(); // Blocks until all are in position
    gripper.write(joint_step[5]);  // move gripper after manipulator reaches goal   
  }
  joint_status = 0;
  nh.spinOnce();
  
  
}
