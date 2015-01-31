#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <circle/linear_mv.h>
#include <circle/n_images.h>

//Subscribe Messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>



// Image Transport
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// Used to display OPENCV images
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// the US Digital USB4 device support
#include "/usr/include/libusdusb4.h"


#define COMMAND_POS_0  0x00
#define COMMAND_POS_1  0x01
#define COMMAND_POS_2  0x02
#define COMMAND_POS_3  0x03
#define COMMAND_POS_4  0x04
#define COMMAND_POS_5  0x05
#define COMMAND_POS_6  0x06
#define COMMAND_POS_7  0x07
#define COMMAND_POS_8  0x08
#define COMMAND_POS_9  0x09
#define COMMAND_POS_10 0x0A
#define COMMAND_POS_11 0x0B
#define COMMAND_POS_12 0x0C
#define COMMAND_POS_13 0x0D
#define COMMAND_POS_14 0x0E
#define COMMAND_POS_15 0x0F
#define COMMAND_START    (1<<6)
#define COMMAND_RESET    (1<<5)
#define COMMAND_SERVO_ON (1<<4)

#define COMMAND_HOME     COMMAND_POS_0
#define COMMAND_FAR      COMMAND_POS_15
#define COMMAND_RELATIVE COMMAND_POS_1

#define POSITION_COMPLETE (1<<4)


using std::string;


class linearRailNode
{
private:
  // Define Node
  ros::NodeHandle node_;
  image_transport::ImageTransport *it_;
  image_transport::Subscriber image_sub_;
  ros::ServiceServer mv_home_sptr_;
  ros::ServiceServer mv_far_sptr_;
  ros::ServiceServer mv_rel_sptr_;
  ros::ServiceServer mv_to_sptr_;
  ros::ServiceServer oneI_sptr_;
  ros::ServiceServer fdI_sptr_;
  ros::ServiceServer n_image_sptr_;
  ros::ServiceServer cycle_sptr_;
  ros::ServiceServer increment_dir_;
  std::string image_base_name_;
  std::string image_dir_name_;
  int collect_image_;
  int  image_number_;
  int  dir_number_;
public:
	
  explicit linearRailNode(const ros::NodeHandle& nh): node_(nh)
  {
    image_base_name_ = "test";
    image_dir_name_ = "images";
    image_number_ = 0;
    dir_number_ = 0;
    collect_image_ = 0;

    // do the subscription
    it_ = new image_transport::ImageTransport(node_);
    image_sub_ = it_->subscribe("CameraImage",1,&linearRailNode::imageCallback, this);
    mv_home_sptr_ = node_.advertiseService("LR_mv_home",&linearRailNode::mv_home, this);
    mv_far_sptr_ = node_.advertiseService("LR_mv_far",&linearRailNode::mv_far, this);
    mv_rel_sptr_ = node_.advertiseService("LR_mv_rel",&linearRailNode::mv_rel, this);
    mv_to_sptr_ = node_.advertiseService("LR_mv_to",&linearRailNode::mv_to, this);
    oneI_sptr_ = node_.advertiseService("LR_one_image",&linearRailNode::one_image, this);
    fdI_sptr_ = node_.advertiseService("LR_fifty_images",&linearRailNode::fifty_images, this);
    n_image_sptr_ = node_.advertiseService("LR_n_images",&linearRailNode::n_images, this);
    cycle_sptr_ = node_.advertiseService("LR_cycle",&linearRailNode::cycle, this);
    increment_dir_ = node_.advertiseService("LR_inc_dir",&linearRailNode::inc_dir, this);
    
    // Initialize the USB4 driver.
    short deviceCount = 0;
    int result = USB4_Initialize(&deviceCount);
    if(result == USB4_SUCCESS){
      printf("Successfully opened the USB4 encoder/io device\n");
    }
    char command=0; // 8 bits all off
    USB4_WriteOutputPortRegister(0, command); 
  };


  ~linearRailNode()
  { 
    delete(it_);
  };

  void imageCallback(const sensor_msgs::ImageConstPtr& image_msg);
  bool mv_home(std_srvs::Empty::Request& req, std_srvs::Empty::Response &rep);
  bool mv_far(std_srvs::Empty::Request& req, std_srvs::Empty::Response &rep);
  bool mv_rel(std_srvs::Empty::Request& req, std_srvs::Empty::Response &rep);
  bool mv_to(circle::linear_mv::Request& req, circle::linear_mv::Response &rep);
  bool one_image(std_srvs::Empty::Request& req, std_srvs::Empty::Response &rep);
  bool fifty_images(std_srvs::Empty::Request& req, std_srvs::Empty::Response &rep);
  bool n_images(circle::n_images::Request& req, circle::n_images::Response &rep);
  bool cycle(std_srvs::Empty::Request& req, std_srvs::Empty::Response &rep);
  bool inc_dir(std_srvs::Empty::Request& req, std_srvs::Empty::Response &rep);
  void wait_till_move_done(unsigned char cmd);
};

void linearRailNode::wait_till_move_done(unsigned char cmd)
{
  unsigned char move_done_bits;
  unsigned char command;
  bool move_done = false;

  do{
    unsigned char move_done_bits;
    USB4_ReadInputPortRegister(0,&move_done_bits);
    if(move_done_bits & POSITION_COMPLETE) move_done = true;
  }while(move_done == false);

  // shut off motion when done
  USB4_WriteOutputPortRegister(0, cmd); 
  sleep(1); // added to make sure stopped prior to collecting images
}


void linearRailNode::imageCallback(const sensor_msgs::ImageConstPtr& image_msg)
{
  //Use CV Bridge to convert images	
  cv_bridge::CvImagePtr image_ptr;
  image_ptr = cv_bridge::toCvCopy(image_msg,sensor_msgs::image_encodings::BGR8);
  
  if(collect_image_ >0){
    string nn = ros::this_node::getName();
    node_.getParam(nn + "/image_base_name",image_base_name_);
    node_.setParam(nn + "/image_base_name",image_base_name_);
    node_.getParam(nn + "/image_dir_name",image_dir_name_);
    node_.setParam(nn + "/image_dir_name",image_dir_name_);

    char image_number[10];
    sprintf(image_number,"%03d",image_number_);
    char dir_number[10];
    sprintf(dir_number,"D%03d",dir_number_);
    std::string image_name = image_dir_name_ 
      + "/" 
      + dir_number
      + "/" 
      + image_base_name_ 
      + image_number 
      + ".tiff";
    image_number_++;
    ROS_INFO("writing %s",image_name.c_str());
    imwrite(image_name.c_str(),image_ptr->image);
    collect_image_--;
  }

}


bool linearRailNode::mv_home(std_srvs::Empty::Request& req, std_srvs::Empty::Response &rep)
{
  ROS_INFO("RUNNING mv_home()");

  char command=0; // 8 bits
  command = COMMAND_HOME;
  USB4_WriteOutputPortRegister(0, command); 
  //sleep(1);
  command = COMMAND_HOME | COMMAND_SERVO_ON | COMMAND_START;
  USB4_WriteOutputPortRegister(0, command); 
  wait_till_move_done(COMMAND_HOME);
  
  return(true);
}


bool linearRailNode::mv_far(std_srvs::Empty::Request& req, std_srvs::Empty::Response &rep)
{
  ROS_INFO("RUNNING mv_far()");
  unsigned char command=0; // 8 bits
  command = COMMAND_FAR;
  USB4_WriteOutputPortRegister(0, command); 
  //sleep(1);
  command = COMMAND_FAR | COMMAND_SERVO_ON | COMMAND_START;
  USB4_WriteOutputPortRegister(0, command); 
  wait_till_move_done(COMMAND_FAR);
  return(true);

}

bool linearRailNode::mv_rel(std_srvs::Empty::Request& req, std_srvs::Empty::Response &rep)
{
  ROS_INFO("RUNNING mv_rel()");
  unsigned char command=0; // 8 bits
  command = COMMAND_RELATIVE;
  USB4_WriteOutputPortRegister(0, command); 
  //sleep(1);
  command = COMMAND_RELATIVE | COMMAND_SERVO_ON | COMMAND_START;
  USB4_WriteOutputPortRegister(0, command); 
  wait_till_move_done(COMMAND_RELATIVE);
  return(true);
}


bool linearRailNode::mv_to(circle::linear_mv::Request& req, circle::linear_mv::Response &rep)
{
  ROS_INFO("RUNNING mv_to(%d)",req.cmd);
  if(req.cmd>0x0f){
    return(false);
  }
  unsigned char command=0; // 8 bits
  command = req.cmd;
  USB4_WriteOutputPortRegister(0, command); 
  //sleep(1);
  command = req.cmd | COMMAND_SERVO_ON | COMMAND_START;
  USB4_WriteOutputPortRegister(0, command); 
  wait_till_move_done(req.cmd);
  return(true);
}

bool linearRailNode::one_image(std_srvs::Empty::Request& req, std_srvs::Empty::Response &rep)
{
  ROS_INFO("RUNNING oneimage()");
  collect_image_++;
}


bool linearRailNode::fifty_images(std_srvs::Empty::Request& req, std_srvs::Empty::Response &rep)
{
  ROS_INFO("RUNNING fifty_mages() ");
  collect_image_ += 50;
  return(true);
}

bool linearRailNode::n_images(circle::n_images::Request& req, circle::n_images::Response &rep)
{
  ROS_INFO("RUNNING n_mages(%d) ",req.num);
  collect_image_ += (int)req.num;
  return(true);
}

bool linearRailNode::cycle(std_srvs::Empty::Request& req, std_srvs::Empty::Response &rep)
{
  ROS_INFO("RUNNING cycle() ");
  inc_dir(req,rep);
  mv_rel(req,rep);
  fifty_images(req,rep);
  return(true);
}

bool linearRailNode::inc_dir(std_srvs::Empty::Request& req, std_srvs::Empty::Response &rep)
{
  ROS_INFO("RUNNING inc_dir() ");
  image_number_ = 0; // reset image number
  dir_number_++; // increment the directory
  return(true);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "linearRail");
  ros::NodeHandle n;
  linearRailNode LRN(n);
  ros::spin();
  return 0;
}

