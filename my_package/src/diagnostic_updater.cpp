  
 #include <diagnostic_updater/diagnostic_updater.h>
 #include <std_msgs/Bool.h>
 #include <diagnostic_updater/publisher.h>
 #include "ros/ros.h"
 #include "std_msgs/String.h"
 #include "sensor_msgs/LaserScan.h"
 #include "sensor_msgs/Imu.h"

 // ERROR defined in windows.h causes name collision, undefine the macro to fix the issue
 #ifdef ERROR
 #undef ERROR
 #endif
  
 double time_to_launch;

 std_msgs::Header header;
 float time_inc = 0;
 float scan_time = 0;
 float angle_min = 0;
 float angle_max = 0;
 float angle_inc = 0;
 float range_min = 0;
 float range_max = 0;
//  float ranges[];
//  float intensities[];
// Variables for frequency measurement use:
 double freq = 0;
 double callback_time = 0;
 double last_callback_time = 0;
 double time_since_last_callback = 0;
 bool not_first_callback = false;
 // LiDAR Average frequency
 double avg_freq_param = 20;  // Take and average 20 freq samples
 double temp = 0;
 double avg_freq = 0;
 int count = 0;
 // LiDAR Stats
 double last_seq = 0;
 double cur_seq = 0;

 void callBack(const sensor_msgs::LaserScan::ConstPtr& msg){
   header = msg->header;
   time_inc = msg->time_increment;
   scan_time = msg->scan_time;
   angle_min = msg->angle_min;
   angle_max = msg->angle_max;
   angle_inc = msg->angle_increment;
   range_min = msg->range_min;
   range_max = msg->range_max;
   last_seq = header.seq;


   callback_time = ros::Time::now().toSec();
   if (not_first_callback){
     time_since_last_callback = ros::Time::now().toSec() - last_callback_time;
   }
   last_callback_time = callback_time;
   not_first_callback = true;
   freq = 1/time_since_last_callback;

   // Calculate average frequency
   temp += freq;
   count++;
   if(count == avg_freq_param){
     avg_freq = temp/avg_freq_param;
     count = 0;
     temp = 0;
   }
  //  ROS_INFO("Average LiDAR frequency = %f", avg_freq);
 }

void lidar_status_check(diagnostic_updater::DiagnosticStatusWrapper &stat){
  if(last_seq == cur_seq){
    //ERROR
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,"LiDAR not working");
  }else{
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK,"LiDAR OK");
    last_seq = cur_seq;
    cur_seq = header.seq;
  }
  stat.add("LiDAR_last_seq",last_seq);
  stat.add("LiDAR_cur_seq",cur_seq);
}

void lidar_freq_check(diagnostic_updater::DiagnosticStatusWrapper &stat)
 {
   if (avg_freq < 19)
     stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "LiDAR Low Frequency Warning");
   else if(avg_freq > 21){
     stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "LiDAR High Frequency Warning");
   }else
     stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "LiDAR Frequency OK");
  
   stat.add("Lidar avg_freq", avg_freq);
 }
  
 int main(int argc, char **argv)
 {
   ros::init(argc, argv, "diagnostic_updater_ex");
   
   ros::NodeHandle n;
   
   // The Updater class advertises to /diagnostics, and has a
   // ~diagnostic_period parameter that says how often the diagnostics
   // should be published.
   diagnostic_updater::Updater updater;

   updater.setHardwareIDf("LiDAR_IMU_test");
  
  //  updater.broadcast(0, "Doing important initialization stuff.");
  
   ros::Publisher pub1 = n.advertise<std_msgs::String>("topic", 1);
   ros::Subscriber pub = n.subscribe("/r2000_node/scan",1000, callBack);
   
  //  ros::Duration(2).sleep(); // It isn't important if it doesn't take time.

   updater.add("Frequency Status Check",lidar_freq_check);
   updater.add("LiDAR Status Check",lidar_status_check);
  
   // If we know that the state of the node just changed, we can force an
   // immediate update.
   updater.force_update();  
   // We can remove a task by refering to its name.
  //  if (!updater.removeByName("Bound check"))
  //    ROS_ERROR("The Bound check task was not found when trying to remove it.");
   while (n.ok())
   {
     std_msgs::Bool msg;
    //  ros::Duration(0.1).sleep();
     
     // Calls to pub1 have to be accompanied by calls to pub1_freq to keep
     // the statistics up to date.
     msg.data = false;
     pub1.publish(msg);
     // tick() Signals that a publication has occurred
     ros::spinOnce();

     // We can call updater.update whenever is convenient. It will take care
     // of rate-limiting the updates.
     updater.update();
   }  
   return 0; 
 }