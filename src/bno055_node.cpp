#include "ros_bno055/bno055_driver.h"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <ros_bno055/OrientationEuler.h>
#include <ros_bno055/Gravity.h>

namespace bno055 {
class Bno055Node {
public:
  Bno055Node(const ros::NodeHandle& nh_priv) : nh_priv_(nh_priv) {
    nh_priv_.param("operation_mode", operation_mode, (std::string)"imu");
    nh_priv_.param("power_mode", power_mode, (std::string)"normal");
    nh_priv_.param("device", i2c_bus, (std::string)"/dev/i2c-3");
    nh_priv_.param("address", i2c_addr, (int)40);

    // Initialize ROS publishers and ROS topics
    imu_pub_ = nh_priv_.advertise<sensor_msgs::Imu>("imu/data", 1);
    euler_pub_ = nh_priv_.advertise<ros_bno055::OrientationEuler>("orientation_euler", 1);
    mag_pub_ = nh_priv_.advertise<sensor_msgs::MagneticField>("magnetic_field", 1);
    temp_pub_ = nh_priv_.advertise<sensor_msgs::Temperature>("temperature", 1);
    grv_pub_ = nh_priv_.advertise<ros_bno055::Gravity>("gravity", 1);
  }

  // Initialize BNO055 sensor
  void init() {
    bno055_driver_.initI2c(i2c_bus.c_str(), i2c_addr);
    bno055_driver_.reset();

    selfCalibrate();

    bno055_driver_.setOperationMode(operation_mode.c_str());
 }
  
  // Check if BNO055 sensor is calibrated.
  bool isCalibrated() {
    switch(bno055_driver_.getOprMode()) {
	case 0x00:
    	    return true;
	case 0x01:
    	    if (bno055_driver_.data_.calib_stat_acc_ == 3) return true;
	    break;
	case 0x02:
	    if (bno055_driver_.data_.calib_stat_mag_ == 3) return true;
	    break;
	case 0x03:
	    if (bno055_driver_.data_.calib_stat_gyr_ == 3) return true;
	    break;
	case 0x04:
	    if (bno055_driver_.data_.calib_stat_acc_ == 3 && 
		bno055_driver_.data_.calib_stat_mag_ == 3) return true;
	    break;
	case 0x05:
	    if (bno055_driver_.data_.calib_stat_acc_ == 3 && 
		bno055_driver_.data_.calib_stat_gyr_ == 3) return true;
	    break;
	case 0x06:
	    if (bno055_driver_.data_.calib_stat_mag_ == 3 && 
		bno055_driver_.data_.calib_stat_gyr_ == 3) return true;
	    break;
	case 0x07:
	    if (bno055_driver_.data_.calib_stat_acc_ == 3 && 
		bno055_driver_.data_.calib_stat_mag_ == 3 && 
		bno055_driver_.data_.calib_stat_gyr_ == 3) return true;
	    break;
	case 0x08:
	    if (bno055_driver_.data_.calib_stat_acc_ == 3 && 
		bno055_driver_.data_.calib_stat_gyr_ == 3) return true;
	    break;
	case 0x09:
	    if (bno055_driver_.data_.calib_stat_acc_ == 3 && 
		bno055_driver_.data_.calib_stat_mag_ == 3) return true;
	    break;
	case 0x0A:
	    if (bno055_driver_.data_.calib_stat_acc_ == 3 && 
		bno055_driver_.data_.calib_stat_mag_ == 3) return true;
	    break;
	case 0x0B:
	    if (bno055_driver_.data_.calib_stat_acc_ == 3 && 
		bno055_driver_.data_.calib_stat_mag_ == 3 && 
		bno055_driver_.data_.calib_stat_gyr_ == 3) return true;
	    break;
	case 0x0C:
	    if (bno055_driver_.data_.calib_stat_acc_ == 3 && 
		bno055_driver_.data_.calib_stat_mag_ == 3 && 
		bno055_driver_.data_.calib_stat_gyr_ == 3) return true;
	    break;
    }

    return false;
}
 
  // Self-calibrate BNO055 sensor.
  void selfCalibrate() {
    ROS_INFO("Self calibrating...");
    if (bno055_driver_.loadCalib() < 0) ROS_ERROR("Failed to load calibration offset and radius data.");
    if (bno055_driver_.getCalibStat() < 0) ROS_ERROR("Failed to get calibration status data.");
    //if (bno055_driver_.getCalibOffset() < 0) ROS_ERROR("Failed to get calibration offset data.");
    //if (bno055_driver_.getCalibRadius() < 0) ROS_ERROR("Failed to get calibration radius data.");
  }

  // Publish ROS msgs
  void publishData() {
    //if (bno055_driver_.getAcc() < 0) ROS_ERROR("Failed to get accelerometer data.");
    //if (bno055_driver_.getMag() < 0) ROS_ERROR("Failed to get magnometer data.");
    if (bno055_driver_.getGyr() < 0) ROS_ERROR("Failed to get gyroscope data.");
    if (bno055_driver_.getEul() < 0) ROS_ERROR("Failed to get euler angles data.");
    if (bno055_driver_.getQua() < 0) ROS_ERROR("Failed to get quaternions data.");
    if (bno055_driver_.getLia() < 0) ROS_ERROR("Failed to get linear acceleration data.");
    if (bno055_driver_.getGrv() < 0) ROS_ERROR("Failed to get gravity vector data.");
    if (bno055_driver_.getTemp() < 0) ROS_ERROR("Failed to get temperature data.");
   
    // Construct ROS messages.
    ros::Time time_stamp = ros::Time::now();

    imu_msg_.header.stamp = time_stamp;
    imu_msg_.orientation.x = bno055_driver_.data_.qua_x_; 
    imu_msg_.orientation.y = bno055_driver_.data_.qua_y_;
    imu_msg_.orientation.z = bno055_driver_.data_.qua_z_;
    imu_msg_.orientation.w = bno055_driver_.data_.qua_w_;
    imu_msg_.angular_velocity.x = bno055_driver_.data_.gyr_x_;
    imu_msg_.angular_velocity.y = bno055_driver_.data_.gyr_y_;
    imu_msg_.angular_velocity.z = bno055_driver_.data_.gyr_z_;
    imu_msg_.linear_acceleration.x = bno055_driver_.data_.lia_x_;
    imu_msg_.linear_acceleration.y = bno055_driver_.data_.lia_y_;
    imu_msg_.linear_acceleration.z = bno055_driver_.data_.lia_z_;
    
    mag_msg_.header.stamp = time_stamp;
    mag_msg_.magnetic_field.x = bno055_driver_.data_.mag_x_;
    mag_msg_.magnetic_field.y = bno055_driver_.data_.mag_y_;
    mag_msg_.magnetic_field.z = bno055_driver_.data_.mag_z_;

    temp_msg_.header.stamp = time_stamp;
    temp_msg_.temperature = bno055_driver_.data_.temp_;

    euler_msg_.header.stamp = time_stamp;
    euler_msg_.heading = bno055_driver_.data_.eul_heading_;
    euler_msg_.roll = bno055_driver_.data_.eul_roll_;
    euler_msg_.pitch = bno055_driver_.data_.eul_pitch_;

    grv_msg_.header.stamp = time_stamp;
    grv_msg_.x = bno055_driver_.data_.grv_x_;
    grv_msg_.y = bno055_driver_.data_.grv_y_;
    grv_msg_.z = bno055_driver_.data_.grv_z_;
    
    imu_pub_.publish(imu_msg_);
    mag_pub_.publish(mag_msg_);
    temp_pub_.publish(temp_msg_);
    euler_pub_.publish(euler_msg_);
    grv_pub_.publish(grv_msg_);
  }

  ~Bno055Node() {}
private:
  int i2c_addr;
  std::string operation_mode;
  std::string power_mode;
  std::string i2c_bus;

  ros::NodeHandle nh_priv_;
  ros::Publisher imu_pub_;
  ros::Publisher mag_pub_;
  ros::Publisher temp_pub_;
  ros::Publisher euler_pub_;
  ros::Publisher grv_pub_;
  sensor_msgs::Imu imu_msg_;
  sensor_msgs::MagneticField mag_msg_;
  sensor_msgs::Temperature temp_msg_;
  ros_bno055::OrientationEuler euler_msg_;
  ros_bno055::Gravity grv_msg_;

  bno055::Bno055Driver bno055_driver_;
};
} // namespace bno055


int main(int argc, char* argv[]) {
  ros::init(argc, argv, "bno055_node");
  ros::NodeHandle nh_priv("~");
  bno055::Bno055Node bno055_node(nh_priv);

  int param_rate;
  nh_priv.param("loop_rate", param_rate, (int)100);
  int calib_rate;
  nh_priv.param("calib_rate", calib_rate, (int)1);

  ros::Rate loop_rate(param_rate);
  ros::Rate calibration_rate(calib_rate);
   
  bno055_node.init();

  while (ros::ok()) {
    // Always check if BNO055 sensor is calibrated.
    if (bno055_node.isCalibrated()) bno055_node.publishData();
    else {
      ROS_WARN("Sensor not calibrated. Running self-calibration sequence...");
      bno055_node.selfCalibrate();
      calibration_rate.sleep();
    }
      
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
