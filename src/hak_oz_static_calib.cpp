/**
 * @file hak_oz_static_calib.cpp
 * @author German Moreno Escudero
 * @brief This ROS node subscribes to LPMS IMU data coming from OpenZen library and computes arm position joints 
 *      through a static calibration process.
 * @version 0.1
 * @date 2022-11-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h"

extern "C"{
#include "launch.h"
#include "general.h"
#include "imu.h"
#include "calib.h"
#include "arm.h"
}

#define TOPIC_SUB_1 "/s1_imu/data" // First sensor topic
#define TOPIC_SUB_2 "/s2_imu/data" // Second sensor topic
#define TOPIC_PUB "/joints_set" // Rviz topic
#define NODE_NAME "hak_pub_oz_static_calib_node" // Node name
#define ROS_RATE (100) // Loop rate (in Hz)

typedef enum CALIB_STEPS_ENUM {
  NOT_DONE = (0),
  WAITING,
  DONE
} CALIB_STEPS;

class Hak {
private:
  ERROR_CODE status; // The error code of the HAK library
  sensor_msgs::JointState jointsMsg; // Standard joints positions to be published
  sensor_msgs::Imu imuMsg1; // Standard IMU data retrieved from OpenZen first imu
  sensor_msgs::Imu imuMsg2; // Standard IMU data retrieved from OpenZen second imu
  int imusNum; // Number of IMUs connected
  double timestamp; // Timestamp in ms
  Quaternion calibratedQuats[2]; // Calibrated quaternions
  bool newData; // New data available
  CALIB_STEPS calibrationStep; // Calibrating flag
  double calibrationStartTime; // Timestamp when starting calibration

  /**
   * @brief Hak procedure setup
   * 
   */
  void _hakSetup() {
    /* Initialize HAK library */
    if (RET_OK == status) status = hak_initialize(false,false);
    /* Set the number of imus in calibration pack */
    if (RET_OK == status) cal_static_imu_quat_number_of_imus_set(imusNum);
  }

  /**
   * @brief Retrieve joint angles from database and fill joints message
   * 
   */
  void _jointsMsgFromDbSet() {
    double shoulder[3];
    double elbow[3];
    /* Compute current shoulder angles */
    if (RET_OK == status) {
      status = arm_shoulder_angles_compute(calibratedQuats[0],shoulder);
      if (RET_OK != status) ROS_ERROR("Failed to compute shoulder angles");
    }
    /* Compute current elbow angles */
    if (RET_OK == status) {
      double z_vect[] = { 0,0,1};
      double x_vect[] = {-1,0,0};
      status = arm_elbow_angles_from_rotation_vectors_get(
      calibratedQuats[0],calibratedQuats[1],z_vect,x_vect,elbow);
      if (RET_OK != status) ROS_ERROR("Failed to compute elbow angles");
    }
    /* Fill the joint message */
    if (RET_OK == status) {
      jointsMsg.position.clear();
      jointsMsg.position.push_back( shoulder[SH_ABDUCTION]);
      jointsMsg.position.push_back(-shoulder[SH_FLEXION]);
      jointsMsg.position.push_back( shoulder[SH_ROTATION]);
      jointsMsg.position.push_back( elbow[ALPHA_FE]);
      jointsMsg.position.push_back(-elbow[GAMMA_PS]);
    }
  }

  /**
   * @brief Update database with imus data
   * 
   * @param index (input) The imu index
   * @param msg (input) The imu data
   */
  void _imuDatabaseUpdate(int index, const sensor_msgs::Imu::ConstPtr& msg) {
    timestamp = 1e-6*(double)msg->header.stamp.toNSec();
    double acc[3]    = {msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z};
    double gyr[3]    = {msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z};
    double linAcc[3] = {msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z};
    double angVel[3] = {msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z};
    /* TODO: Sometimes, IMUs rotating axes are swapped for unknown reasons and mess with everything else that uses them. 
            Find a way to detect/avoid that swapping. */
    double quat[4]   = {msg->orientation.w,msg->orientation.x,msg->orientation.y,msg->orientation.z}; 
    // double quat[4]   = {msg->orientation.w,-msg->orientation.x,-msg->orientation.y,-msg->orientation.z}; 
    
    newData = true;
    if (RET_OK == status) status = db_write(DB_IMU_TIMESTAMP,           index, &timestamp);
    if (RET_OK == status) status = db_write(DB_IMU_ACCELEROMETER,       index, &acc);
    if (RET_OK == status) status = db_write(DB_IMU_GYROSCOPE,           index, &gyr);
    if (RET_OK == status) status = db_write(DB_IMU_LINEAR_ACCELERATION, index, &linAcc);
    if (RET_OK == status) status = db_write(DB_IMU_ANGULAR_VELOCITY,    index, &angVel);
    if (RET_OK == status) status = db_write(DB_IMU_QUATERNION,          index, &quat);
    if (RET_OK == status) status = db_write(DB_IMU_NEW_DATA,            index, &newData);
    if (RET_OK != status) ROS_ERROR("Failed to update database fileds with IMU%d data",index);
    
    /* Calibrate quaternions */
    if (RET_OK == status) {
      Quaternion q_tmp;
      quaternion_from_buffer_build(quat,&q_tmp);
      if (DONE == calibrationStep) {
        status = cal_static_imu_quat_calibration_apply(q_tmp,index,&calibratedQuats[index]);
      }
      else {
        Quaternion_copy(&q_tmp,&calibratedQuats[index]);
      }
    }
  }

  /**
   * @brief First imu topic subscriber callback
   * 
   * @param msg (input) The received Imu data
   */
  void _imu1DataGetCallback(const sensor_msgs::Imu::ConstPtr& msg){
    _imuDatabaseUpdate(0,msg);
  }
  /**
   * @brief Second imu topic subscriber callback
   * 
   * @param msg (input) The received Imu data
   */
  void _imu2DataGetCallback(const sensor_msgs::Imu::ConstPtr& msg){
    _imuDatabaseUpdate(1,msg);
  }

public:
  /**
   * @brief Construct the Hak object
   * 
   * @param argc (input) Number of arguments given to the node
   * @param arg (input) Arguments given to the node
   */
  Hak(int argc, char **argv):status(RET_OK),imusNum(2),newData(false),calibrationStep(NOT_DONE) {
    // Initialize ROS
    ros::init(argc, argv, NODE_NAME);
    // Create a node handler
    ros::NodeHandle nh;
    // Set the loop rate
    ros::Rate       rh(ROS_RATE);
    // Create the joints publisher
    ros::Publisher  pub = nh.advertise<sensor_msgs::JointState>(TOPIC_PUB, 10);
    // Create the imus data subscribers
    ros::Subscriber sub1 = nh.subscribe(TOPIC_SUB_1, 40, &Hak::_imu1DataGetCallback, this);
    ros::Subscriber sub2 = nh.subscribe(TOPIC_SUB_2, 40, &Hak::_imu2DataGetCallback, this);

    // Initialize ROS message
    jointsMsg.header.frame_id.assign("jointSetPointScheduler");
    jointsMsg.name.clear();
    jointsMsg.position.clear();

    jointsMsg.name.push_back("sh_z");
    jointsMsg.name.push_back("sh_y");
    jointsMsg.name.push_back("sh_x");
    jointsMsg.name.push_back("el_z");
    jointsMsg.name.push_back("el_x");

    // Setup HAK procedure before looping
    _hakSetup();

    while(ros::ok() && RET_OK == status) {
      if (true == newData) {
        if (NOT_DONE == calibrationStep) {
          /* Prepare calibration */
          ROS_INFO("\n\n\t\tStand in T-Pose to calibrate\n");
          calibrationStep = WAITING;
          calibrationStartTime = timestamp;
        }
        else if (WAITING == calibrationStep && 4000 < timestamp - calibrationStartTime) {
          /* Perform calibration */
          Quaternion known_quats[2] = { /* Quats for T-pose */
            {.w = 1.0, .v={0.0, 0.0, 0.0}},
            {.w = 1.0, .v={0.0, 0.0, 0.0}},
          };
          if (RET_OK == status) cal_static_imu_quat_calibration_set(known_quats, calibratedQuats);
          /* Get the current quaternions to zero the elbow angles*/
          double z_vect[] = { 0, 0, 1};
          double x_vect[] = {-1, 0, 0};
          if (RET_OK == status) status = cal_static_imu_quat_calibrated_data_get(calibratedQuats);
          if (RET_OK == status) status = arm_elbow_angles_zero(0.0,0.0,calibratedQuats[0],calibratedQuats[1],z_vect,x_vect);
          if (RET_OK == status) ROS_INFO("\n\n\t\tZero position set\n");
          else                  ROS_ERROR("Failed to perform zero procedure");
          if (RET_OK == status) calibrationStep = DONE;
        }

        /* Publish the current joint angles */
        if (RET_OK == status) {
          // Retrieve the shoulder and elbow angles from the database and set the joints vector
          _jointsMsgFromDbSet();
          if (RET_OK == status) pub.publish(jointsMsg);
        }

        /* If iteration not executed reset error code */
        if (RET_NO_EXEC == status) status = RET_OK;
        else if (RET_OK != status) ros::shutdown();

        /* Reset new data flag */
        if (RET_OK == status) newData = false;
      }

      // ROS callbacks and synchronization
      ros::spinOnce();
      rh.sleep();
    }
  }

  /**
   * @brief Destroy the Hak object
   * 
   */
  virtual ~Hak(){
    if (RET_OK != status) ROS_ERROR("Closing ROS node due to HAK error");
    status = hak_terminate();
    if (RET_OK != status) ROS_ERROR("Failed to clean all resources");
  }

  /**
   * @brief Get the Error Code object
   * 
   * @return ERROR_CODE 
   */
  ERROR_CODE getErrorCode(){return status;}
};

/************************************************************************/

/**
 * @brief Main entry point
 * 
 * @param argc (input) Number of arguments given to the node
 * @param arg (input) Arguments given to the node
 * @return int: 0 if success, error otherwise
 */
int main(int argc, char **argv) {
  // Create HAK handler object
  Hak hakHandler(argc,argv);
  // Finished HAK execution. Return final Error code
  return hakHandler.getErrorCode();
}