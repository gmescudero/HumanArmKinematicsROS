/**
 * @file hak_oz_static_calib.cpp
 * @author German Moreno Escudero
 * @brief This ROS node subscribes to LPMS IMU data coming from OpenZen library and computes arm position joints 
 *      through an online calibration process.
 * @version 0.1
 * @date 2022-11-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h"
#include "hak_package/RecordElbowAngles.h"

extern "C"{
#include "launch.h"
#include "general.h"
#include "imu.h"
#include "calib.h"
#include "arm_kin.h"
#include "vector3.h"
}

#define TOPIC_SUB_1 "/s1_imu/data" // First sensor topic
#define TOPIC_SUB_2 "/s2_imu/data" // Second sensor topic
#define TOPIC_PUB "/joints_set" // Rviz topic
#define NODE_NAME "hak_pub_oz_trac_node" // Node name
#define ROS_RATE (100) // Loop rate (in Hz)

typedef enum CALIB_STEPS_ENUM {
  NOT_DONE = (0),
  GATHERING_DATA,
  ZEROING,
  DONE
} CALIB_STEPS;

class Hak {
private:
  ERROR_CODE status; // The error code of the HAK library
  sensor_msgs::JointState jointsMsg; // Standard joints positions to be published
  int imusNum; // Number of IMUs connected
  double timestamp; // Timestamp in ms
  double elbowRotFE[3] = {0,0,1}; // Rotation vector 1 of the elbow from the sensor coordinates
  double elbowRotPS[3] = {1,0,0}; // Rotation vector 2 of the elbow from the sensor coordinates
  Quaternion q_sensors[2]; // Sensor quaternions
  bool newData; // New data available

  CALIB_STEPS calibrationStep; // Calibrating flag
  double calibrationStartTime; // Timestamp when starting calibration

  double minObservationVelocity; // Threshold to consider the velocity enough
  double referenceCalibrationError; // Calibration error after finishing first calibration procedure
  double calibrationError; // Current calibration error

  /**
   * @brief Hak procedure setup
   * 
   */
  void _hakSetup() {
    /* Initialize HAK library */
    if (RET_OK == status) status = hak_initialize(false,false);
  }

  /**
   * @brief Go over the calibration process
   * 
   */
  void _hakCalibration() {
    /* Update the observations buffer */
    if (RET_OK == status) status = cal_gn2_observations_from_database_update(minObservationVelocity);

    switch (calibrationStep) {
      case NOT_DONE: 
        /* Start data gathering */
        ROS_INFO("\n\n\t\tPerform arbitrary motions of the elbow including flexion/extension and pronation/supination\n");
        calibrationStep = GATHERING_DATA;
        calibrationStartTime = timestamp;
        break;
      case GATHERING_DATA:
        if (CALIB_TWO_ROT_AXES_WINDOW <= db_field_buffer_current_size_get(DB_CALIB_OMEGA,0)) {
          /* Perform calibration algorithm */
          ROS_INFO("\n\n\t\tData gathered\n");
          calibrationStep = ZEROING;
          calibrationStartTime = timestamp;
          ROS_INFO("\n\n\t\tStand in T-Pose to calibrate\n");
          if (RET_OK == status) status = cal_gn2_two_rot_axes_calib(elbowRotFE,elbowRotPS);
          if (RET_OK == status) {
            /* Calculate minimum velocity for recalibration observations */
            DB_BUFFER_STATS stats = db_field_buffer_stats_compute(DB_CALIB_OMEGA,0);
            double diff[3];
            double diff_norm;
            for (int i = 0; i < 3; i++) {
              diff[i] = stats.max[i] - stats.min[i];
            }
            if (RET_OK == status) status = vector3_norm(stats.mean,&diff_norm);
            if (RET_OK == status) minObservationVelocity = diff_norm*0.1;
          }
          /* Compute reference calibration error */
          if (RET_OK == status) status = cal_gn2_root_mean_square(elbowRotFE,elbowRotPS, &referenceCalibrationError);
        }
        else if (2000 < timestamp - calibrationStartTime) {
          /* Log current progress in data gathering */
          ROS_INFO("Current observations count: %d/%d",
            db_field_buffer_current_size_get(DB_CALIB_OMEGA,0),CALIB_TWO_ROT_AXES_WINDOW);
          calibrationStartTime = timestamp;
        }
        break;
      case ZEROING:
        /* Set the zero position */
        calibrationStep = DONE;
        calibrationStartTime = timestamp;
        if (RET_OK == status) status = arm_elbow_angles_zero(
          0.0,0.0,q_sensors[0],q_sensors[1],elbowRotFE,elbowRotPS);
        if (RET_OK == status) ROS_INFO("\n\n\t\tZero position set\n");
        else                  ROS_ERROR("Failed to perform zero procedure");
        ROS_INFO("\tRotation vector 1:[%f,%f,%f]", elbowRotFE[0],elbowRotFE[1], elbowRotFE[2]);
        ROS_INFO("\tRotation vector 2:[%f,%f,%f]", elbowRotPS[0],elbowRotPS[1], elbowRotPS[2]);
        ROS_INFO("\tMin observations velocity: %f",minObservationVelocity);
        ROS_INFO("\tReference calibration error: %f",referenceCalibrationError);
        break;
      case DONE:
        /* Check current error against reference and correct calibration when needed */
        if (RET_OK == status) status = cal_gn2_root_mean_square(elbowRotFE,elbowRotPS, &calibrationError);
        ROS_INFO("Current calibration error: %f (reference: %f)",calibrationError,referenceCalibrationError);
        if (RET_OK == status && (calibrationError > referenceCalibrationError || 5000 < timestamp - calibrationStartTime)) {
          calibrationStartTime = timestamp;
          status = cal_gn2_two_rot_axes_calib_correct(elbowRotFE,elbowRotPS);
          if (RET_OK == status) ROS_INFO("\n\n\t\tOnline recalibration performed <%f,%f,%f> <%f,%f,%f>\n",
            elbowRotFE[0],elbowRotFE[1], elbowRotFE[2],elbowRotPS[0],elbowRotPS[1], elbowRotPS[2]);
          else                  ROS_ERROR("Failed to perform online recalibration");
        } 
        break;
      default:
        ROS_ERROR("Unknown calibration state");
        break;
    }
  }

  /**
   * @brief Retrieve joint angles from database and fill joints message
   * 
   */
  void _jointsMsgFromDbSet() {
    double shoulder[3];
    double elbow[3];

    /* Compute current shoulder angles */
    // if (RET_OK == status) {
    //   status = arm_shoulder_angles_compute(q_sensors[0],shoulder);
    //   if (RET_OK != status) ROS_ERROR("Failed to compute shoulder angles");
    // }
    /* Compute current elbow angles */
    if (RET_OK == status) {
      double z_vect[] = { 0,0,1};
      double x_vect[] = {1,0,0};
      status = arm_elbow_angles_from_rotation_vectors_get(
      q_sensors[0],q_sensors[1],elbowRotFE,elbowRotPS,elbow);
      if (RET_OK != status) ROS_ERROR("Failed to compute elbow angles");
    }
    /* Fill the joint message */
    if (RET_OK == status) {
      jointsMsg.position.clear();
      // jointsMsg.position.push_back( shoulder[SH_ABDUCTION]);
      // jointsMsg.position.push_back(-shoulder[SH_FLEXION]);
      // jointsMsg.position.push_back( shoulder[SH_ROTATION]);
      jointsMsg.position.push_back( 0.0);
      jointsMsg.position.push_back( 0.0);
      jointsMsg.position.push_back( 0.0);
      jointsMsg.position.push_back( elbow[ALPHA_FE]);
      jointsMsg.position.push_back( elbow[GAMMA_PS]);
      // ROS_INFO("Carrying: %f",elbow[BETA_CARRYING]);
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
            Find a way to detect/avoid that swapping. Last time checked 10/11/2022 */
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
      Quaternion_copy(&q_tmp,&q_sensors[index]);
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

  /**
   * @brief This ROS service returns to the caller with the elbow angles at the calling time
   * 
   * @param req (input) request descriptor
   * @param res (output) response descriptor
   * @return true if the elbow angles are correctly read from the database
   * @return false otherwise
   */
  bool _srv_recordElbowAngles(hak_package::RecordElbowAngles::Request  &req,
                              hak_package::RecordElbowAngles::Response &res)
  {
    double elbow[3];
    if (RET_OK == status) status = db_read(DB_ARM_ELBOW_ANGLES,0,elbow);
    if (RET_OK == status) {
      res.timestamp = ros::Time::now();
      res.fe   = elbow[ALPHA_FE];
      res.ps   = elbow[GAMMA_PS];
      res.beta = elbow[BETA_CARRYING];
      ROS_INFO("Recorded elbow angles: fe <%f>, ps <%f>, beta <%f>",res.fe,res.ps,res.beta);
      return true;
    }
    return false;
  }

public:
  /**
   * @brief Construct the Hak object
   * 
   * @param argc (input) Number of arguments given to the node
   * @param arg (input) Arguments given to the node
   */
  Hak(int argc, char **argv):status(RET_OK),imusNum(2),newData(false),calibrationStep(NOT_DONE),minObservationVelocity(0.0) {
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
    // Create service to record elbow angles
    ros::ServiceServer service = nh.advertiseService("elbowAngles", &Hak::_srv_recordElbowAngles, this);

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
        // Go over the calibration FSM
        _hakCalibration();

        if (RET_OK == status && DONE == calibrationStep) {
          // Retrieve the shoulder and elbow angles from the database and set the joints vector
          _jointsMsgFromDbSet();
          pub.publish(jointsMsg);
        }

        // If iteration not executed reset error code 
        if (RET_NO_EXEC == status) status = RET_OK;
        // If any error has been detected shut down the process
        else if (RET_OK != status) ros::shutdown();
        
        // Reset new data flag
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