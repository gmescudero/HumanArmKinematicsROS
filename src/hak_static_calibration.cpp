#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

extern "C"{
#include "launch.h"
#include "general.h"
#include "imu.h"
#include "calib.h"
#include "arm.h"
}

#define TOPIC_PUB "/joints_set" // Rviz topic
#define NODE_NAME "hak_pub_static_calib_node" // Node name
#define ROS_RATE (10) // Loop rate (in Hz)

class Hak {
private:
  ERROR_CODE status; // The error code of the HAK library
  sensor_msgs::JointState jointsMsg; // Standard joints positions to be published
  int imusNum; // Number of IMUs connected
  ImuData data[2]; // Imus data
  Quaternion read_quats[2];

  /**
   * @brief Hak procedure setup
   * 
   */
  void _hakSetup() {
    /* Initialize HAK library */
    if (RET_OK == status) status = hak_initialize(false,false);

    /* Look for IMU sensors and initialize the required of them */
    if (RET_OK == status) {
      ROS_INFO("Initializating %d IMU sensors",imusNum);
      status = imu_batch_search_and_initialize(imusNum);
      if (RET_OK != status) ROS_ERROR("Failed to setup IMU sensors");
    }

    /* Read IMUs data for the given amount of time */
    if (RET_OK == status) {
      ROS_INFO("Starting loop gathering IMU sensors data to calibrate rotation axes");
      ROS_INFO(" -> [USER]: Perform arbitrary motions of the elbow including flexion/extension and pronation/supination");
      
      /* Start IMU reading callbacks */
      for (int imu = 0; RET_OK == status && imu < imusNum; imu++) {
        status = imu_read_callback_attach(imu, 0==imu);
        if (RET_OK != status) ROS_ERROR("Failed to initialize reading callback for IMU sensor %d",imu);
      }
      sleep_s(2);
    }

    /* Retrieve current IMU data */
    if (RET_OK == status) {
      ROS_INFO("Calibrate IMU sensors");
      ROS_INFO(" -> [USER]: Stand in T-pose to calibrate IMU orientations");
      if (RET_OK == status) sleep_s(4);
      /* Reset IMUs offset */
      if (RET_OK == status) {
          status = imu_orientation_offset_reset();
          if (RET_OK != status) ROS_ERROR("Failed to reset IMU sensors orientation");
      }
      /* Read data for calibration*/
      if (RET_OK == status) status = imu_batch_read(imusNum, data);
      if (RET_OK != status) ROS_ERROR("Failed to read IMUs data");
    }

    /* Set calibration data */
    if (RET_OK == status) {
      Quaternion known_quats[2] = { /* Quats for T-pose */
          {.w = 1.0, .v={0.0, 0.0, 0.0}},
          {.w = 1.0, .v={0.0, 0.0, 0.0}},
      };
      for (int imu = 0; imu < imusNum; imu++) {
        quaternion_from_float_buffer_build(data[imu].q, &read_quats[imu]);
      }
      cal_static_imu_quat_calibration_set(known_quats, read_quats);
      /* Get the current quaternions */
      double z_vect[] = {0,0,1};
      double x_vect[] = {-1,0,0};
      if (RET_OK == status) status = cal_static_imu_quat_calibrated_data_get(read_quats);
      if (RET_OK == status) status = arm_elbow_angles_zero(0.0,0.0,read_quats[0],read_quats[1],z_vect,x_vect);
      if (RET_OK == status) ROS_INFO(" -> [USER]: Zero position set ");
      else                  ROS_ERROR("Failed to perform zero procedure");
    }

    /* Start IMU reading callbacks */
    for (int imu = 0; RET_OK == status && imu < imusNum; imu++) {
      status = imu_read_callback_attach(imu, 0==imu);
      if (RET_OK != status) ROS_ERROR("Failed to initialize reading callback for IMU sensor %d",imu);
      else sleep_s(2);
    }
  }

  /**
   * @brief Retrieve joint angles from database and fill joints message
   * 
   */
  void _jointsMsgFromDbSet() {
    double shoulder[3];
    double elbow[3];
    /* Get joints angles from the database */
    if (RET_OK == status) status = db_read(DB_ARM_SHOULDER_ANGLES,0,shoulder);
    if (RET_OK == status) status = db_read(DB_ARM_ELBOW_ANGLES,0,elbow);
    /* Fill the joint message */
    if (RET_OK == status) {
      jointsMsg.position.clear();
      jointsMsg.position.push_back(shoulder[SH_ABDUCTION]);
      jointsMsg.position.push_back(shoulder[SH_FLEXION]);
      jointsMsg.position.push_back(shoulder[SH_ROTATION]);
      jointsMsg.position.push_back(elbow[ALPHA_FE]);
      jointsMsg.position.push_back(elbow[GAMMA_PS]);
    }
  }

public:
  /**
   * @brief Construct the Hak object
   * 
   * @param argc (input) Number of arguments given to the node
   * @param arg (input) Arguments given to the node
   */
  Hak(int argc, char **argv):status(RET_OK),imusNum(2) {
    // Initialize ROS
    ros::init(argc, argv, NODE_NAME);
    // Create a node handler
    ros::NodeHandle nh;
    // Set the loop rate
    ros::Rate       rh(ROS_RATE);
    // Create the joints publisher
    ros::Publisher  pub = nh.advertise<sensor_msgs::JointState>(TOPIC_PUB, 10);

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

      /* Get current sensor data from database */
      if (RET_OK == status) {
        status = cal_static_imu_quat_calibrated_data_get(read_quats);
        if (RET_OK != status) ROS_ERROR("Failed to retrieve calibrated imus");
      }

      /* Compute current shoulder angles */
      if (RET_OK == status) {
        status = arm_shoulder_angles_compute(read_quats[0],NULL);
        if (RET_OK != status) ROS_ERROR("Failed to compute shoulder angles");
      }

      /* Compute current elbow angles */
      if (RET_OK == status) {
        double z_vect[] = { 0,0,1};
        double x_vect[] = {-1,0,0};
        status = arm_elbow_angles_from_rotation_vectors_get(
          read_quats[0],read_quats[1],z_vect,x_vect,NULL);
        if (RET_OK != status) ROS_ERROR("Failed to compute elbow angles");
      }

      /* Publish the current joint angles */
      if (RET_OK == status) {
        _jointsMsgFromDbSet();
        if (RET_OK == status) pub.publish(jointsMsg);
      }

      db_field_print(DB_IMU_QUATERNION,0);
      db_field_print(DB_IMU_QUATERNION,1);

      /* If iteration not executed reset error code */
      if (RET_NO_EXEC == status) status = RET_OK;

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