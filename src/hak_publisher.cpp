#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "hak_package/RecordElbowAngles.h"

#define TOPIC_PUB "/joints_set"         // Rviz topic
#define NODE_NAME "hak_publisher_node"  // Node name
#define ROS_RATE (10)                   // Loop rate (in Hz)

extern "C"{
#include "launch.h"
#include "general.h"
#include "imu.h"
#include "calib.h"
#include "arm_kin.h"
#include "Quaternion.h"
}

class Hak {
private:
  ERROR_CODE status; // The error code of the HAK library
  sensor_msgs::JointState jointsMsg; // Standard joints positions to be published
  int imusNum; // Number of IMUs connected
  double elbowRotFE[3] = {0,0,1}; // Rotation vector 1 of the elbow from the sensor coordinates
  double elbowRotPS[3] = {1,0,0}; // Rotation vector 2 of the elbow from the sensor coordinates
  Quaternion q1; // Orientation quaternion of the arm sensor 
  Quaternion q2; // Orientation quaternion of the forearm sensor 

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

    /* Loop while gathering data for two axes calibration */
    if (RET_OK == status) {
      int observations;
      do {
        if (RET_OK == status) sleep_s(1);
        if (RET_OK == status) status = cal_gn2_observations_from_database_update(0.0);
        observations = db_field_buffer_current_size_get(DB_CALIB_OMEGA,0);
        ROS_INFO("Current observations count: %d/%d",observations,CALIB_TWO_ROT_AXES_WINDOW);
      } while (RET_OK == status && CALIB_TWO_ROT_AXES_WINDOW > observations);
      
      ROS_INFO(" -> [USER]: Finished recording calibration data");
    }

    /* Perform calibration algorithm */
    if (RET_OK == status) {
      ROS_INFO("Calibrating rotation two axes");

      status = cal_gn2_two_rot_axes_calib(elbowRotFE,elbowRotPS);
      if (RET_OK != status) ROS_ERROR("Failed to initialize reading callback for IMU sensors");
      else {
        ROS_INFO("Finished two axes calibration: ");
        ROS_INFO("\tRotation vector 1:[%f,%f,%f]", elbowRotFE[0],elbowRotFE[1], elbowRotFE[2]);
        ROS_INFO("\tRotation vector 2:[%f,%f,%f]", elbowRotPS[0],elbowRotPS[1], elbowRotPS[2]);
      }
    }

    /* Set the angles to zero in the current pose */
    if (RET_OK == status) {
      ROS_INFO("Set the zero point");
      ROS_INFO(" -> [USER]: Stand in a pose to be considered as zero in 5");
      for (int i = 4; RET_OK == status && i >= 0; i--) {
        sleep_s(1);
        ROS_INFO(" -> [USER]: %d",i);
        if (1 == i) status = imu_orientation_offset_set(1);
      }
      if (RET_OK == status) _imusQuatsFromDbSet();
      if (RET_OK == status) status = arm_elbow_angles_zero(0.0,0.0,q1,q2,elbowRotFE,elbowRotPS);
      if (RET_OK == status) ROS_INFO(" -> [USER]: Zero position set ");
      else                  ROS_ERROR("Failed to perform zero procedure");
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
      jointsMsg.position.push_back(-shoulder[SH_FLEXION]);
      jointsMsg.position.push_back(shoulder[SH_ROTATION]);
      jointsMsg.position.push_back(elbow[ALPHA_FE]);
      jointsMsg.position.push_back(-elbow[GAMMA_PS]);
    }
  }

  /**
   * @brief Retrieve quaternion orientations of each IMU sensor and fill quaternion attributes
   * 
   */
  void _imusQuatsFromDbSet() {
    // Get the current quaternions 
    double qBuff[4];
    if (RET_OK == status) status = db_read(DB_IMU_QUATERNION,0,qBuff);
    if (RET_OK == status) quaternion_from_buffer_build(qBuff, &q1);
    if (RET_OK == status) status = db_read(DB_IMU_QUATERNION,1,qBuff);
    if (RET_OK == status) quaternion_from_buffer_build(qBuff, &q2);   
    if (RET_OK != status) ROS_ERROR("Failed to retrieve current sensor quaternions");
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
  Hak(int argc, char **argv):status(RET_OK),imusNum(2) {
    // Initialize ROS
    ros::init(argc, argv, NODE_NAME);
    // Create a node handler
    ros::NodeHandle nh;
    // Set the loop rate
    ros::Rate       rh(ROS_RATE);
    // Create the joints publisher
    ros::Publisher  pub = nh.advertise<sensor_msgs::JointState>(TOPIC_PUB, 10);
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

      /* Get current sensor data from database */
      if (RET_OK == status) _imusQuatsFromDbSet();

      /* Compute current shoulder angles */
      if (RET_OK == status) {
        status = arm_shoulder_angles_compute(q1,NULL);
        if (RET_OK != status) ROS_ERROR("Failed to compute shoulder angles");
      }

      /* Compute current elbow angles */
      if (RET_OK == status) {
        status = arm_elbow_angles_from_rotation_vectors_get(q1,q2,elbowRotFE,elbowRotPS,NULL);
        if (RET_OK != status) ROS_ERROR("Failed to compute elbow angles");
      }

      /* Publish the current joint angles */
      if (RET_OK == status) {
        _jointsMsgFromDbSet();
        if (RET_OK == status) pub.publish(jointsMsg);
      }

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