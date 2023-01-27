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
#include "hak_package/Recalib.h"

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
#define NODE_NAME "hak_node" // Node name
#define ROS_RATE (50) // Loop rate (in Hz)
#define IMUS_NUM (2) // Number of imu sensors

typedef enum CALIB_METHOD_ENUM {
  STATIC,
  TWO_AXES,
  TWO_AXES_ONLINE,
  NUMBER_OF_CHOICES
} CALIB_METHOD;

typedef enum IMU_SOURCE_ENUM {
  LPSENSOR,
  OPENZEN
} IMU_SOURCE;

typedef enum CALIB_STEPS_ENUM {
  NOT_DONE = (0),
  GATHERING_DATA,
  ZEROING,
  DONE
} CALIB_STEPS;

class Hak {
private:
  ERROR_CODE status; // The error code of the HAK library
  CALIB_METHOD configCalibMethod; // The configurated calibration method
  IMU_SOURCE configImuSource; // The configured imu source

  sensor_msgs::JointState jointsMsg; // Standard joints positions to be published
  double elbowRotFE[3] = {0,0,1}; // Rotation vector 1 of the elbow from the sensor coordinates
  double elbowRotPS[3] = {1,0,0}; // Rotation vector 2 of the elbow from the sensor coordinates
  Quaternion q_sensors[2]; // Sensor quaternions
  bool newData; // New data available

  CALIB_STEPS calibrationStep; // Calibrating flag
  ros::Time calibrationStartTime; // Timestamp when starting calibration

  double minObservationVelocity; // Threshold to consider the velocity enough
  double referenceCalibrationError; // Calibration error after finishing first calibration procedure
  double calibrationError; // Current calibration error

  /**
   * @brief Set internal configuration from rosparm file
   * 
   */
  void _hakConfigSet(std::string calib_method, std::string imu_source) {
    // Check calibration method from configuration
    if      (0 == calib_method.compare("static"))        configCalibMethod = STATIC;
    else if (0 == calib_method.compare("twoAxes"))       configCalibMethod = TWO_AXES;
    else if (0 == calib_method.compare("twoAxesOnline")) configCalibMethod = TWO_AXES_ONLINE;
    else {
      ROS_WARN("Invalid configuration of calibration method. Defaults to \"static\"");
    }
    // Check sensor source from configuration
    if      (0 == imu_source.compare("lpsensor")) configImuSource = LPSENSOR;
    else if (0 == imu_source.compare("openzen"))  configImuSource = OPENZEN;
    else {
      ROS_WARN("Invalid configuration of sensor source. Defaults to \"lpsensor\"");
    }
  }

  /**
   * @brief Hak procedure setup
   * 
   */
  void _hakSetup() {
    /* Initialize HAK library */
    if (RET_OK == status) status = hak_initialize(false,false);

    if (LPSENSOR == configImuSource) {
      /* Look for IMU sensors and initialize the required of them */
      if (RET_OK == status) {
        ROS_INFO("Initializating %d IMU sensors",IMUS_NUM);
        status = imu_batch_search_and_initialize(IMUS_NUM);
        if (RET_OK != status) ROS_ERROR("Failed to setup IMU sensors");
      }
      /* Reset orientation offset */
      if (RET_OK == status) status = imu_orientation_offset_reset();
      /* Start IMU reading callbacks */
      if (RET_OK == status) {
        for (int imu = 0; RET_OK == status && imu < IMUS_NUM; imu++) {
          status = imu_read_callback_attach(imu, false);
          if (RET_OK != status) ROS_ERROR("Failed to initialize reading callback for IMU sensor %d",imu);
        }
        sleep_s(2);
      }
    }
    else {
      /* Set the number of imus in calibration pack */
      if (RET_OK == status) {
        cal_static_imu_quat_number_of_imus_set(IMUS_NUM);
      }
      sleep_s(1);
    }
  }

  /**
   * @brief At the start of the process ask the user for calibration instructions and start gathering data
   * 
   */
  void _hakCalibrationNotDoneHandle() {
    if (TWO_AXES == configCalibMethod || TWO_AXES_ONLINE == configCalibMethod) {
      ROS_INFO("\n\n\t\tPerform arbitrary motions of the elbow including flexion/extension and pronation/supination\n");
      calibrationStep = GATHERING_DATA;
      calibrationStartTime = ros::Time::now();
    } 
    else { /* STATIC calibration */
      calibrationStep = ZEROING;
      calibrationStartTime = ros::Time::now();
      ROS_INFO("\n\n\t\tStand in T-Pose to calibrate\n");
    }
  }
  
  /**
   * @brief Gather data of imu motion until buffer is full
   * 
   */
  void _hakCalibrationGatheringDataHandle() {
    /* Update the observations buffer */
    if (RET_OK == status) status = cal_gn2_observations_from_database_update(minObservationVelocity);

    if (CALIB_TWO_ROT_AXES_WINDOW <= db_field_buffer_current_size_get(DB_CALIB_OMEGA,0)) {
      /* Perform calibration algorithm */
      ROS_INFO("\n\n\t\tData gathered\n");
      calibrationStep = ZEROING;
      calibrationStartTime = ros::Time::now();
      ROS_INFO("\n\n\t\tStand in T-Pose to calibrate\n");
      // if (RET_OK == status) status = cal_gn2_two_rot_axes_calib(elbowRotFE,elbowRotPS);
      if (RET_OK == status) {
        elbowRotFE[0] = 0.0; elbowRotFE[1] = 0.0; elbowRotFE[2] = 1.0; 
        elbowRotPS[0] = 1.0; elbowRotPS[1] = 0.0; elbowRotPS[2] = 0.0; 
      }
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
    else if (2 < (ros::Time::now()- calibrationStartTime).toSec()) {
      /* Log current progress in data gathering */
      ROS_INFO("Current observations count: %d/%d",
        db_field_buffer_current_size_get(DB_CALIB_OMEGA,0),CALIB_TWO_ROT_AXES_WINDOW);
      calibrationStartTime = ros::Time::now();
    }
  }

  /**
   * @brief Set the zero position of the calibration
   * 
   */
  void _hakCalibrationZeroingHandle() {
    if (5 < (ros::Time::now()- calibrationStartTime).toSec()) {
      calibrationStep = DONE;
      calibrationStartTime = ros::Time::now();
      // Static calibration
      Quaternion known_quats[2] = { /* Quats for T-pose */
        {.w = 1.0, .v={0.0, 0.0, 0.0}},
        {.w = 1.0, .v={0.0, 0.0, 0.0}},
      };
      for (int i = 0; RET_OK == status && i < IMUS_NUM; i++) {
          // Retrieve Imu quaternion data
          double q_buff[4];
          Quaternion q_tmp;
          status = db_read(DB_IMU_QUATERNION, i, q_buff);
          if (RET_OK == status) quaternion_from_buffer_build(q_buff, &q_sensors[i]);
      }
      if (RET_OK == status) cal_static_imu_quat_calibration_set(known_quats, q_sensors);

      // Zero the elbow angles
      if (RET_OK == status) status = arm_elbow_angles_zero(
        0.0,0.0,q_sensors[0],q_sensors[1],elbowRotFE,elbowRotPS);
      // Report
      if (RET_OK == status) {
        ROS_INFO("\n\n\t\tZero position set\n");
        if (TWO_AXES == configCalibMethod || TWO_AXES_ONLINE == configCalibMethod) {
          ROS_INFO("\tRotation vector 1:[%f,%f,%f]", elbowRotFE[0],elbowRotFE[1], elbowRotFE[2]);
          ROS_INFO("\tRotation vector 2:[%f,%f,%f]", elbowRotPS[0],elbowRotPS[1], elbowRotPS[2]);
        }
        if (TWO_AXES_ONLINE == configCalibMethod) {
          ROS_INFO("\tMin observations velocity: %f",minObservationVelocity);
          ROS_INFO("\tReference calibration error: %f",referenceCalibrationError);
        }
      }
      else ROS_ERROR("Failed to perform zero procedure");
    }
  }

  /**
   * @brief Perform autocalibration when required
   * 
   */
  void _hakCalibrationDoneHandle() {
      // Static calibration
      if (RET_OK == status) status = cal_static_imu_quat_calibrated_data_get(q_sensors);
      if (RET_OK == status) ROS_DEBUG("Calibrated quats: <%f,%f,%f,%f> <%f,%f,%f,%f>",
        q_sensors[0].w,q_sensors[0].v[0],q_sensors[0].v[1],q_sensors[0].v[2],
        q_sensors[1].w,q_sensors[1].v[0],q_sensors[1].v[1],q_sensors[1].v[2]
      );
      if (RET_OK != status) ROS_ERROR("Failed to retrieve calibrated imus");
      // Permorm recalibration 
      if (TWO_AXES_ONLINE == configCalibMethod) { // /s1_imu/reset_heading
      double oldCalibError = calibrationError;
      /* Update the observations buffer */
      if (RET_OK == status) status = cal_gn2_observations_from_database_update(minObservationVelocity);
      /* Compute current calibration error*/
      if (RET_OK == status) status = cal_gn2_root_mean_square(elbowRotFE,elbowRotPS, &calibrationError);
      /* Check error and recalibrate if needed */
      if (RET_OK == status) {
        if (calibrationError > referenceCalibrationError){
          calibrationStartTime = ros::Time::now();
          status = cal_gn2_two_rot_axes_calib_correct(elbowRotFE,elbowRotPS);
          if (RET_OK == status) {
            ROS_INFO("\n\n\t\tOnline recalibration performed\n");
            ROS_INFO("\tCalibration error went from %f to %f (reference at %f)", 
                oldCalibError, calibrationError,referenceCalibrationError);
            ROS_INFO("\tRotation vector 1:[%f,%f,%f]", elbowRotFE[0],elbowRotFE[1], elbowRotFE[2]);
            ROS_INFO("\tRotation vector 2:[%f,%f,%f]", elbowRotPS[0],elbowRotPS[1], elbowRotPS[2]);
          }
          else {
            // Make the reference slowly update with newer error values
            referenceCalibrationError = referenceCalibrationError*0.97 + oldCalibError*0.02 + calibrationError*0.01;
          }
        }
        else ROS_ERROR("Failed to perform online recalibration");
      }
    } 
  }

  /**
   * @brief Go over the calibration process
   * 
   */
  void _hakCalibration() {
    switch (calibrationStep) {
      case NOT_DONE: 
        /* Start data gathering */
        _hakCalibrationNotDoneHandle();
        break;
      case GATHERING_DATA:
        /* Store data until buffer is full */
        _hakCalibrationGatheringDataHandle();
        break;
      case ZEROING:
        /* Set the zero position */
        _hakCalibrationZeroingHandle();
        break;
      case DONE:
        /* Check current error against reference and correct calibration when needed */
        _hakCalibrationDoneHandle();
        break;
      default:
        ROS_ERROR("Unknown calibration state");
        status = RET_ERROR;
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
    if (RET_OK == status) {
      status = arm_shoulder_angles_compute(q_sensors[0],shoulder);
      if (RET_OK != status) ROS_ERROR("Failed to compute shoulder angles");
    }
    /* Compute current elbow angles */
    if (RET_OK == status) {
      if (STATIC == configCalibMethod) {
        Quaternion q_relative = arm_quaternion_between_two_get(q_sensors[0],q_sensors[1]);
        Quaternion_toEulerZYX(&q_relative,elbow);
        status = db_write(DB_ARM_ELBOW_ANGLES,0,elbow);
      }
      else {
        status = db_read(DB_IMU_QUATERNION,0,&q_sensors[0]);
        if (RET_OK == status)status = db_read(DB_IMU_QUATERNION,1,&q_sensors[1]);
        if (RET_OK == status)status = arm_elbow_angles_from_rotation_vectors_get(
            q_sensors[0],q_sensors[1],elbowRotFE,elbowRotPS,elbow);
        if (RET_OK != status) ROS_ERROR("Failed to compute elbow angles");
      }
    }
    
    /* Fill the joint message */
    if (RET_OK == status) {
      jointsMsg.position.clear();
      jointsMsg.position.push_back(shoulder[SH_ABDUCTION]);
      jointsMsg.position.push_back(-shoulder[SH_FLEXION]);
      jointsMsg.position.push_back(shoulder[SH_ROTATION]);
      jointsMsg.position.push_back(elbow[ALPHA_FE]);
      jointsMsg.position.push_back(elbow[GAMMA_PS]);
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
    double timestamp = 1e-6*(double)msg->header.stamp.toNSec();
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
  }

  /**
   * @brief First imu topic subscriber callback
   * 
   * @param msg (input) The received Imu data
   */
  void _sub_imu1DataGetCallback(const sensor_msgs::Imu::ConstPtr& msg){
    _imuDatabaseUpdate(0,msg);
  }
  /**
   * @brief Second imu topic subscriber callback
   * 
   * @param msg (input) The received Imu data
   */
  void _sub_imu2DataGetCallback(const sensor_msgs::Imu::ConstPtr& msg){
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

  bool _srv_recalibrate(hak_package::Recalib::Request  &req,
                        hak_package::Recalib::Response &res)
  {
    calibrationStep = NOT_DONE;
    calibrationStartTime = ros::Time::now();
    _hakCalibrationNotDoneHandle();
    ROS_INFO("Recalibration started");
    return true;
  }

public:
  /**
   * @brief Construct the Hak object
   * 
   * @param argc (input) Number of arguments given to the node
   * @param arg (input) Arguments given to the node
   */
  Hak(int argc, char **argv):
  status(RET_OK),
  newData(false),
  calibrationStep(NOT_DONE),
  minObservationVelocity(0.0),
  configCalibMethod(STATIC),
  configImuSource(LPSENSOR) 
  {
    // Initialize ROS
    ros::init(argc, argv, NODE_NAME);
    // Create a node handler
    ros::NodeHandle nh;
    // Get parameters
    std::string calib_method; 
    std::string sensors_source;
    if (    nh.getParam("calib_method", calib_method) 
         && nh.getParam("sensors_source", sensors_source))
    {
      _hakConfigSet(calib_method,sensors_source);
    }
    else ROS_WARN("Failed to retrieve configuration parameters. Using defaults");
    // Set the loop rate
    ros::Rate       rh(ROS_RATE);
    // Create the joints publisher
    ros::Publisher  pub = nh.advertise<sensor_msgs::JointState>(TOPIC_PUB, 10);
    // Create the imus data subscribers
    ros::Subscriber sub1, sub2;
    if (OPENZEN == configImuSource) {
      ROS_INFO("Subscribe to Openzen topics for IMU data");
      sub1 = nh.subscribe(TOPIC_SUB_1, 40, &Hak::_sub_imu1DataGetCallback, this);
      sub2 = nh.subscribe(TOPIC_SUB_2, 40, &Hak::_sub_imu2DataGetCallback, this);
    }
    // Create service to record elbow angles
    ros::ServiceServer service1 = nh.advertiseService("elbowAngles", &Hak::_srv_recordElbowAngles, this);
    ros::ServiceServer service2 = nh.advertiseService("recalibrate", &Hak::_srv_recalibrate, this);

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
      // Check for new data
      if (RET_OK == status && false == newData) {
        status = db_read(DB_IMU_NEW_DATA, 0, &newData);
      }
      if (RET_OK == status && false == newData) {
        status = db_read(DB_IMU_NEW_DATA, 1, &newData);
      }
      if (RET_OK == status && true == newData) {
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
        if (RET_OK == status) {
          newData = false;
          db_write(DB_IMU_NEW_DATA,0,&newData);
          db_write(DB_IMU_NEW_DATA,1,&newData);
        }
      }
      else {
        ROS_WARN("No available data for current cycle");
      }

      // db_field_print(DB_ARM_ELBOW_ANGLES,0);
      // db_field_print(DB_IMU_QUATERNION,0);
      // db_field_print(DB_IMU_QUATERNION,1);
      // quaternion_print(q_sensors[0],"up_arm");
      // quaternion_print(q_sensors[1],"forearm");

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