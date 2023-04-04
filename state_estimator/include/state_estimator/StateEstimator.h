/*
* Software License Agreement (BSD License)
* Copyright (c) 2013, Georgia Institute of Technology
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**********************************************
 * @file StateEstimator.cpp
 * @author Paul Drews <pdrews3@gatech.edu>
 * @author Edited by Hojin Lee <hojinlee@unist.ac.kr> 
 * @date May 1, 2017 / modified 2022
 * @copyright 2017 Georgia Institute of Technology
 * @brief ROS node to fuse information sources and create an accurate state estimation *
 * @details Subscribes to other pose estimate solution, GPS, IMU, and wheel odometry topics, claculates
 * an estimate of the car's current state using GTSAM, and publishes that data.
 ***********************************************/

#ifndef StateEstimator_H_
#define StateEstimator_H_

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/base/timing.h>
#include <GeographicLib/LocalCartesian.hpp>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/navigation/GPSFactor.h>

#include <list>
#include <iostream>
#include <fstream>
#include <queue>




#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "geometry_msgs/msg/transform_stamped.hpp"


#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <cmath>

#include "BlockingQueue.h"

#define PI 3.14159265358979323846264338


using namespace gtsam;

  class StateEstimator
  {
  private:
    std::shared_ptr<rclcpp::Node> nh_;
  
    // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr estPosePub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;    




    rclcpp::TimerBase::SharedPtr main_timer_, tf_timer_;
    rclcpp::CallbackGroup::SharedPtr timer_main_group_, tf_group_;
     
    rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    
    // BlockingQueue<sensor_msgs::NavSatFixConstPtr> gpsOptQ_;
    BlockingQueue<sensor_msgs::msg::Imu::ConstPtr> imuOptQ_;
    // BlockingQueue<nav_msgs::msg::Odometry::ConstPtr> odomOptQ_;
    BlockingQueue<geometry_msgs::msg::TransformStamped::ConstPtr> localposeOptQ_, dual_localposeOptQ_;
    
    // BlockingQueue<geometry_msgs::PoseStampedConstPtr> localPoseOptQ_;

    boost::mutex optimizedStateMutex_;
    gtsam::NavState optimizedState_;
    double optimizedTime_;
    std::shared_ptr<gtsam::PreintegratedImuMeasurements> imuPredictor_;
    
    gtsam::imuBias::ConstantBias optimizedBias_, previousBias_;
    sensor_msgs::msg::Imu::ConstPtr lastIMU_;
    std::shared_ptr<gtsam::PreintegrationParams> preintegrationParams_;

    std::list<sensor_msgs::msg::Imu::ConstPtr> imuMeasurements_, imuGrav_;
    
    // geometry_msgs::PoseStampedConstPtr ip;
    // imu_3dm_gx4::FilterOutput initialPose_;
    gtsam::Pose3 bodyPSensor_, carENUPcarNED_;
    gtsam::Pose3 imuPgps_;


    gtsam::SharedDiagonal priorNoisePose_;
    gtsam::SharedDiagonal priorNoiseVel_;
    gtsam::SharedDiagonal priorNoiseBias_;
    gtsam::Vector noiseModelBetweenBias_sigma_;
    gtsam::ISAM2 *isam_;

    nav_msgs::msg::Odometry::ConstPtr lastOdom_;
    geometry_msgs::msg::TransformStamped::ConstPtr lastLocalPose_;



      
  
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;



    
    
    double startTime;
    int odomKey, imuKey, latestGPSKey;
    
    imuBias::ConstantBias prevBias;
    Vector3 prevVel;
    Pose3 prevPose;


    // rclcpp::CallbackGroup::SharedPtr vehicle_status_callback_group_;
    

  
      double lastImuT_, lastImuTgps_;
    unsigned char status_;

    double localPoseSigma_;
    int maxQSize_;

        bool fixedOrigin_;
    // GeographicLib::LocalCartesian enu_;   /// Object to put lat/lon coordinates into local cartesian
    bool gotFirstFix_, gotFirstLocalPose_, gotFirstImu_;

    std::string toFrame, fromFrame;
    
    bool usingLocalPose_;
    double maxGPSError_, maxLocalPoseError_;


double initialRotationNoise, initialVelNoise, initialBiasNoiseAcc, initialBiasNoiseGyro, accSigma, gyroSigma, accelBiasSigma_, gyroBiasSigma_, 
gpsSigma_, sensorX, sensorY, sensorZ, sensorXAngle, sensorYAngle, sensorZAngle, carXAngle, carYAngle, carZAngle, gravityMagnitude, imuDt_;
bool invertx_, inverty_, invertz_;

  public:
    StateEstimator(const std::shared_ptr<rclcpp::Node>& node);
    // StateEstimator();
    ~StateEstimator();
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    // void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);    
    // void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void mainloop();
    void tfloop();
    void GetAccGyro(sensor_msgs::msg::Imu::ConstPtr imu, gtsam::Vector3 &acc, gtsam::Vector3 &gyro);
    
  };


#endif /* StateEstimator_H_ */
