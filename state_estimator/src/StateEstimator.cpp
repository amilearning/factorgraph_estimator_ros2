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

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread/thread.hpp>
#include <vector>
#include "StateEstimator.h"

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>


 
#include "rclcpp/rclcpp.hpp"





using namespace gtsam;
// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::V;
using symbol_shorthand::B;
using symbol_shorthand::G; // GPS pose


// macro for getting the time stamp of a ros message
#define TIME(msg) ( (msg)->header.stamp.sec + (msg)->header.stamp.nanosec*1e-9 )


  StateEstimator::StateEstimator(const std::shared_ptr<rclcpp::Node>& node_) : nh_(node_),
    lastImuT_(0.0),
    lastImuTgps_(0.0),
    maxQSize_(0),    
    imuOptQ_(400),
    localposeOptQ_(100),
    dual_localposeOptQ_(100),
    gotFirstFix_(false),
    gotFirstLocalPose_(false),
    gotFirstImu_(false),
    odomKey(1),
    imuKey(1),
    latestGPSKey(0)
  {
    prevVel = (Vector(3) << 0.0,0.0,0.0).finished();
    // temporary variables to retrieve parameters
    nh_->declare_parameter<std::string>("toFrame", "odom");
    nh_->declare_parameter<std::string>("fromFrame", "map");
    nh_->get_parameter("toFrame",toFrame);    
    nh_->get_parameter("fromFrame",fromFrame );
    

    nh_->declare_parameter<double>("InitialRotationNoise", 1.0);
    nh_->declare_parameter<double>("InitialVelocityNoise", 0.1);
    nh_->declare_parameter<double>("InitialBiasNoiseAcc", 1e-1);
    nh_->declare_parameter<double>("InitialBiasNoiseGyro", 1e-2);
    nh_->declare_parameter<double>("AccelerometerSigma", 6.0e-2);
    nh_->declare_parameter<double>("GyroSigma", 2.0e-2);
    nh_->declare_parameter<double>("AccelBiasSigma", 2.0e-4);
    nh_->declare_parameter<double>("GyroBiasSigma", 3.0e-5);
    nh_->declare_parameter<double>("GPSSigma", 0.07);
    nh_->declare_parameter<double>("localPoseSigma", 0.01);
    nh_->declare_parameter<double>("SensorTransformX", 0.0);
    nh_->declare_parameter<double>("SensorTransformY", 0.0);
    nh_->declare_parameter<double>("SensorTransformZ", 0.0);
    nh_->declare_parameter<double>("SensorXAngle", 0);
    nh_->declare_parameter<double>("SensorYAngle", 0);
    nh_->declare_parameter<double>("SensorZAngle", 0);
    nh_->declare_parameter<double>("CarXAngle", 0);
    nh_->declare_parameter<double>("CarYAngle", 0);
    nh_->declare_parameter<double>("CarZAngle", 0);
    nh_->declare_parameter<double>("Gravity", 9.81);
    nh_->declare_parameter<bool>("InvertX", false);
    nh_->declare_parameter<bool>("InvertY", false);
    nh_->declare_parameter<bool>("InvertZ",  false);
    nh_->declare_parameter<double>("Imudt", 1.0/200.0);
   

    nh_->get_parameter("InitialRotationNoise",initialRotationNoise);    
    nh_->get_parameter("InitialVelocityNoise",initialVelNoise );
    nh_->get_parameter("InitialBiasNoiseAcc",initialBiasNoiseAcc );
    nh_->get_parameter("InitialBiasNoiseGyro",initialBiasNoiseGyro );
    nh_->get_parameter("AccelerometerSigma",accSigma );
    nh_->get_parameter("GyroSigma",gyroSigma );
    nh_->get_parameter("AccelBiasSigma",accelBiasSigma_ );
    nh_->get_parameter("GyroBiasSigma",gyroBiasSigma_ );
    nh_->get_parameter("GPSSigma",gpsSigma_ );
    nh_->get_parameter("SensorTransformX",sensorX );
    nh_->get_parameter("SensorTransformY",sensorY );
    nh_->get_parameter("SensorTransformZ",sensorZ );
    nh_->get_parameter("SensorXAngle",sensorXAngle );
    nh_->get_parameter("SensorYAngle",sensorYAngle );
    nh_->get_parameter("SensorZAngle",sensorZAngle); 
    nh_->get_parameter("CarXAngle",carXAngle );
    nh_->get_parameter("CarYAngle",carYAngle );
    nh_->get_parameter("CarZAngle",carZAngle );
    nh_->get_parameter("Gravity",gravityMagnitude );
    nh_->get_parameter("InvertX",invertx_); 
    nh_->get_parameter("InvertY",inverty_); 
    nh_->get_parameter("InvertZ",invertz_); 
    nh_->get_parameter("Imudt",imuDt_ );


  double imuPgps_x, imuPgps_y,imuPgps_z; 
    
    nh_->declare_parameter<double>("GPSX",  0);
    nh_->declare_parameter<double>("GPSY",  0);
    nh_->declare_parameter<double>("GPSZ",  0);
    nh_->get_parameter("GPSZ",  imuPgps_x);
    nh_->get_parameter("GPSZ",  imuPgps_y);
    nh_->get_parameter("GPSZ",  imuPgps_z);
    
    imuPgps_ = Pose3(Rot3(), Point3(imuPgps_x, imuPgps_y,imuPgps_z));
    imuPgps_.print("IMU->GPS");

 
     bool fixedInitialPose;
    double initialRoll, intialPitch, initialYaw;

    nh_->declare_parameter<bool>("FixedInitialPose", false);
    nh_->declare_parameter<double>("initialRoll", 0);
    nh_->declare_parameter<double>("intialPitch", 0);
    nh_->declare_parameter<double>("initialYaw", 0);

    nh_->get_parameter("FixedInitialPose",  fixedInitialPose);
    nh_->get_parameter("initialRoll",  initialRoll);
    nh_->get_parameter("intialPitch",  intialPitch);
    nh_->get_parameter("initialYaw",  initialYaw);    

    double latOrigin, lonOrigin, altOrigin;
    nh_->declare_parameter<bool>("FixedOrigin", false);
    nh_->declare_parameter<double>("latOrigin", 0);
    nh_->declare_parameter<double>("lonOrigin", 0);
    nh_->declare_parameter<double>("altOrigin", 0);
    nh_->get_parameter("FixedOrigin", fixedOrigin_);
    nh_->get_parameter("latOrigin", latOrigin);
    nh_->get_parameter("lonOrigin", lonOrigin);
    nh_->get_parameter("altOrigin", altOrigin);


    
    nh_->declare_parameter<bool>("UseLocalPose",  false);
    nh_->declare_parameter<double>("MaxGPSError",  10.0);
    nh_->declare_parameter<double>("maxLocalPoseError",  10.0);


    
    nh_->get_parameter("MaxGPSError",  maxGPSError_);
    // nh_->get_parameter("maxLocalPoseError",  maxLocalPoseError);
    

    
    std::cout << "InitialRotationNoise " << initialRotationNoise << "\n"
    << "InitialVelocityNoise " << initialVelNoise << "\n"
    << "InitialBiasNoiseAcc " << initialBiasNoiseAcc << "\n"
    << "InitialBiasNoiseGyro " << initialBiasNoiseGyro << "\n"
    << "AccelerometerSigma " << accSigma << "\n"
    << "GyroSigma " << gyroSigma << "\n"
    << "AccelBiasSigma " << accelBiasSigma_ << "\n"
    << "GyroBiasSigma " << gyroBiasSigma_ << "\n"
    << "GPSSigma " << gpsSigma_ << "\n"
    << "SensorTransformX " << sensorX << "\n"
    << "SensorTransformY " << sensorY << "\n"
    << "SensorTransformZ " << sensorZ << "\n"
    << "SensorXAngle " <<  sensorXAngle << "\n"
    << "SensorYAngle " << sensorYAngle << "\n"
    << "SensorZAngle " <<   sensorZAngle << "\n"
    << "CarXAngle " <<  carXAngle << "\n"
    << "CarYAngle " <<  carYAngle << "\n"
    << "CarZAngle " <<  carZAngle << "\n"
    << "Gravity " <<   gravityMagnitude << "\n";
    
    // // Use an ENU frame
    preintegrationParams_ =  PreintegrationParams::MakeSharedU(gravityMagnitude);
    preintegrationParams_->accelerometerCovariance = accSigma * I_3x3;
    preintegrationParams_->gyroscopeCovariance = gyroSigma * I_3x3;
    preintegrationParams_->integrationCovariance = 1e-5 * I_3x3;

    Vector biases((Vector(6) << 0, 0, 0, 0, 0, 0).finished());
    optimizedBias_ = imuBias::ConstantBias(biases);
    previousBias_ = imuBias::ConstantBias(biases);
    imuPredictor_ = std::make_shared<PreintegratedImuMeasurements>(preintegrationParams_, optimizedBias_);

    optimizedTime_ = 0;

    odom_pub_ = nh_->create_publisher<nav_msgs::msg::Odometry>("/est_odom", 10);

    tf_buffer_ =  std::make_unique<tf2_ros::Buffer>(nh_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);    // 20Hz
    tf_timer_ = nh_->create_wall_timer( std::chrono::milliseconds(50), std::bind(&StateEstimator::tfloop, this),tf_group_); 
    
    imu_callback_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto imu_optins =  rclcpp::SubscriptionOptions(); 
    imu_optins.callback_group = imu_callback_group_;
    imu_sub_ = nh_->create_subscription<sensor_msgs::msg::Imu>("/imu", 10, std::bind(&StateEstimator::imuCallback, this, std::placeholders::_1), imu_optins);
    
    timer_main_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    main_timer_ = nh_->create_wall_timer( std::chrono::milliseconds(100), std::bind(&StateEstimator::mainloop, this),timer_main_group_); 
    
  }

  StateEstimator::~StateEstimator()
  {}

  void StateEstimator::tfloop(){
     geometry_msgs::msg::TransformStamped t;
        try {
          t = tf_buffer_->lookupTransform(
            toFrame, fromFrame,
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(nh_->get_logger(), "Could not transform %s to %s: %s",
            toFrame.c_str(), fromFrame.c_str(), ex.what());
          return;
        }
        std::shared_ptr<const geometry_msgs::msg::TransformStamped> t_shared_ptr = std::make_shared<const geometry_msgs::msg::TransformStamped>(t);

        if (!localposeOptQ_.pushNonBlocking(t_shared_ptr)){
          RCLCPP_INFO(nh_->get_logger(), "Dropping a LocalPose measurement due to full queue!!");
          }

        if (!dual_localposeOptQ_.pushNonBlocking(t_shared_ptr)){
          RCLCPP_INFO(nh_->get_logger(), "Dropping a LocalPose measurement due to full queue!!");
        }
          

        if(!gotFirstLocalPose_){       

          Rot3 initRot(Quaternion(t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z));

          bodyPSensor_ = Pose3(Rot3::RzRyRx(sensorXAngle, sensorYAngle, sensorZAngle),
              Point3(sensorX, sensorY, sensorZ));
          carENUPcarNED_ = Pose3(Rot3::RzRyRx(carXAngle, carYAngle, carZAngle), Point3());

          bodyPSensor_.print("Body pose\n");
          carENUPcarNED_.print("CarBodyPose\n");

          ISAM2Params params;
          params.factorization = ISAM2Params::QR;
          isam_ = new ISAM2(params);

          // prior on the first pose
          priorNoisePose_ = noiseModel::Diagonal::Sigmas(
              (Vector(6) << initialRotationNoise, initialRotationNoise, 3*initialRotationNoise,
                  gpsSigma_, gpsSigma_, gpsSigma_).finished());

          // Add velocity prior
          priorNoiseVel_ = noiseModel::Diagonal::Sigmas(
              (Vector(3) << initialVelNoise, initialVelNoise, initialVelNoise).finished());

          // Add bias prior
          priorNoiseBias_ = noiseModel::Diagonal::Sigmas(
              (Vector(6) << initialBiasNoiseAcc,
                  initialBiasNoiseAcc,
                  initialBiasNoiseAcc,
                  initialBiasNoiseGyro,
                  initialBiasNoiseGyro,
                  initialBiasNoiseGyro).finished());

          std::cout<<"checkpoint"<<std::endl;

          Vector sigma_acc_bias_c(3), sigma_gyro_bias_c(3);
          sigma_acc_bias_c << accelBiasSigma_,  accelBiasSigma_,  accelBiasSigma_;
          sigma_gyro_bias_c << gyroBiasSigma_, gyroBiasSigma_, gyroBiasSigma_;
          noiseModelBetweenBias_sigma_ = (Vector(6) << sigma_acc_bias_c, sigma_gyro_bias_c).finished();



          gotFirstLocalPose_ = true;
        }
        
  }


void StateEstimator::imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu)   {
      
        
        double dt;
        if (lastImuT_ == 0) dt = 0.005; // 200 Hz 
        else dt = TIME(imu) - lastImuT_;

        lastImuT_ = TIME(imu);
        //ros::Time before = ros::Time::now();

        // Push the IMU measurement to the optimization thread
        int qSize = imuOptQ_.size();
        if (qSize > maxQSize_)
          maxQSize_ = qSize;
        if (!imuOptQ_.pushNonBlocking(imu))
          RCLCPP_WARN(nh_->get_logger(),"Dropping an IMU measurement due to full queue!!");

        // Each time we get an imu measurement, calculate the incremental pose from the last GTSAM pose
        imuMeasurements_.push_back(imu);
        //Grab the most current optimized state
        double optimizedTime;
        NavState optimizedState;
        imuBias::ConstantBias optimizedBias;
        unsigned char status;
        {
          boost::mutex::scoped_lock guard(optimizedStateMutex_);
          optimizedState = optimizedState_;
          optimizedBias = optimizedBias_;
          optimizedTime = optimizedTime_;
          
        }
        if (optimizedTime == 0) return; // haven't optimized first state yet

        bool newMeasurements = false;
        int numImuDiscarded = 0;
        double imuQPrevTime;
        Vector3 acc, gyro;
        while (!imuMeasurements_.empty() && (TIME(imuMeasurements_.front()) < optimizedTime))
        {
          imuQPrevTime = TIME(imuMeasurements_.front());
          imuMeasurements_.pop_front();
          newMeasurements = true;
          numImuDiscarded++;
        }

        if(newMeasurements)
        {
          //We need to reset integration and iterate through all our IMU measurements
          imuPredictor_->resetIntegration();
          int numMeasurements = 0;
          for (auto it=imuMeasurements_.begin(); it!=imuMeasurements_.end(); ++it)
          {
            double dt_temp =  TIME(*it) - imuQPrevTime;
            imuQPrevTime = TIME(*it);
            GetAccGyro(*it, acc, gyro);
            imuPredictor_->integrateMeasurement(acc, gyro, dt_temp);
            numMeasurements++;
            // ROS_INFO("IMU time %f, dt %f", (*it)->header.stamp.toSec(), dt_temp);
          }
          // ROS_INFO("Resetting Integration, %d measurements integrated, %d discarded", numMeasurements, numImuDiscarded);
        }
        else
        {
          //Just need to add the newest measurement, no new optimized pose
          GetAccGyro(imu, acc, gyro);
          imuPredictor_->integrateMeasurement(acc, gyro, dt);
          // ROS_INFO("Integrating %f, dt %f", m_lastImuT, dt);
        }

        // predict next state given the imu measurements
        NavState currentPose = imuPredictor_->predict(optimizedState, optimizedBias);
        nav_msgs::msg::Odometry poseNew;
        poseNew.header.stamp = imu->header.stamp;

        Vector4 q = currentPose.quaternion().coeffs();
        poseNew.pose.pose.orientation.x = q[0];
        poseNew.pose.pose.orientation.y = q[1];
        poseNew.pose.pose.orientation.z = q[2];
        poseNew.pose.pose.orientation.w = q[3];

        poseNew.pose.pose.position.x = currentPose.position().x();
        poseNew.pose.pose.position.y = currentPose.position().y();
        poseNew.pose.pose.position.z = currentPose.position().z();

        poseNew.twist.twist.linear.x = currentPose.velocity().x();
        poseNew.twist.twist.linear.y = currentPose.velocity().y();
        poseNew.twist.twist.linear.z = currentPose.velocity().z();
        
        poseNew.twist.twist.angular.x = gyro.x() + optimizedBias.gyroscope().x();
        poseNew.twist.twist.angular.y = gyro.y() + optimizedBias.gyroscope().y();
        poseNew.twist.twist.angular.z = gyro.z() + optimizedBias.gyroscope().z();

        poseNew.child_frame_id = "base_link";
        poseNew.header.frame_id = "map";

        odom_pub_->publish(poseNew);
        
        return;
      }


  
  void StateEstimator::GetAccGyro(sensor_msgs::msg::Imu::ConstPtr imu, Vector3 &acc, Vector3 &gyro)
  {
    double accx, accy, accz;
    if (invertx_) accx = -imu->linear_acceleration.x;
    else accx = imu->linear_acceleration.x;
    if (inverty_) accy = -imu->linear_acceleration.y;
    else accy = imu->linear_acceleration.y;
    if (invertz_) accz = -imu->linear_acceleration.z;
    else accz = imu->linear_acceleration.z;
    acc = Vector3(accx, accy, accz);

    double gx, gy, gz;
    if (invertx_) gx = -imu->angular_velocity.x;
    else gx = imu->angular_velocity.x;
    if (inverty_) gy = -imu->angular_velocity.y;
    else gy = imu->angular_velocity.y;
    if (invertz_) gz = -imu->angular_velocity.z;
    else gz = imu->angular_velocity.z;

    gyro = Vector3(gx, gy, gz);
  }


  void StateEstimator::mainloop()
  { if(!gotFirstLocalPose_ || !gotFirstImu_){
      return;
    }
    
      bool optimize = false;
      
      if (!gotFirstFix_)
      {
        geometry_msgs::msg::TransformStamped::ConstPtr fix = localposeOptQ_.popBlocking();
        startTime = TIME(fix);
        if(imuOptQ_.size() <= 0) {          
          RCLCPP_WARN(nh_->get_logger(),"no IMU messages before first fix, continuing until there is");
          return;
        }
        // errors out if the IMU and GPS are not close in timestamps
        double most_recent_imu_time = imuOptQ_.back()->header.stamp.sec+imuOptQ_.back()->header.stamp.nanosec*1e-9;
        if(std::abs(most_recent_imu_time - startTime) > 0.1) {
          RCLCPP_WARN(nh_->get_logger(),"There is a very large difference in the LocalPose and IMU timestamps ");          
          exit(-1);
        }

        lastLocalPose_ = dual_localposeOptQ_.popBlocking();

        NonlinearFactorGraph newFactors;
        Values newVariables;
        gotFirstFix_ = true;

        double E, N, U;
        E = fix->transform.translation.x;
        N = fix->transform.translation.y;
        U = fix->transform.translation.z;
        
        // Add prior factors on pose, vel and bias
        Rot3 initialOrientation = Rot3::Quaternion(fix->transform.rotation.w, fix->transform.rotation.x, fix->transform.rotation.y, fix->transform.rotation.z);
          
        std::cout << "Initial orientation" << std::endl;
        std::cout << bodyPSensor_.rotation() * initialOrientation * carENUPcarNED_.rotation() << std::endl;
        Pose3 x0(bodyPSensor_.rotation() * initialOrientation * carENUPcarNED_.rotation(),
            Point3(E, N, U));
        prevPose = x0;
        PriorFactor<Pose3> priorPose(X(0), x0, priorNoisePose_);
        newFactors.add(priorPose);
        PriorFactor<Vector3> priorVel(V(0), Vector3(0, 0, 0), priorNoiseVel_);
        newFactors.add(priorVel);
        Vector biases((Vector(6) << 0, 0, 0, 0.0, 0.0, 0.0).finished());
        prevBias = imuBias::ConstantBias(biases);
        PriorFactor<imuBias::ConstantBias> priorBias(B(0), imuBias::ConstantBias(biases), priorNoiseBias_);
        newFactors.add(priorBias);

        //Factor for imu->gps translation
        BetweenFactor<Pose3> imuPgpsFactor(X(0), G(0), imuPgps_,
            noiseModel::Diagonal::Sigmas((Vector(6) << 0.001,0.001,0.001,0.03,0.03,0.03).finished()));
        newFactors.add(imuPgpsFactor);

        // add prior values on pose, vel and bias
        newVariables.insert(X(0), x0);
        newVariables.insert(V(0), Vector3(0, 0, 0));
        newVariables.insert(B(0), imuBias::ConstantBias(biases));
        newVariables.insert(G(0), x0.compose(imuPgps_));

        isam_->update(newFactors, newVariables);
        //Read IMU measurements up to the first GPS measurement
        lastIMU_ = imuOptQ_.popBlocking();
        //If we only pop one, we need some dt
        lastImuTgps_ = TIME(lastIMU_) - 0.005;
        while(TIME(lastIMU_) < TIME(fix))
        {
          lastImuTgps_ = TIME(lastIMU_);
          lastIMU_ = imuOptQ_.popBlocking();
        }
      }
      else
      {
        NonlinearFactorGraph newFactors;
        Values newVariables;

        // add IMU measurements
        while (imuOptQ_.size() > 0 && (TIME(imuOptQ_.back()) > (startTime + imuKey * 0.1)))
        {          
          double curTime = startTime + imuKey * 0.1;
          PreintegratedImuMeasurements pre_int_data(preintegrationParams_, previousBias_);
          while(TIME(lastIMU_) < curTime)
          {
            Vector3 acc, gyro;
            GetAccGyro(lastIMU_, acc, gyro);
            double imuDT = TIME(lastIMU_) - lastImuTgps_;
            lastImuTgps_ = TIME(lastIMU_);
            pre_int_data.integrateMeasurement(acc, gyro, imuDT);
            lastIMU_ = imuOptQ_.popBlocking();
          }
          // adding the integrated IMU measurements to the factor graph
          ImuFactor imuFactor(X(imuKey-1), V(imuKey-1), X(imuKey), V(imuKey), B(imuKey-1), pre_int_data);
          newFactors.add(imuFactor);
          newFactors.add(BetweenFactor<imuBias::ConstantBias>(B(imuKey-1), B(imuKey), imuBias::ConstantBias(),
              noiseModel::Diagonal::Sigmas( sqrt(pre_int_data.deltaTij()) * noiseModelBetweenBias_sigma_)));

          // Predict forward to get an initial estimate for the pose and velocity
          NavState curNavState(prevPose, prevVel);
          NavState nextNavState = pre_int_data.predict(curNavState, prevBias);
          newVariables.insert(X(imuKey), nextNavState.pose());
          newVariables.insert(V(imuKey), nextNavState.v());
          newVariables.insert(B(imuKey), previousBias_);
          newVariables.insert(G(imuKey), nextNavState.pose().compose(imuPgps_));
          prevPose = nextNavState.pose();
          prevVel = nextNavState.v();
          ++imuKey;
          optimize = true;
        }
        

        // add GPS measurements that are not ahead of the imu messages
        while (optimize && localposeOptQ_.size() > 0 && TIME(localposeOptQ_.front()) < (startTime + (imuKey-1)*0.1 + 1e-2))
        {
          geometry_msgs::msg::TransformStamped::ConstPtr fix = localposeOptQ_.popBlocking();
          
          double timeDiff = (TIME(fix) - startTime) / 0.1; // 
          int key = round(timeDiff); 
          
          RCLCPP_WARN(nh_->get_logger(),"timeDiff = %f", timeDiff);
          RCLCPP_WARN(nh_->get_logger(),"key = %d", key);
          
          if (std::abs(timeDiff - key) < 1e-1)
          { 
            
            // this is a gps message for a factor
            latestGPSKey = key;
            double E,N,U;
            E = fix->transform.translation.x;
            N = fix->transform.translation.y;
            U = fix->transform.translation.z;
            // check if the GPS message is close to our expected position
            Pose3 expectedState;
            
            if (newVariables.exists(X(key)))
              expectedState = (Pose3) newVariables.at<Pose3>(X(key));
            else
              expectedState = isam_->calculateEstimate<Pose3>(X(key));

            // ROS_WARN("x = %f,y = %f,z = %f,",expectedState.x(),expectedState.y(),expectedState.z());
            // ROS_WARN("E = %f,N = %f",E,N);
            double dist = std::sqrt( std::pow(expectedState.x() - E, 2) + std::pow(expectedState.y() - N, 2) );
            
            // ROS_WARN("dist = %f", dist);
            // if (dist > maxGPSError_ )
            // {ROS_WARN("dist > maxGPSError");}
            // if (latestGPSKey > imuKey-2)
            // {ROS_WARN("latestGPSKey > imuKey-2");}

            if (dist < maxGPSError_ || latestGPSKey < imuKey-2)
            {

              SharedDiagonal gpsNoise = noiseModel::Diagonal::Sigmas(Vector3(gpsSigma_, gpsSigma_, gpsSigma_));
              GPSFactor gpsFactor(G(key), Point3(E, N, U), gpsNoise);
              newFactors.add(gpsFactor);
              BetweenFactor<Pose3> imuPgpsFactor(X(key), G(key), imuPgps_,
                  noiseModel::Diagonal::Sigmas((Vector(6) << 0.001,0.001,0.001,0.03,0.03,0.03).finished()));
              newFactors.add(imuPgpsFactor);

            }
            else
            {
              RCLCPP_WARN(nh_->get_logger(),"Receied Bad Local Pose");
            }
          }
        }

//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
        // add LocalPose measurements that are not ahead of the imu messages         
        while (optimize && dual_localposeOptQ_.size() > 0 && TIME(dual_localposeOptQ_.front()) < (startTime + (imuKey-1)*0.1 + 1e-2))
        {
          geometry_msgs::msg::TransformStamped::ConstPtr local_pose_ = dual_localposeOptQ_.popBlocking();
          double timeDiff_ = (TIME(local_pose_) - startTime) / 0.1; // 
          int key_ = round(timeDiff_);                       
          if (std::abs(timeDiff_ - key_) < 1e-1)
          { 
            gtsam::Pose3 local_pose3_ = Pose3(Rot3::Quaternion(local_pose_->transform.rotation.w, local_pose_->transform.rotation.x, local_pose_->transform.rotation.y, local_pose_->transform.rotation.z),Point3(local_pose_->transform.translation.x, local_pose_->transform.translation.y, local_pose_->transform.translation.z));

            // check if the LocalPoseMsg message is close to our expected position
            Pose3 expectedState_;            
            if (newVariables.exists(X(key_)))
              expectedState_ = (Pose3) newVariables.at<Pose3>(X(key_));
            else
              expectedState_ = isam_->calculateEstimate<Pose3>(X(key_));            
            double dist_ = std::sqrt( std::pow(expectedState_.x() - local_pose_->transform.translation.x, 2) + std::pow(expectedState_.y() - local_pose_->transform.translation.y, 2) );
        
            if (dist_ < maxLocalPoseError_ || key_ < imuKey-2)
            {
              SharedDiagonal LocalPoseNoise = noiseModel::Diagonal::Sigmas(
                      (Vector(6) << localPoseSigma_*0.1,localPoseSigma_*0.1,localPoseSigma_*0.1,localPoseSigma_,localPoseSigma_,localPoseSigma_).finished());

              PriorFactor<Pose3> localPosePrior_(X(key_), local_pose3_, LocalPoseNoise);
              newFactors.add(localPosePrior_);
              // newFactors.emplace_shared<PriorFactor<Pose3>>(X(key_), local_pose3_, LocalPoseNoise);
              // newVariables.insert(X(key_), local_pose3_);              
            }
            else
            {
               RCLCPP_WARN(nh_->get_logger(),"Receied Bad Local Pose");
            }
          }
        }

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
        
      


        // if we processed imu - then we can optimize the state
        if (optimize)
        {
          try
          {
            isam_->update(newFactors, newVariables);
            Pose3 nextState = isam_->calculateEstimate<Pose3>(X(imuKey-1));

            prevPose = nextState;
            prevVel = isam_->calculateEstimate<Vector3>(V(imuKey-1));
            prevBias = isam_->calculateEstimate<imuBias::ConstantBias>(B(imuKey-1));

            // if we haven't added gps data for 2 message (0.2s) then change status
            if (latestGPSKey + 3 < imuKey)
            {
              RCLCPP_WARN(nh_->get_logger(),"WARN");
              
            }
            

            double curTime = startTime + (imuKey-1) * 0.1;

            {
              boost::mutex::scoped_lock guard(optimizedStateMutex_);
              optimizedState_ = NavState(prevPose, prevVel);
              optimizedBias_ = prevBias;
              optimizedTime_ = curTime;
              
            }

         
          }
          catch(gtsam::IndeterminantLinearSystemException ex)
          {
              RCLCPP_WARN(nh_->get_logger(),"Encountered Indeterminant System Error!");
            
          }
        }
       
    }
}


  




int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // create the ROS 2 node
    auto node = std::make_shared<rclcpp::Node>("est_node");

    // create the controller object
    StateEstimator wpt(node);

    // run the node
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}