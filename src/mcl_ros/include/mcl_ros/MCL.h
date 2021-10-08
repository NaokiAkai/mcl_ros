/****************************************************************************
 * mcl_ros: Monte Carlo localization with ROS
 * Copyright (C) 2021 Naoki Akai
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * @author Naoki Akai
 ****************************************************************************/

#ifndef __MCL_H__
#define __MCL_H__

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <mcl_ros/Pose.h>
#include <mcl_ros/Particle.h>

namespace mcl_ros {

class MCL {
private:
    ros::NodeHandle nh_;

    // subscribers
    std::string scanName_, odomName_, mapName_;
    ros::Subscriber scanSub_, odomSub_, mapSub_, initialPoseSub_;

    // publishers
    std::string poseName_, particlesName_, unknownScanName_, residualErrorsName_;
    ros::Publisher posePub_, particlesPub_, unknownScanPub_, residualErrorsPub_;

    // frames
    std::string laserFrame_, baseLinkFrame_, mapFrame_, odomFrame_;
    bool useOdomFrame_;

    std::vector<double> initialPose_;
    Pose mclPose_, baseLink2Laser_, odomPose_;
    ros::Time mclPoseStamp_, odomPoseStamp_;

    // particles
    int particlesNum_;
    std::vector<Particle> particles_;
    std::vector<double> initialNoise_;
    bool useAugmentedMCL_, addRandomParticlesInResampling_;
    double randomParticlesRate_;
    std::vector<double> randomParticlesNoise_;

    // map
    cv::Mat distMap_;
    double mapResolution_;
    Pose mapOrigin_;
    int mapWidth_, mapHeight_;
    bool gotMap_;

    // motion
    double deltaX_, deltaY_, deltaDist_, deltaYaw_;
    double deltaXSum_, deltaYSum_, deltaDistSum_, deltaYawSum_, deltaTimeSum_;
    std::vector<double> resampleThresholds_;
    std::vector<double> odomNoiseDDM_, odomNoiseODM_;
    bool useOmniDirectionalModel_;

    // measurements
    sensor_msgs::LaserScan scan_, unknownScan_;
    bool canUpdateScan_;

    // measurement model
    // 0: likelihood field model, 1: beam model, 2: class conditional measurement model
    int measurementModelType_;
    double zHit_, zShort_, zMax_, zRand_;
    double varHit_, lambdaShort_, lambdaUnknown_;
    double normConstHit_, denomHit_, pRand_;
    double measurementModelRandom_, measurementModelInvalidScan_;
    double pKnownPrior_, pUnknownPrior_, unknownScanProbThreshold_;
    double alphaSlow_, alphaFast_, omegaSlow_, omegaFast_;
    int scanStep_;
    bool rejectUnknownScan_, publishUnknownScan_, publishResidualErrors_;
    bool gotScan_;
    double resampleThresholdESS_;

    // localization result
    double totalLikelihood_, averageLikelihood_, maxLikelihood_;
    double amclRandomParticlesRate_, effectiveSampleSize_;
    int maxLikelihoodParticleIdx_;

    // other parameters
    tf::TransformBroadcaster tfBroadcaster_;
    tf::TransformListener tfListener_;
    bool isInitialized_;
    double localizationHz_;

    // constant parameters
    const double rad2deg_;

public:
    // inline setting functions
    inline void setCanUpdateScan(bool canUpdateScan) { canUpdateScan_ = canUpdateScan; }

    // inline getting functions
    inline double getLocalizationHz(void) { return localizationHz_; }

    MCL(void):
        nh_("~"),
        scanName_("/scan"),
        odomName_("/odom"),
        mapName_("/map"),
        poseName_("/mcl_pose"),
        particlesName_("/mcl_particles"),
        unknownScanName_("/unknown_scan"),
        residualErrorsName_("/residual_errors"),
        laserFrame_("/laser"),
        baseLinkFrame_("/base_link"),
        mapFrame_("/map"),
        odomFrame_("/odom"),
        useOdomFrame_(true),
        initialPose_({0.0, 0.0, 0.0}),
        particlesNum_(100),
        initialNoise_({0.2, 0.2, 0.02}),
        useAugmentedMCL_(false),
        addRandomParticlesInResampling_(true),
        randomParticlesRate_(0.1),
        randomParticlesNoise_({0.1, 0.1, 0.01}),
        odomNoiseDDM_({1.0, 0.5, 0.5, 1.0}),
        odomNoiseODM_({1.0, 0.5, 0.5, 0.5, 1.0, 0.5, 0.5, 0.5, 1.0}),
        deltaXSum_(0.0),
        deltaYSum_(0.0),
        deltaDistSum_(0.0),
        deltaYawSum_(0.0),
        deltaTimeSum_(0.0),
        resampleThresholds_({-1.0, -1.0, -1.0, -1.0, -1.0}),
        useOmniDirectionalModel_(false),
        measurementModelType_(0),
        pKnownPrior_(0.5),
        pUnknownPrior_(0.5),
        scanStep_(10),
        zHit_(0.9),
        zShort_(0.2),
        zMax_(0.05),
        zRand_(0.05),
        varHit_(0.1),
        lambdaShort_(3.0),
        lambdaUnknown_(1.0),
        alphaSlow_(0.001),
        alphaFast_(0.9),
        omegaSlow_(0.0),
        omegaFast_(0.0),
        rejectUnknownScan_(false),
        publishUnknownScan_(false),
        publishResidualErrors_(false),
        resampleThresholdESS_(0.5),
        localizationHz_(10.0),
        gotMap_(false),
        gotScan_(false),
        isInitialized_(true),
        canUpdateScan_(true),
        tfListener_(),
        rad2deg_(180.0 / M_PI)
    {
        // topic and frame names
        nh_.param("scan_name", scanName_, scanName_);
        nh_.param("odom_name", odomName_, odomName_);
        nh_.param("map_name", mapName_, mapName_);
        nh_.param("pose_name", poseName_, poseName_);
        nh_.param("particles_name", particlesName_, particlesName_);
        nh_.param("unknown_scan_name", unknownScanName_, unknownScanName_);  
        nh_.param("residual_errors_name", residualErrorsName_, residualErrorsName_);      
        nh_.param("laser_frame", laserFrame_, laserFrame_);
        nh_.param("base_link_frame", baseLinkFrame_, baseLinkFrame_);
        nh_.param("map_frame", mapFrame_, mapFrame_);
        nh_.param("odom_frame", odomFrame_, odomFrame_);
        nh_.param("use_odom_frame", useOdomFrame_, useOdomFrame_);

        // particle filter parameters
        nh_.param("initial_pose", initialPose_, initialPose_);
        nh_.param("particle_num", particlesNum_, particlesNum_);
        nh_.param("initial_noise", initialNoise_, initialNoise_);
        nh_.param("use_augmented_mcl", useAugmentedMCL_, useAugmentedMCL_);
        nh_.param("add_random_particles_in_resampling", addRandomParticlesInResampling_, addRandomParticlesInResampling_);
        nh_.param("random_particles_rate", randomParticlesRate_, randomParticlesRate_);
        nh_.param("random_particles_noise", randomParticlesNoise_, randomParticlesNoise_);

        // motion
        nh_.param("odom_noise_ddm", odomNoiseDDM_, odomNoiseDDM_);
        nh_.param("odom_noise_odm", odomNoiseODM_, odomNoiseODM_);
        nh_.param("use_omni_directional_model", useOmniDirectionalModel_, useOmniDirectionalModel_);

        // measurement model
        nh_.param("measurement_model_type", measurementModelType_, measurementModelType_);
        nh_.param("scan_step", scanStep_, scanStep_);
        nh_.param("z_hit", zHit_, zHit_);
        nh_.param("z_short", zShort_, zShort_);
        nh_.param("z_max", zMax_, zMax_);
        nh_.param("z_rand", zRand_, zRand_);
        nh_.param("var_hit", varHit_, varHit_);
        nh_.param("lambda_short", lambdaShort_, lambdaShort_);
        nh_.param("lambda_unknown", lambdaUnknown_, lambdaUnknown_);
        nh_.param("known_class_prior", pKnownPrior_, pKnownPrior_);
        nh_.param("unknown_scan_prob_threshold", unknownScanProbThreshold_, unknownScanProbThreshold_);
        nh_.param("alpha_slow", alphaSlow_, alphaSlow_);
        nh_.param("alpha_fast", alphaFast_, alphaFast_);
        nh_.param("reject_unknown_scan", rejectUnknownScan_, rejectUnknownScan_);
        nh_.param("publish_unknown_scan", publishUnknownScan_, publishUnknownScan_);
        nh_.param("publish_residual_errors", publishResidualErrors_, publishResidualErrors_);
        nh_.param("resample_threshold_ess", resampleThresholdESS_, resampleThresholdESS_);
        nh_.param("resample_thresholds", resampleThresholds_, resampleThresholds_);
        pUnknownPrior_ = 1.0 - pKnownPrior_;

        // other parameters
        nh_.param("localization_hz", localizationHz_, localizationHz_);

        // set subscribers
        scanSub_ = nh_.subscribe(scanName_, 10, &MCL::scanCB, this);
        odomSub_ = nh_.subscribe(odomName_, 100, &MCL::odomCB, this);
        mapSub_ = nh_.subscribe(mapName_, 1, &MCL::mapCB, this);
        initialPoseSub_ = nh_.subscribe("/initialpose", 1, &MCL::initialPoseCB, this);

        // set publishers
        posePub_ = nh_.advertise<geometry_msgs::PoseStamped>(poseName_, 1);
        particlesPub_ = nh_.advertise<geometry_msgs::PoseArray>(particlesName_, 1);
        unknownScanPub_ = nh_.advertise<sensor_msgs::LaserScan>(unknownScanName_, 1);
        residualErrorsPub_ = nh_.advertise<sensor_msgs::LaserScan>(residualErrorsName_, 1);

        // set initial pose
        mclPose_.setPose(initialPose_[0], initialPose_[1], initialPose_[2]);
        resetParticlesDistribution();

        // get the relative pose from the base link to the laser from the tf tree
        ros::Rate loopRate(10);
        tf::StampedTransform tfBaseLink2Laser;
        int tfFailedCnt = 0;
        while (ros::ok()) {
            ros::spinOnce();
            try {
                ros::Time now = ros::Time::now();
                tfListener_.waitForTransform(baseLinkFrame_, laserFrame_, now, ros::Duration(1.0));
                tfListener_.lookupTransform(baseLinkFrame_, laserFrame_, now, tfBaseLink2Laser);
                break;
            } catch (tf::TransformException ex) {
                tfFailedCnt++;
                if (tfFailedCnt >= 30) {
                    ROS_ERROR("Cannot get the relative pose from the base link to the laser from the tf tree."
                        " Did you set the static transform publisher between %s to %s?",
                        baseLinkFrame_.c_str(), laserFrame_.c_str());
                    exit(1);
                }
                loopRate.sleep();
            }
        }
        tf::Quaternion quatBaseLink2Laser(tfBaseLink2Laser.getRotation().x(),
            tfBaseLink2Laser.getRotation().y(),
            tfBaseLink2Laser.getRotation().z(),
            tfBaseLink2Laser.getRotation().w());
        double baseLink2LaserRoll, baseLink2LaserPitch, baseLink2LaserYaw;
        tf::Matrix3x3 rotMatBaseLink2Laser(quatBaseLink2Laser);
        rotMatBaseLink2Laser.getRPY(baseLink2LaserRoll, baseLink2LaserPitch, baseLink2LaserYaw);
        baseLink2Laser_.setX(tfBaseLink2Laser.getOrigin().x());
        baseLink2Laser_.setY(tfBaseLink2Laser.getOrigin().y());
        baseLink2Laser_.setYaw(baseLink2LaserYaw);

        int mapFailedCnt = 0;
        while (ros::ok()) {
            ros::spinOnce();
            if (gotMap_)
                break;
            mapFailedCnt++;
            if (mapFailedCnt >= 100) {
                ROS_ERROR("Cannot get a map message."
                    " Did you pulish the map?"
                    " Expected map topic name is %s\n", mapName_.c_str());
                exit(1);
            }
            loopRate.sleep();
        }

        int scanFailedCnt = 0;
        while (ros::ok()) {
            ros::spinOnce();
            if (gotScan_)
                break;
            scanFailedCnt++;
            if (scanFailedCnt >= 100) {
                ROS_ERROR("Cannot get a scan message."
                    " Did you pulish the scan?"
                    " Expected scan topic name is %s\n", scanName_.c_str());
                exit(1);
            }
            loopRate.sleep();
        }

        // measurement model
        normConstHit_ = 1.0 / sqrt(2.0 * varHit_ * M_PI);
        denomHit_ = 1.0 / (2.0 * varHit_);
        pRand_ = 1.0 / (scan_.range_max / mapResolution_);
        measurementModelRandom_ = zRand_ * pRand_;
        measurementModelInvalidScan_ = zMax_ + zRand_ * pRand_;

        isInitialized_ = true;

        ROS_INFO("MCL ready to localize\n");
    }

    void updataParticlesByMotionModel(void) {
        double deltaX = deltaX_;
        double deltaY = deltaY_;
        double deltaDist = deltaDist_;
        double deltaYaw = deltaYaw_;
        deltaX_ = deltaY_ = deltaDist_ = deltaYaw_ = 0.0;
        deltaXSum_ += fabs(deltaX);
        deltaYSum_ += fabs(deltaY);
        deltaDistSum_ += fabs(deltaDist);
        deltaYawSum_ += fabs(deltaYaw);

        if (!useOmniDirectionalModel_) {
            // differential drive model
            double dist2 = deltaDist * deltaDist;
            double yaw2 = deltaYaw * deltaYaw;
            double distRandVal = dist2 * odomNoiseDDM_[0] + yaw2 * odomNoiseDDM_[1];
            double yawRandVal = dist2 * odomNoiseDDM_[2] + yaw2 * odomNoiseDDM_[3];
            for (int i = 0; i < particlesNum_; ++i) {
                double ddist = deltaDist + nrand(distRandVal);
                double dyaw = deltaYaw + nrand(yawRandVal);
                double yaw = particles_[i].getYaw();
                double t = yaw + dyaw / 2.0;
                double x = particles_[i].getX() + ddist * cos(t);
                double y = particles_[i].getY() + ddist * sin(t);
                yaw += dyaw;
                particles_[i].setPose(x, y, yaw);
            }
        } else {
            // omni directional model
            double x2 = deltaX * deltaX;
            double y2 = deltaY * deltaY;
            double yaw2 = deltaYaw * deltaYaw;
            double xRandVal = x2 * odomNoiseODM_[0] + y2 * odomNoiseODM_[1] + yaw2 * odomNoiseODM_[2];
            double yRandVal = x2 * odomNoiseODM_[3] + y2 * odomNoiseODM_[4] + yaw2 * odomNoiseODM_[5];
            double yawRandVal = x2 * odomNoiseODM_[6] + y2 * odomNoiseODM_[7] + yaw2 * odomNoiseODM_[8];
            for (int i = 0; i < particlesNum_; ++i) {
                double dx = deltaX + nrand(xRandVal);
                double dy = deltaY + nrand(yRandVal);
                double dyaw = deltaYaw + nrand(yawRandVal);
                double yaw = particles_[i].getYaw();
                double t = yaw + dyaw / 2.0;
                double x = particles_[i].getX() + dx * cos(t) + dy * cos(t + M_PI / 2.0f);
                double y = particles_[i].getY() + dx * sin(t) + dy * sin(t + M_PI / 2.0f);;
                yaw += dyaw;
                particles_[i].setPose(x, y, yaw);
            }
        }
    }

    void calculateLikelihoodsByMeasurementModel(void) {
        if (rejectUnknownScan_ && (measurementModelType_ == 0 || measurementModelType_ == 1))
            rejectUnknownScan();

        mclPoseStamp_ = scan_.header.stamp;
        double xo = baseLink2Laser_.getX();
        double yo = baseLink2Laser_.getY();
        double yawo = baseLink2Laser_.getYaw();
        std::vector<Pose> sensorPoses(particlesNum_);
        for (int i = 0; i < particlesNum_; ++i) {
            double yaw = particles_[i].getYaw();
            double sensorX = xo * cos(yaw) - yo * sin(yaw) + particles_[i].getX();
            double sensorY = xo * sin(yaw) + yo * cos(yaw) + particles_[i].getY();
            double sensorYaw = yawo + yaw;
            Pose sensorPose(sensorX, sensorY, sensorYaw);
            sensorPoses[i] = sensorPose;
            particles_[i].setW(0.0);
        }

        for (int i = 0; i < (int)scan_.ranges.size(); i += scanStep_) {
            double range = scan_.ranges[i];
            double rangeAngle = (double)i * scan_.angle_increment + scan_.angle_min;
            double max;
            for (int j = 0; j < particlesNum_; ++j) {
                double p;
                if (measurementModelType_ == 0)
                    p = calculateLikelihoodFieldModel(sensorPoses[j], range, rangeAngle);
                else if (measurementModelType_ == 1)
                    p = calculateBeamModel(sensorPoses[j], range, rangeAngle);
                else
                    p = calculateClassConditionalMeasurementModel(sensorPoses[j], range, rangeAngle);
                double w = particles_[j].getW();
                w += log(p);
                particles_[j].setW(w);
                if (j == 0) {
                    max = w;
                } else {
                    if (max < w)
                        max = w;
                }
            }

            // Too small values cannot be calculated.
            // The log sum values are shifted if the maximum value is less than threshold.
            if (max < -400.0) {
                for (int j = 0; j < particlesNum_; ++j) {
                    double w = particles_[j].getW() + 400.0;
                    particles_[j].setW(w);
                }
            }
        }

        double sum = 0.0;
        double max;
        int maxIdx;
        for (int i = 0; i < particlesNum_; ++i) {
            // The log sum is converted to the probability.
            double w = exp(particles_[i].getW());
            particles_[i].setW(w);
            sum += w;
            if (i == 0) {
                max = w;
                maxIdx = i;
            } else if (max < w) {
                max = w;
                maxIdx = i;
            }
        }
        totalLikelihood_ = sum;
        averageLikelihood_ = sum / (double)particlesNum_;
        maxLikelihood_ = max;
        maxLikelihoodParticleIdx_ = maxIdx;

        // augmented MCL
        omegaSlow_ += alphaSlow_ * (averageLikelihood_ - omegaSlow_);
        omegaFast_ += alphaFast_ * (averageLikelihood_ - omegaFast_);
        amclRandomParticlesRate_ = 1.0 - omegaFast_ / omegaSlow_;
        if (amclRandomParticlesRate_ < 0.0)
            amclRandomParticlesRate_ = 0.0;

        // If publishUnknownScan_ is true and the class conditional measurement model is used,
        // unknown scan measurements are estimated based on the maximum likelihood particle.
        if (publishUnknownScan_ && measurementModelType_ == 2) {
            Pose mlPose = particles_[maxLikelihoodParticleIdx_].getPose();
            estimateUnknownScanWithClassConditionalMeasurementModel(mlPose);
        }

        // calculate the effective sample size
        double sum2 = 0.0;
        double wo = 1.0 / (double)particlesNum_;
        for (int i = 0; i < particlesNum_; ++i) {
            double w = particles_[i].getW() / sum;
            if (std::isnan(w))
                w = wo;
            particles_[i].setW(w);
            sum2 += w * w;
        }
        effectiveSampleSize_ = 1.0 / sum2;
    }

    void estimatePose(void) {
        double tmpYaw = mclPose_.getYaw();
        double x = 0.0, y = 0.0, yaw = 0.0;
        for (size_t i = 0; i < particlesNum_; ++i) {
            double w = particles_[i].getW();
            x += particles_[i].getX() * w;
            y += particles_[i].getY() * w;
            double dyaw = tmpYaw - particles_[i].getYaw();
            while (dyaw < -M_PI)
                dyaw += 2.0 * M_PI;
            while (dyaw > M_PI)
                dyaw -= 2.0 * M_PI;
            yaw += dyaw * w;
        }
        yaw = tmpYaw - yaw;
        mclPose_.setPose(x, y, yaw);
    }

    void resampleParticles(void) {
        double threshold = (double)particlesNum_ * resampleThresholdESS_;
        if (effectiveSampleSize_ > threshold)
            return;

        if (deltaXSum_ < resampleThresholds_[0] && deltaYSum_ < resampleThresholds_[1] &&
            deltaDistSum_ < resampleThresholds_[2] && deltaYawSum_ < resampleThresholds_[3] &&
            deltaTimeSum_ < resampleThresholds_[4])
            return;

        deltaXSum_ = deltaYSum_ = deltaDistSum_ = deltaYSum_ = deltaTimeSum_ = 0.0;
        std::vector<double> wBuffer(particlesNum_);
        wBuffer[0] = particles_[0].getW();
        for (int i = 1; i < particlesNum_; ++i)
            wBuffer[i] = particles_[i].getW() + wBuffer[i - 1];

        std::vector<Particle> tmpParticles = particles_;
        double wo = 1.0 / (double)particlesNum_;

        if (!addRandomParticlesInResampling_ && !useAugmentedMCL_) {
            // normal resampling
            for (int i = 0; i < particlesNum_; ++i) {
                double darts = (double)rand() / ((double)RAND_MAX + 1.0);
                for (int j = 0; j < particlesNum_; ++j) {
                    if (darts < wBuffer[j]) {
                        particles_[i].setPose(tmpParticles[j].getPose());
                        particles_[i].setW(wo);
                        break;
                    }
                }
            }
        } else {
            // resampling and add random particles
            double randomParticlesRate = randomParticlesRate_;
            if (useAugmentedMCL_ && amclRandomParticlesRate_ > 0.0) {
                omegaSlow_ = omegaFast_ = 0.0;
                randomParticlesRate = amclRandomParticlesRate_;
            } else if (!addRandomParticlesInResampling_) {
                randomParticlesRate = 0.0;
            }
            int resampledParticlesNum = (int)((1.0 - randomParticlesRate) * (double)particlesNum_);
            int randomParticlesNum = particlesNum_ - resampledParticlesNum;
            for (int i = 0; i < resampledParticlesNum; ++i) {
                double darts = (double)rand() / ((double)RAND_MAX + 1.0);
                for (int j = 0; j < particlesNum_; ++j) {
                    if (darts < wBuffer[j]) {
                        particles_[i].setPose(tmpParticles[j].getPose());
                        particles_[i].setW(wo);
                        break;
                    }
                }
            }
            double xo = mclPose_.getX();
            double yo = mclPose_.getY();
            double yawo = mclPose_.getYaw();
            for (int i = resampledParticlesNum; i < resampledParticlesNum + randomParticlesNum; ++i) {
                double x = xo + nrand(randomParticlesNoise_[0]);
                double y = yo + nrand(randomParticlesNoise_[1]);
                double yaw = yawo + nrand(randomParticlesNoise_[2]);
                particles_[i].setPose(x, y, yaw);
                particles_[i].setW(wo);
            }
        }
    }

    std::vector<float> getResidualErrors(void) {
        double yaw = mclPose_.getYaw();
        double sensorX = baseLink2Laser_.getX() * cos(yaw) - baseLink2Laser_.getY() * sin(yaw) + mclPose_.getX();
        double sensorY = baseLink2Laser_.getX() * sin(yaw) + baseLink2Laser_.getY() * cos(yaw) + mclPose_.getY();
        double sensorYaw = baseLink2Laser_.getYaw() + yaw;
        std::vector<float> residualErrors((int)scan_.ranges.size());
        for (int i = 0; i < (int)scan_.ranges.size(); ++i) {
            double r = scan_.ranges[i];
            if (r <= scan_.range_min || scan_.range_max <= r) {
                residualErrors[i] = -1.0;
                continue;
            }
            double t = (double)i * scan_.angle_increment + scan_.angle_min + sensorYaw;
            double x = r * cos(t) + sensorX;
            double y = r * sin(t) + sensorY;
            int u, v;
            xy2uv(x, y, &u, &v);
            if (onMap(u, v)) {
                float dist = distMap_.at<float>(v, u);
                residualErrors[i] = dist;
            } else {
                residualErrors[i] = -1.0;
            }
        }
        return residualErrors;
    }

    void printResult(void) {
        std::cout << "MCL: x = " << mclPose_.getX() << " [m], y = " << mclPose_.getY() << " [m], yaw = " << mclPose_.getYaw() * rad2deg_ << " [deg]" << std::endl;
        std::cout << "Odom: x = " << odomPose_.getX() << " [m], y = " << odomPose_.getY() << " [m], yaw = " << odomPose_.getYaw() * rad2deg_ << " [deg]" << std::endl;
        std::cout << "total likelihood = " << totalLikelihood_ << std::endl;
        std::cout << "average likelihood = " << averageLikelihood_ << std::endl;
        std::cout << "max likelihood = " << maxLikelihood_ << std::endl;
        std::cout << "effective sample size = " << effectiveSampleSize_ << std::endl;
        std::cout << "amcl random particles rate = " << amclRandomParticlesRate_ << std::endl;
        std::cout << std::endl;
    }

    void publishROSMessages(void) {
        // pose
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = mapFrame_;
        pose.header.stamp = mclPoseStamp_;
        pose.pose.position.x = mclPose_.getX();
        pose.pose.position.y = mclPose_.getY();
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(mclPose_.getYaw());
        posePub_.publish(pose);

        // particles
        geometry_msgs::PoseArray particlesPoses;
        particlesPoses.header.frame_id = mapFrame_;
        particlesPoses.header.stamp = mclPoseStamp_;
        particlesPoses.poses.resize(particlesNum_);
        for (int i = 0; i < particlesNum_; ++i) {
            geometry_msgs::Pose pose;
            pose.position.x = particles_[i].getX();
            pose.position.y = particles_[i].getY();
            pose.orientation = tf::createQuaternionMsgFromYaw(particles_[i].getYaw());
            particlesPoses.poses[i] = pose;
        }
        particlesPub_.publish(particlesPoses);

        // tf from odom to base link (mcl frame)
        if (useOdomFrame_) {
            double dx = mclPose_.getX() - odomPose_.getX();
            double dy = mclPose_.getY() - odomPose_.getY();
            double dyaw = mclPose_.getYaw() - odomPose_.getYaw();
            double l = sqrt(dx * dx + dy * dy);
            double t = atan2(dy, dx) - odomPose_.getYaw();
            double x = l * cos(t);
            double y = l * sin(t);
            tf::Transform tf;
            tf::Quaternion q;
            tf.setOrigin(tf::Vector3(x, y, 0.0));
            q.setRPY(0.0, 0.0, dyaw);
            tf.setRotation(q);
            tfBroadcaster_.sendTransform(tf::StampedTransform(tf, mclPoseStamp_, odomFrame_, baseLinkFrame_));
        } else {
            tf::Transform tf;
            tf::Quaternion q;
            tf.setOrigin(tf::Vector3(mclPose_.getX(), mclPose_.getY(), 0.0));
            q.setRPY(0.0, 0.0, mclPose_.getYaw());
            tf.setRotation(q);
            tfBroadcaster_.sendTransform(tf::StampedTransform(tf, mclPoseStamp_, mapFrame_, baseLinkFrame_));
        }

        // unknown scan
        if (publishUnknownScan_ && (rejectUnknownScan_ || measurementModelType_ == 2))
            unknownScanPub_.publish(unknownScan_);

        // residual errors
        if (publishResidualErrors_) {
            sensor_msgs::LaserScan residualErrors = scan_;
            residualErrors.ranges = getResidualErrors();
            residualErrorsPub_.publish(residualErrors);
        }
    }

private:
    inline double nrand(double n) { return (n * sqrt(-2.0 * log((double)rand() / RAND_MAX)) * cos(2.0 * M_PI * rand() / RAND_MAX)); }

    inline bool onMap(int u, int v) {
        if (0 <= u && u < mapWidth_ && 0 <= v && v < mapHeight_)
            return true;
        else
            return false;
    }

    inline void xy2uv(double x, double y, int *u, int *v) {
        double dx = x - mapOrigin_.getX();
        double dy = y - mapOrigin_.getY();
        double yaw = -mapOrigin_.getYaw();
        double xx = dx * cos(yaw) - dy * sin(yaw);
        double yy = dx * sin(yaw) + dy * cos(yaw);
        *u = (int)(xx / mapResolution_);
        *v = (int)(yy / mapResolution_);
    }

    void scanCB(const sensor_msgs::LaserScan::ConstPtr &msg) {
        if (canUpdateScan_)
            scan_ = *msg;
        if (!gotScan_)
            gotScan_ = true;
    }

    void odomCB(const nav_msgs::Odometry::ConstPtr &msg) {
        static double prevTime;
        double currTime = msg->header.stamp.toSec();
        if (isInitialized_) {
            prevTime = currTime;
            isInitialized_ = false;
            return;
        }
        odomPoseStamp_ = msg->header.stamp;
        double deltaTime = currTime - prevTime;
        deltaX_ += msg->twist.twist.linear.x * deltaTime;
        deltaY_ += msg->twist.twist.linear.y * deltaTime;
        deltaDist_ += msg->twist.twist.linear.x * deltaTime;
        deltaYaw_ += msg->twist.twist.angular.z * deltaTime;
        deltaTimeSum_ += deltaTime;

        tf::Quaternion q(msg->pose.pose.orientation.x, 
            msg->pose.pose.orientation.y, 
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        odomPose_.setPose(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);

        prevTime = currTime;
    }

    void mapCB(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
        // perform distance transform to build the distance field
        mapWidth_ = msg->info.width;
        mapHeight_ = msg->info.height;
        mapResolution_ = msg->info.resolution;
        cv::Mat binMap(mapHeight_, mapWidth_, CV_8UC1);
        for (int v = 0; v < mapHeight_; v++) {
            for (int u = 0; u < mapWidth_; u++) {
                int node = v * mapWidth_ + u;
                int val = msg->data[node];
                if (val == 100)
                    binMap.at<uchar>(v, u) = 0;
                else
                    binMap.at<uchar>(v, u) = 1;
            }
        }
        cv::Mat distMap(mapHeight_, mapWidth_, CV_32FC1);
        cv::distanceTransform(binMap, distMap, cv::DIST_L2, 5);
        for (int v = 0; v < mapHeight_; v++) {
            for (int u = 0; u < mapWidth_; u++) {
                float d = distMap.at<float>(v, u) * (float)mapResolution_;
                distMap.at<float>(v, u) = d;
            }
        }
        distMap_ = distMap;
        tf::Quaternion q(msg->info.origin.orientation.x, 
            msg->info.origin.orientation.y, 
            msg->info.origin.orientation.z,
            msg->info.origin.orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        mapOrigin_.setX(msg->info.origin.position.x);
        mapOrigin_.setY(msg->info.origin.position.y);
        mapOrigin_.setYaw(yaw);
        gotMap_ = true;
    }

    void initialPoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
        tf::Quaternion q(msg->pose.pose.orientation.x, 
            msg->pose.pose.orientation.y, 
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        mclPose_.setPose(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
        resetParticlesDistribution();
        isInitialized_ = true;
    }

    void resetParticlesDistribution(void) {
        particles_.resize(particlesNum_);
        double xo = mclPose_.getX();
        double yo = mclPose_.getY();
        double yawo = mclPose_.getYaw();
        double wo = 1.0 / (double)particlesNum_;
        for (int i = 0; i < particlesNum_; ++i) {
            double x = xo + nrand(initialNoise_[0]);
            double y = yo + nrand(initialNoise_[1]);
            double yaw = yawo + nrand(initialNoise_[2]);
            particles_[i].setPose(x, y, yaw);
            particles_[i].setW(wo);
        }
    }

    void rejectUnknownScan(void) {
        unknownScan_ = scan_;
        double xo = baseLink2Laser_.getX();
        double yo = baseLink2Laser_.getY();
        double yawo = baseLink2Laser_.getYaw();
        double hitThreshold = 0.5 * mapResolution_;
        for (int i = 0; i < (int)unknownScan_.ranges.size(); ++i) {
            if (i % scanStep_ != 0) {
                unknownScan_.ranges[i] = 0.0;
                continue;
            }
            double r = unknownScan_.ranges[i];
            if (r <= unknownScan_.range_min || unknownScan_.range_max <= r) {
                unknownScan_.ranges[i] = 0.0;
                continue;
            }
            double laserYaw = (double)i * unknownScan_.angle_increment + unknownScan_.angle_min;
            double pShortSum = 0.0, pBeamSum = 0.0;
            for (int j = 0; j < particlesNum_; ++j) {
                double yaw = particles_[j].getYaw();
                double x = xo * cos(yaw) - yo * sin(yaw) + particles_[j].getX();
                double y = xo * sin(yaw) + yo * cos(yaw) + particles_[j].getY();
                double t = yawo + yaw + laserYaw;
                double dx = mapResolution_ * cos(t);
                double dy = mapResolution_ * sin(t);
                int u, v;
                double expectedRange = -1.0;
                for (double range = 0.0; range <= unknownScan_.range_max; range += mapResolution_) {
                    xy2uv(x, y, &u, &v);
                    if (onMap(u, v)) {
                        double dist = (double)distMap_.at<float>(v, u);
                        if (dist < hitThreshold) {
                            expectedRange = range;
                            break;
                        }
                    } else {
                        break;
                    }
                    x += dx;
                    y += dy;
                }
                if (r <= expectedRange) {
                    double error = expectedRange - r;
                    double pHit = normConstHit_ * exp(-(error * error) * denomHit_) * mapResolution_;
                    double pShort = lambdaShort_ * exp(-lambdaShort_ * r) / (1.0 - exp(-lambdaShort_ * unknownScan_.range_max)) * mapResolution_;
                    pShortSum += pShort;
                    pBeamSum += zHit_ * pHit + zShort_ * pShort + measurementModelRandom_;
                } else {
                    pBeamSum += measurementModelRandom_;
                }
            }
            double pShort = pShortSum;
            double pBeam = pBeamSum;
            double pUnknown = pShortSum / pBeamSum;
            if (pUnknown < unknownScanProbThreshold_) {
                unknownScan_.ranges[i] = 0.0;
            } else {
                // unknown scan is rejected from the scan message used for localization
                scan_.ranges[i] = 0.0;
            }
        }
    }

    double calculateLikelihoodFieldModel(Pose pose, double range, double rangeAngle) {
        if (range <= scan_.range_min || scan_.range_max <= range)
            return measurementModelInvalidScan_;

        double t = pose.getYaw() + rangeAngle;
        double x = range * cos(t) + pose.getX();
        double y = range * sin(t) + pose.getY();
        int u, v;
        xy2uv(x, y, &u, &v);
        double p;
        if (onMap(u, v)) {
            double dist = (double)distMap_.at<float>(v, u);
            double pHit = normConstHit_ * exp(-(dist * dist) * denomHit_) * mapResolution_;
            p = zHit_ * pHit + measurementModelRandom_;
        } else {
            p = measurementModelRandom_;
        }
        if (p > 1.0)
            p = 1.0;
        return p;
    }

    double calculateBeamModel(Pose pose, double range, double rangeAngle) {
        if (range <= scan_.range_min || scan_.range_max <= range)
            return measurementModelInvalidScan_;

        double t = pose.getYaw() + rangeAngle;
        double x = pose.getX();
        double y = pose.getY();
        double dx = mapResolution_ * cos(t);
        double dy = mapResolution_ * sin(t);
        int u, v;
        double expectedRange = -1.0;
        double hitThreshold = 0.5 * mapResolution_;
        for (double r = 0.0; r < scan_.range_max; r += mapResolution_) {
            xy2uv(x, y, &u, &v);
            if (onMap(u, v)) {
                double dist = (double)distMap_.at<float>(v, u);
                if (dist < hitThreshold) {
                    expectedRange = r;
                    break;
                }
            } else {
                break;
            }
            x += dx;
            y += dy;
        }

        double p;
        if (range <= expectedRange) {
            double error = expectedRange - range;
            double pHit = normConstHit_ * exp(-(error * error) * denomHit_) * mapResolution_;
            double pShort = lambdaShort_ * exp(-lambdaShort_ * range) / (1.0 - exp(-lambdaShort_ * scan_.range_max)) * mapResolution_;
            p = zHit_ * pHit + zShort_ * pShort + measurementModelRandom_;
        } else {
            p = measurementModelRandom_;
        }
        if (p > 1.0)
            p = 1.0;
        return p;
    }

    double calculateClassConditionalMeasurementModel(Pose pose, double range, double rangeAngle) {
        if (range <= scan_.range_min || scan_.range_max <= range)
            return measurementModelInvalidScan_;

        double t = pose.getYaw() + rangeAngle;
        double x = range * cos(t) + pose.getX();
        double y = range * sin(t) + pose.getY();
        double pUnknown = lambdaUnknown_ * exp(-lambdaUnknown_ * range) / (1.0 - exp(-lambdaUnknown_ * scan_.range_max)) * mapResolution_ * pUnknownPrior_;
        int u, v;
        xy2uv(x, y, &u, &v);
        double p = pUnknown;
        if (onMap(u, v)) {
            double dist = (double)distMap_.at<float>(v, u);
            double pHit = normConstHit_ * exp(-(dist * dist) * denomHit_) * mapResolution_;
            p += (zHit_ * pHit + measurementModelRandom_) * pKnownPrior_;
        } else {
            p += measurementModelRandom_ * pKnownPrior_;
        }
        if (p > 1.0)
            p = 1.0;
        return p;
    }

    double estimateUnknownScanWithClassConditionalMeasurementModel(Pose pose) {
        unknownScan_ = scan_;
        double sensorX = pose.getX();
        double sensorY = pose.getY();
        double sensorYaw = pose.getYaw();
        for (int i = 0; i < (int)unknownScan_.ranges.size(); ++i) {
            double r = unknownScan_.ranges[i];
            if (r <= unknownScan_.range_min || unknownScan_.range_max <= r) {
                unknownScan_.ranges[i] = 0.0;
                continue;
            }
            double t = sensorYaw + (double)i * unknownScan_.angle_increment + unknownScan_.angle_min;
            double x = r * cos(t) + sensorX;
            double y = r * sin(t) + sensorY;
            int u, v;
            xy2uv(x, y, &u, &v);
            double pKnown;
            double pUnknown = lambdaUnknown_ * exp(-lambdaUnknown_ * r) / (1.0 - exp(-lambdaUnknown_ * unknownScan_.range_max)) * mapResolution_ * pUnknownPrior_;
            if (onMap(u, v)) {
                double dist = (double)distMap_.at<float>(v, u);
                double pHit = normConstHit_ * exp(-(dist * dist) * denomHit_) * mapResolution_;
                pKnown = (zHit_ * pHit + measurementModelRandom_) * pKnownPrior_;
            } else {
                pKnown = measurementModelRandom_ * pKnownPrior_;
            }
            double sum = pKnown + pUnknown;
            pUnknown /= sum;
            if (pUnknown < unknownScanProbThreshold_)
                unknownScan_.ranges[i] = 0.0;
        }
    }

}; // class MCL

} // namespace mcl_ros

#endif // __MCL_H__
