#ifndef ORB_SLAM3_OFFLINE_DIAGNOSTICS_H
#define ORB_SLAM3_OFFLINE_DIAGNOSTICS_H

#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

namespace orb3_offline
{

struct ImuSample
{
    double timestamp = 0.0;
    cv::Point3f gyro;
    cv::Point3f acceleration;
};

struct Calibration
{
    cv::Mat camera_matrix;
    double depth_map_factor = 1000.0;
    Eigen::Matrix3d rotation_body_from_camera = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation_body_from_camera = Eigen::Vector3d::Zero();
};

struct VisualRotationResult
{
    bool valid = false;
    int previous_features = 0;
    int current_features = 0;
    int ratio_matches = 0;
    int depth_matches = 0;
    int inliers = 0;
    int occupied_cells = 0;
    double inlier_ratio = std::numeric_limits<double>::quiet_NaN();
    double reprojection_rmse_px = std::numeric_limits<double>::quiet_NaN();
    double rotation_angle_deg = std::numeric_limits<double>::quiet_NaN();
    Eigen::Matrix3d rotation_current_from_previous = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation_current_from_previous = Eigen::Vector3d::Zero();
    Eigen::Vector3d camera_displacement_previous = Eigen::Vector3d::Zero();
    Eigen::Vector3d body_displacement_previous = Eigen::Vector3d::Zero();
    double camera_translation_norm = std::numeric_limits<double>::quiet_NaN();
    double body_translation_norm = std::numeric_limits<double>::quiet_NaN();
};

struct ImuIntegrationResult
{
    bool valid = false;
    int sample_count = 0;
    double first_sample_dt = std::numeric_limits<double>::quiet_NaN();
    double last_sample_dt = std::numeric_limits<double>::quiet_NaN();
    double maximum_gap = std::numeric_limits<double>::quiet_NaN();
    double rotation_angle_deg = std::numeric_limits<double>::quiet_NaN();
    Eigen::Matrix3d rotation_previous_from_current = Eigen::Matrix3d::Identity();
};

struct RotationResidual
{
    bool valid = false;
    double roll_deg = std::numeric_limits<double>::quiet_NaN();
    double pitch_deg = std::numeric_limits<double>::quiet_NaN();
    double yaw_deg = std::numeric_limits<double>::quiet_NaN();
    double angle_deg = std::numeric_limits<double>::quiet_NaN();
    double imu_camera_rotation_deg = std::numeric_limits<double>::quiet_NaN();
};

class VisualRotationEstimator
{
public:
    explicit VisualRotationEstimator(const Calibration &calibration,
                                     int feature_count = 2000);

    VisualRotationResult Estimate(const cv::Mat &previous_rgb,
                                  const cv::Mat &previous_depth,
                                  const cv::Mat &current_rgb);

private:
    Calibration calibration_;
    cv::Ptr<cv::ORB> orb_;
    cv::BFMatcher matcher_;
};

bool LoadCalibration(const std::string &settings_path,
                     double image_scale,
                     Calibration &calibration,
                     std::string &error);

Eigen::Vector3d EstimateInitialGyroBias(const std::vector<ImuSample> &samples,
                                       double start_time,
                                       double duration_seconds,
                                       size_t &used_samples);

ImuIntegrationResult IntegrateGyroscope(const std::vector<ImuSample> &samples,
                                        double start_time,
                                        double end_time,
                                        const Eigen::Vector3d &gyro_bias);

RotationResidual CompareRotations(const VisualRotationResult &visual,
                                  const ImuIntegrationResult &imu,
                                  const Eigen::Matrix3d &rotation_body_from_camera);

double InferImageSourceTimestamp(const std::string &image_path);
double RotationAngleDegrees(const Eigen::Matrix3d &rotation);

}  // namespace orb3_offline

#endif
