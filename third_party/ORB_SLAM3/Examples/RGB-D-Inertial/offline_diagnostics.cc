#include "offline_diagnostics.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <numeric>
#include <sstream>

#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace orb3_offline
{
namespace
{

constexpr double kRadiansToDegrees = 180.0 / M_PI;

Eigen::Matrix3d ExpSO3(const Eigen::Vector3d &rotation_vector)
{
    const double angle = rotation_vector.norm();
    if(angle < 1e-12)
        return Eigen::Matrix3d::Identity();
    return Eigen::AngleAxisd(angle, rotation_vector / angle).toRotationMatrix();
}

Eigen::Vector3d LogSO3(const Eigen::Matrix3d &rotation)
{
    Eigen::AngleAxisd angle_axis(rotation);
    if(!std::isfinite(angle_axis.angle()))
        return Eigen::Vector3d::Constant(
            std::numeric_limits<double>::quiet_NaN());
    return angle_axis.axis() * angle_axis.angle();
}

Eigen::Vector3d PointToEigen(const cv::Point3f &point)
{
    return Eigen::Vector3d(point.x, point.y, point.z);
}

bool InterpolateGyro(const std::vector<ImuSample> &samples,
                     double timestamp,
                     Eigen::Vector3d &gyro)
{
    if(samples.size() < 2 || timestamp < samples.front().timestamp ||
       timestamp > samples.back().timestamp)
        return false;

    auto upper = std::upper_bound(
        samples.begin(), samples.end(), timestamp,
        [](double time, const ImuSample &sample) {
            return time < sample.timestamp;
        });

    if(upper == samples.begin())
        return false;
    if(upper == samples.end())
    {
        gyro = PointToEigen(samples.back().gyro);
        return true;
    }

    const ImuSample &before = *(upper - 1);
    const ImuSample &after = *upper;
    const double interval = after.timestamp - before.timestamp;
    const double alpha = interval > 0.0
        ? (timestamp - before.timestamp) / interval : 0.0;
    gyro = (1.0 - alpha) * PointToEigen(before.gyro) +
           alpha * PointToEigen(after.gyro);
    return true;
}

double ReadDepthMeters(const cv::Mat &depth, int x, int y, double factor)
{
    if(x < 0 || y < 0 || x >= depth.cols || y >= depth.rows || factor <= 0.0)
        return std::numeric_limits<double>::quiet_NaN();

    double value = 0.0;
    if(depth.type() == CV_16UC1)
        value = static_cast<double>(depth.at<uint16_t>(y, x)) / factor;
    else if(depth.type() == CV_32FC1)
        value = static_cast<double>(depth.at<float>(y, x)) / factor;
    else if(depth.type() == CV_64FC1)
        value = depth.at<double>(y, x) / factor;
    else
        return std::numeric_limits<double>::quiet_NaN();
    return value;
}

cv::Mat ToGray(const cv::Mat &image)
{
    if(image.channels() == 1)
        return image;
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    return gray;
}

}  // namespace

VisualRotationEstimator::VisualRotationEstimator(const Calibration &calibration,
                                                 int feature_count)
    : calibration_(calibration),
      orb_(cv::ORB::create(feature_count)),
      matcher_(cv::NORM_HAMMING, false)
{
}

VisualRotationResult VisualRotationEstimator::Estimate(
    const cv::Mat &previous_rgb,
    const cv::Mat &previous_depth,
    const cv::Mat &current_rgb)
{
    VisualRotationResult result;
    if(previous_rgb.empty() || previous_depth.empty() || current_rgb.empty())
        return result;

    std::vector<cv::KeyPoint> previous_keypoints;
    std::vector<cv::KeyPoint> current_keypoints;
    cv::Mat previous_descriptors;
    cv::Mat current_descriptors;
    orb_->detectAndCompute(ToGray(previous_rgb), cv::noArray(),
                           previous_keypoints, previous_descriptors);
    orb_->detectAndCompute(ToGray(current_rgb), cv::noArray(),
                           current_keypoints, current_descriptors);
    result.previous_features = static_cast<int>(previous_keypoints.size());
    result.current_features = static_cast<int>(current_keypoints.size());
    if(previous_descriptors.empty() || current_descriptors.empty())
        return result;

    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher_.knnMatch(previous_descriptors, current_descriptors, knn_matches, 2);

    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> image_points;
    const double fx = calibration_.camera_matrix.at<double>(0, 0);
    const double fy = calibration_.camera_matrix.at<double>(1, 1);
    const double cx = calibration_.camera_matrix.at<double>(0, 2);
    const double cy = calibration_.camera_matrix.at<double>(1, 2);
    for(const auto &neighbors : knn_matches)
    {
        if(neighbors.size() < 2 || neighbors[0].distance >= 0.75f * neighbors[1].distance)
            continue;
        ++result.ratio_matches;

        const cv::KeyPoint &previous = previous_keypoints[neighbors[0].queryIdx];
        const cv::KeyPoint &current = current_keypoints[neighbors[0].trainIdx];
        const int x = cvRound(previous.pt.x);
        const int y = cvRound(previous.pt.y);
        const double z = ReadDepthMeters(
            previous_depth, x, y, calibration_.depth_map_factor);
        if(!std::isfinite(z) || z < 0.15 || z > 8.0)
            continue;

        object_points.emplace_back(
            static_cast<float>((previous.pt.x - cx) * z / fx),
            static_cast<float>((previous.pt.y - cy) * z / fy),
            static_cast<float>(z));
        image_points.push_back(current.pt);
    }
    result.depth_matches = static_cast<int>(object_points.size());
    if(object_points.size() < 12)
        return result;

    cv::Mat rotation_vector;
    cv::Mat translation_vector;
    cv::Mat inlier_indices;
    const bool solved = cv::solvePnPRansac(
        object_points, image_points, calibration_.camera_matrix, cv::noArray(),
        rotation_vector, translation_vector, false, 200, 2.5, 0.999,
        inlier_indices, cv::SOLVEPNP_EPNP);
    if(!solved || inlier_indices.rows < 6)
        return result;

    std::vector<cv::Point3f> inlier_objects;
    std::vector<cv::Point2f> inlier_images;
    inlier_objects.reserve(inlier_indices.rows);
    inlier_images.reserve(inlier_indices.rows);
    bool occupied[48] = {false};
    for(int row = 0; row < inlier_indices.rows; ++row)
    {
        const int index = inlier_indices.at<int>(row, 0);
        inlier_objects.push_back(object_points[index]);
        inlier_images.push_back(image_points[index]);
        const int cell_x = std::clamp(
            static_cast<int>(image_points[index].x * 8.0 / current_rgb.cols), 0, 7);
        const int cell_y = std::clamp(
            static_cast<int>(image_points[index].y * 6.0 / current_rgb.rows), 0, 5);
        occupied[cell_y * 8 + cell_x] = true;
    }
    result.inliers = static_cast<int>(inlier_objects.size());
    result.occupied_cells = static_cast<int>(
        std::count(std::begin(occupied), std::end(occupied), true));
    result.inlier_ratio = static_cast<double>(result.inliers) /
                          std::max(1, result.depth_matches);

    cv::solvePnPRefineLM(
        inlier_objects, inlier_images, calibration_.camera_matrix, cv::noArray(),
        rotation_vector, translation_vector);

    std::vector<cv::Point2f> projections;
    cv::projectPoints(inlier_objects, rotation_vector, translation_vector,
                      calibration_.camera_matrix, cv::noArray(), projections);
    double squared_error = 0.0;
    for(size_t index = 0; index < projections.size(); ++index)
    {
        const cv::Point2f error = projections[index] - inlier_images[index];
        squared_error += error.dot(error);
    }
    result.reprojection_rmse_px = std::sqrt(
        squared_error / std::max<size_t>(1, projections.size()));

    cv::Mat rotation_cv;
    cv::Rodrigues(rotation_vector, rotation_cv);
    rotation_cv.convertTo(rotation_cv, CV_64F);
    for(int row = 0; row < 3; ++row)
        for(int column = 0; column < 3; ++column)
            result.rotation_current_from_previous(row, column) =
                rotation_cv.at<double>(row, column);
    translation_vector.convertTo(translation_vector, CV_64F);
    for(int row = 0; row < 3; ++row)
        result.translation_current_from_previous(row) =
            translation_vector.at<double>(row, 0);

    result.camera_displacement_previous =
        -result.rotation_current_from_previous.transpose() *
        result.translation_current_from_previous;
    result.camera_translation_norm = result.camera_displacement_previous.norm();

    // Convert T_c2_c1 from solvePnP to T_b2_b1 using the configured camera
    // to body transform, then invert its translation to obtain the motion of
    // the current body origin expressed in the previous body frame.
    const Eigen::Matrix3d rotation_body_current_from_body_previous =
        calibration_.rotation_body_from_camera *
        result.rotation_current_from_previous *
        calibration_.rotation_body_from_camera.transpose();
    const Eigen::Vector3d translation_body_current_from_body_previous =
        calibration_.rotation_body_from_camera *
        result.translation_current_from_previous +
        calibration_.translation_body_from_camera -
        rotation_body_current_from_body_previous *
        calibration_.translation_body_from_camera;
    result.body_displacement_previous =
        -rotation_body_current_from_body_previous.transpose() *
        translation_body_current_from_body_previous;
    result.body_translation_norm = result.body_displacement_previous.norm();
    result.rotation_angle_deg = RotationAngleDegrees(
        result.rotation_current_from_previous);
    result.valid = result.inliers >= 30 && result.inlier_ratio >= 0.20 &&
                   result.occupied_cells >= 8 &&
                   result.reprojection_rmse_px <= 3.0;
    return result;
}

bool LoadCalibration(const std::string &settings_path,
                     double image_scale,
                     Calibration &calibration,
                     std::string &error)
{
    cv::FileStorage settings(settings_path, cv::FileStorage::READ);
    if(!settings.isOpened())
    {
        error = "cannot open settings file";
        return false;
    }

    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    settings["Camera1.fx"] >> fx;
    settings["Camera1.fy"] >> fy;
    settings["Camera1.cx"] >> cx;
    settings["Camera1.cy"] >> cy;
    settings["RGBD.DepthMapFactor"] >> calibration.depth_map_factor;
    if(fx <= 0.0 || fy <= 0.0 || calibration.depth_map_factor <= 0.0)
    {
        error = "invalid camera intrinsics or depth factor";
        return false;
    }
    calibration.camera_matrix = (cv::Mat_<double>(3, 3) <<
        fx * image_scale, 0.0, cx * image_scale,
        0.0, fy * image_scale, cy * image_scale,
        0.0, 0.0, 1.0);

    cv::Mat transform;
    settings["IMU.T_b_c1"] >> transform;
    if(transform.rows != 4 || transform.cols != 4)
    {
        error = "IMU.T_b_c1 must be a 4x4 matrix";
        return false;
    }
    transform.convertTo(transform, CV_64F);
    for(int row = 0; row < 3; ++row)
    {
        for(int column = 0; column < 3; ++column)
            calibration.rotation_body_from_camera(row, column) =
                transform.at<double>(row, column);
        calibration.translation_body_from_camera(row) =
            transform.at<double>(row, 3);
    }
    return true;
}

Eigen::Vector3d EstimateInitialGyroBias(const std::vector<ImuSample> &samples,
                                       double start_time,
                                       double duration_seconds,
                                       size_t &used_samples)
{
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    used_samples = 0;
    for(const ImuSample &sample : samples)
    {
        if(sample.timestamp < start_time)
            continue;
        if(sample.timestamp > start_time + duration_seconds)
            break;
        const Eigen::Vector3d gyro = PointToEigen(sample.gyro);
        const double acceleration_norm = PointToEigen(sample.acceleration).norm();
        if(gyro.norm() < 0.05 && std::abs(acceleration_norm - 9.80665) < 0.5)
        {
            sum += gyro;
            ++used_samples;
        }
    }
    if(used_samples >= 100)
        return sum / static_cast<double>(used_samples);
    return Eigen::Vector3d::Zero();
}

ImuIntegrationResult IntegrateGyroscope(const std::vector<ImuSample> &samples,
                                        double start_time,
                                        double end_time,
                                        const Eigen::Vector3d &gyro_bias)
{
    ImuIntegrationResult result;
    if(end_time <= start_time || samples.size() < 2 ||
       start_time < samples.front().timestamp || end_time > samples.back().timestamp)
        return result;

    auto first_at_or_after = std::lower_bound(
        samples.begin(), samples.end(), start_time,
        [](const ImuSample &sample, double time) {
            return sample.timestamp < time;
        });
    auto last_at_or_before = std::upper_bound(
        samples.begin(), samples.end(), end_time,
        [](double time, const ImuSample &sample) {
            return time < sample.timestamp;
        });
    if(first_at_or_after != samples.end())
        result.first_sample_dt = first_at_or_after->timestamp - start_time;
    if(last_at_or_before != samples.begin())
        result.last_sample_dt = end_time - (last_at_or_before - 1)->timestamp;
    result.sample_count = static_cast<int>(std::distance(
        first_at_or_after, last_at_or_before));

    std::vector<double> boundaries;
    boundaries.push_back(start_time);
    for(auto iterator = first_at_or_after; iterator != last_at_or_before; ++iterator)
        if(iterator->timestamp > start_time && iterator->timestamp < end_time)
            boundaries.push_back(iterator->timestamp);
    boundaries.push_back(end_time);

    result.maximum_gap = 0.0;
    Eigen::Matrix3d delta_body = Eigen::Matrix3d::Identity();
    for(size_t index = 1; index < boundaries.size(); ++index)
    {
        const double left_time = boundaries[index - 1];
        const double right_time = boundaries[index];
        Eigen::Vector3d left_gyro;
        Eigen::Vector3d right_gyro;
        if(!InterpolateGyro(samples, left_time, left_gyro) ||
           !InterpolateGyro(samples, right_time, right_gyro))
            return result;
        const double interval = right_time - left_time;
        result.maximum_gap = std::max(result.maximum_gap, interval);
        const Eigen::Vector3d average_gyro =
            0.5 * (left_gyro + right_gyro) - gyro_bias;
        delta_body = delta_body * ExpSO3(average_gyro * interval);
    }

    // Gyro integration gives ^b_prev R_b_current.  PnP gives the inverse
    // coordinate transform (^c_current R_c_previous), so store the inverse.
    result.rotation_previous_from_current = delta_body.transpose();
    result.rotation_angle_deg = RotationAngleDegrees(
        result.rotation_previous_from_current);
    result.valid = result.sample_count >= 2;
    return result;
}

RotationResidual CompareRotations(
    const VisualRotationResult &visual,
    const ImuIntegrationResult &imu,
    const Eigen::Matrix3d &rotation_body_from_camera)
{
    RotationResidual residual;
    if(!visual.valid || !imu.valid)
        return residual;

    const Eigen::Matrix3d rotation_camera_from_body =
        rotation_body_from_camera.transpose();
    const Eigen::Matrix3d imu_in_camera =
        rotation_camera_from_body * imu.rotation_previous_from_current *
        rotation_body_from_camera;
    const Eigen::Matrix3d error =
        imu_in_camera.transpose() * visual.rotation_current_from_previous;
    const Eigen::Vector3d error_vector = LogSO3(error) * kRadiansToDegrees;
    residual.roll_deg = error_vector.x();
    residual.pitch_deg = error_vector.y();
    residual.yaw_deg = error_vector.z();
    residual.angle_deg = error_vector.norm();
    residual.imu_camera_rotation_deg = RotationAngleDegrees(imu_in_camera);
    residual.valid = std::isfinite(residual.angle_deg);
    return residual;
}

double InferImageSourceTimestamp(const std::string &image_path)
{
    const std::string stem = std::filesystem::path(image_path).stem().string();
    if(stem.empty() || !std::all_of(stem.begin(), stem.end(), ::isdigit))
        return std::numeric_limits<double>::quiet_NaN();
    try
    {
        return static_cast<double>(std::stoll(stem)) * 1e-9;
    }
    catch(const std::exception &)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
}

double RotationAngleDegrees(const Eigen::Matrix3d &rotation)
{
    const double cosine = std::clamp((rotation.trace() - 1.0) * 0.5, -1.0, 1.0);
    return std::acos(cosine) * kRadiansToDegrees;
}

}  // namespace orb3_offline
