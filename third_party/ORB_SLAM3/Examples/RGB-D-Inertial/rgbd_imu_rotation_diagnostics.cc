#include "offline_diagnostics.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

using namespace std;
using orb3_offline::ImuSample;

namespace
{

bool LoadAssociations(const string &path,
                      vector<double> &timestamps,
                      vector<string> &rgb_paths,
                      vector<string> &depth_paths)
{
    ifstream input(path);
    string line;
    while(getline(input, line))
    {
        const size_t first = line.find_first_not_of(" \t\r\n");
        if(first == string::npos || line[first] == '#')
            continue;
        stringstream stream(line);
        double rgb_time = 0.0;
        double depth_time = 0.0;
        string rgb_path;
        string depth_path;
        if(!(stream >> rgb_time >> rgb_path >> depth_time >> depth_path))
            return false;
        timestamps.push_back(rgb_time);
        rgb_paths.push_back(rgb_path);
        depth_paths.push_back(depth_path);
    }
    return !timestamps.empty();
}

bool LoadImu(const string &path, vector<ImuSample> &samples)
{
    ifstream input(path);
    string line;
    while(getline(input, line))
    {
        const size_t first = line.find_first_not_of(" \t\r\n");
        if(first == string::npos || line[first] == '#')
            continue;
        replace(line.begin(), line.end(), ',', ' ');
        stringstream stream(line);
        ImuSample sample;
        if(!(stream >> sample.timestamp
                    >> sample.gyro.x >> sample.gyro.y >> sample.gyro.z
                    >> sample.acceleration.x >> sample.acceleration.y
                    >> sample.acceleration.z))
            return false;
        samples.push_back(sample);
    }
    return !samples.empty();
}

double Percentile(vector<double> values, double fraction)
{
    if(values.empty())
        return numeric_limits<double>::quiet_NaN();
    sort(values.begin(), values.end());
    const size_t index = static_cast<size_t>(
        round(fraction * static_cast<double>(values.size() - 1)));
    return values[index];
}

}  // namespace

int main(int argc, char **argv)
{
    if(argc != 6)
    {
        cerr << "Usage: ./rgbd_imu_rotation_diagnostics path_to_settings "
             << "path_to_sequence path_to_association path_to_imu output_csv" << endl;
        return 1;
    }

    vector<double> timestamps;
    vector<string> rgb_paths;
    vector<string> depth_paths;
    vector<ImuSample> imu_samples;
    if(!LoadAssociations(argv[3], timestamps, rgb_paths, depth_paths) ||
       !LoadImu(argv[4], imu_samples))
    {
        cerr << "Failed to load associations or IMU data." << endl;
        return 1;
    }

    orb3_offline::Calibration calibration;
    string calibration_error;
    if(!orb3_offline::LoadCalibration(argv[1], 1.0, calibration,
                                     calibration_error))
    {
        cerr << "Failed to load calibration: " << calibration_error << endl;
        return 1;
    }

    size_t bias_samples = 0;
    const Eigen::Vector3d gyro_bias = orb3_offline::EstimateInitialGyroBias(
        imu_samples, timestamps.front(), 5.0, bias_samples);
    cout << fixed << setprecision(9)
         << "Initial gyro bias from " << bias_samples << " stationary samples: ["
         << gyro_bias.x() << ", " << gyro_bias.y() << ", " << gyro_bias.z()
         << "] rad/s" << endl;

    ofstream output(argv[5]);
    if(!output.is_open())
    {
        cerr << "Cannot create output CSV: " << argv[5] << endl;
        return 1;
    }
    output << "frame_previous,frame_current,t_previous,t_current,window_s,"
           << "source_t_previous,source_t_current,camera_offset_previous_s,"
           << "camera_offset_current_s,imu_samples,imu_first_dt_s,imu_last_dt_s,"
           << "imu_max_gap_s,features_previous,features_current,ratio_matches,"
           << "depth_matches,visual_inliers,inlier_ratio,occupied_cells,"
           << "reprojection_rmse_px,visual_valid,visual_rotation_deg,"
           << "imu_rotation_deg,residual_valid,residual_roll_deg,"
           << "residual_pitch_deg,residual_yaw_deg,rotation_residual_deg,"
           << "gyro_bias_x,gyro_bias_y,gyro_bias_z\n";
    output << fixed << setprecision(9);

    cv::Mat previous_rgb = cv::imread(
        string(argv[2]) + "/" + rgb_paths.front(), cv::IMREAD_UNCHANGED);
    cv::Mat previous_depth = cv::imread(
        string(argv[2]) + "/" + depth_paths.front(), cv::IMREAD_UNCHANGED);
    if(previous_rgb.empty() || previous_depth.empty())
    {
        cerr << "Cannot load the first RGB-D pair." << endl;
        return 1;
    }

    orb3_offline::VisualRotationEstimator estimator(calibration, 2000);
    vector<double> valid_residuals;
    for(size_t index = 1; index < timestamps.size(); ++index)
    {
        cv::Mat current_rgb = cv::imread(
            string(argv[2]) + "/" + rgb_paths[index], cv::IMREAD_UNCHANGED);
        cv::Mat current_depth = cv::imread(
            string(argv[2]) + "/" + depth_paths[index], cv::IMREAD_UNCHANGED);
        if(current_rgb.empty() || current_depth.empty())
        {
            cerr << "Cannot load frame " << index << endl;
            return 1;
        }

        const orb3_offline::VisualRotationResult visual = estimator.Estimate(
            previous_rgb, previous_depth, current_rgb);
        const orb3_offline::ImuIntegrationResult imu =
            orb3_offline::IntegrateGyroscope(
                imu_samples, timestamps[index - 1], timestamps[index], gyro_bias);
        const orb3_offline::RotationResidual residual =
            orb3_offline::CompareRotations(
                visual, imu, calibration.rotation_body_from_camera);
        if(residual.valid)
            valid_residuals.push_back(residual.angle_deg);

        const double source_previous =
            orb3_offline::InferImageSourceTimestamp(rgb_paths[index - 1]);
        const double source_current =
            orb3_offline::InferImageSourceTimestamp(rgb_paths[index]);
        output
            << (index - 1) << ',' << index << ',' << timestamps[index - 1] << ','
            << timestamps[index] << ',' << timestamps[index] - timestamps[index - 1]
            << ',' << source_previous << ',' << source_current << ','
            << timestamps[index - 1] - source_previous << ','
            << timestamps[index] - source_current << ',' << imu.sample_count << ','
            << imu.first_sample_dt << ',' << imu.last_sample_dt << ','
            << imu.maximum_gap << ',' << visual.previous_features << ','
            << visual.current_features << ',' << visual.ratio_matches << ','
            << visual.depth_matches << ',' << visual.inliers << ','
            << visual.inlier_ratio << ',' << visual.occupied_cells << ','
            << visual.reprojection_rmse_px << ',' << (visual.valid ? 1 : 0) << ','
            << visual.rotation_angle_deg << ',' << imu.rotation_angle_deg << ','
            << (residual.valid ? 1 : 0) << ',' << residual.roll_deg << ','
            << residual.pitch_deg << ',' << residual.yaw_deg << ','
            << residual.angle_deg << ',' << gyro_bias.x() << ',' << gyro_bias.y()
            << ',' << gyro_bias.z() << '\n';

        previous_rgb = current_rgb;
        previous_depth = current_depth;
        if(index % 500 == 0)
            cout << "Processed " << index << "/" << (timestamps.size() - 1)
                 << " frame intervals" << endl;
    }

    double rms = 0.0;
    for(double value : valid_residuals)
        rms += value * value;
    rms = valid_residuals.empty()
        ? numeric_limits<double>::quiet_NaN()
        : sqrt(rms / static_cast<double>(valid_residuals.size()));
    cout << "Valid visual-IMU intervals: " << valid_residuals.size() << '/'
         << (timestamps.size() - 1) << endl
         << "Rotation residual median: " << Percentile(valid_residuals, 0.5)
         << " deg" << endl
         << "Rotation residual RMS: " << rms << " deg" << endl
         << "Rotation residual p95: " << Percentile(valid_residuals, 0.95)
         << " deg" << endl
         << "Output: " << argv[5] << endl;
    return 0;
}
