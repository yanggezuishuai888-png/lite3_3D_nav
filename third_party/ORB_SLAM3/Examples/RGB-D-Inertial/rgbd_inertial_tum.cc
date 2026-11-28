/**
 * Offline RGB-D-Inertial runner for ORB-SLAM3.
 *
 * Dataset layout follows the TUM RGB-D associations format, with an additional
 * IMU text file containing:
 *   timestamp gx gy gz ax ay az
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <System.h>
#include <ImuTypes.h>
#include "offline_diagnostics.h"

using namespace std;

namespace
{

using orb3_offline::ImuSample;

const char* TrackingStateName(int state)
{
    switch(state)
    {
        case -1: return "SYSTEM_NOT_READY";
        case 0: return "NO_IMAGES_YET";
        case 1: return "NOT_INITIALIZED";
        case 2: return "OK";
        case 3: return "RECENTLY_LOST";
        case 4: return "LOST";
        case 5: return "OK_KLT";
        default: return "UNKNOWN";
    }
}

const char* SlamPhaseName(const ORB_SLAM3::System::DiagnosticSnapshot &snapshot)
{
    if(snapshot.inertial_ba2_done)
        return "FULLY_INITIALIZED";
    if(snapshot.inertial_ba1_done)
        return "VIBA1_DONE";
    if(snapshot.imu_initialized)
        return "IMU_INITIALIZED";
    if(snapshot.tracking_state == 0 || snapshot.tracking_state == 1)
        return "BOOTSTRAP";
    return "PREINERTIAL_TRACKING";
}

struct IntervalCoverage
{
    int count = 0;
    double first_dt = numeric_limits<double>::quiet_NaN();
    double last_dt = numeric_limits<double>::quiet_NaN();
    double maximum_gap = numeric_limits<double>::quiet_NaN();
};

struct IntervalImuStats
{
    double mean_accel_x = numeric_limits<double>::quiet_NaN();
    double mean_accel_y = numeric_limits<double>::quiet_NaN();
    double mean_accel_z = numeric_limits<double>::quiet_NaN();
    double mean_accel_norm = numeric_limits<double>::quiet_NaN();
    double min_accel_norm = numeric_limits<double>::quiet_NaN();
    double max_accel_norm = numeric_limits<double>::quiet_NaN();
    double mean_gyro_x = numeric_limits<double>::quiet_NaN();
    double mean_gyro_y = numeric_limits<double>::quiet_NaN();
    double mean_gyro_z = numeric_limits<double>::quiet_NaN();
    double mean_gyro_norm = numeric_limits<double>::quiet_NaN();
    double max_gyro_norm = numeric_limits<double>::quiet_NaN();
};

IntervalCoverage MeasureImuCoverage(const vector<ImuSample> &samples,
                                    double start_time,
                                    double end_time)
{
    IntervalCoverage coverage;
    auto first = lower_bound(
        samples.begin(), samples.end(), start_time,
        [](const ImuSample &sample, double time) {
            return sample.timestamp < time;
        });
    auto after_last = upper_bound(
        samples.begin(), samples.end(), end_time,
        [](double time, const ImuSample &sample) {
            return time < sample.timestamp;
        });
    coverage.count = static_cast<int>(distance(first, after_last));
    if(first != samples.end())
        coverage.first_dt = first->timestamp - start_time;
    if(after_last != samples.begin())
        coverage.last_dt = end_time - (after_last - 1)->timestamp;

    double maximum_gap = 0.0;
    auto begin_gap = first == samples.begin() ? first : first - 1;
    auto end_gap = after_last == samples.end() ? after_last : after_last + 1;
    for(auto iterator = begin_gap; iterator != end_gap; ++iterator)
    {
        auto next = iterator + 1;
        if(next == end_gap)
            break;
        maximum_gap = max(maximum_gap, next->timestamp - iterator->timestamp);
    }
    coverage.maximum_gap = maximum_gap;
    return coverage;
}

IntervalImuStats MeasureImuStats(const vector<ImuSample> &samples,
                                 double start_time,
                                 double end_time)
{
    IntervalImuStats stats;
    auto first = lower_bound(
        samples.begin(), samples.end(), start_time,
        [](const ImuSample &sample, double time) {
            return sample.timestamp < time;
        });
    auto after_last = upper_bound(
        samples.begin(), samples.end(), end_time,
        [](double time, const ImuSample &sample) {
            return time < sample.timestamp;
        });
    if(first == after_last)
        return stats;

    cv::Point3d accel_sum(0.0, 0.0, 0.0);
    cv::Point3d gyro_sum(0.0, 0.0, 0.0);
    double accel_norm_sum = 0.0;
    double gyro_norm_sum = 0.0;
    double min_accel_norm = numeric_limits<double>::infinity();
    double max_accel_norm = 0.0;
    double max_gyro_norm = 0.0;
    size_t count = 0;
    for(auto iterator = first; iterator != after_last; ++iterator)
    {
        const cv::Point3f &accel = iterator->acceleration;
        const cv::Point3f &gyro = iterator->gyro;
        const double accel_norm = cv::norm(accel);
        const double gyro_norm = cv::norm(gyro);
        accel_sum += cv::Point3d(accel.x, accel.y, accel.z);
        gyro_sum += cv::Point3d(gyro.x, gyro.y, gyro.z);
        accel_norm_sum += accel_norm;
        gyro_norm_sum += gyro_norm;
        min_accel_norm = min(min_accel_norm, accel_norm);
        max_accel_norm = max(max_accel_norm, accel_norm);
        max_gyro_norm = max(max_gyro_norm, gyro_norm);
        ++count;
    }

    const double inverse_count = 1.0 / static_cast<double>(count);
    stats.mean_accel_x = accel_sum.x * inverse_count;
    stats.mean_accel_y = accel_sum.y * inverse_count;
    stats.mean_accel_z = accel_sum.z * inverse_count;
    stats.mean_accel_norm = accel_norm_sum * inverse_count;
    stats.min_accel_norm = min_accel_norm;
    stats.max_accel_norm = max_accel_norm;
    stats.mean_gyro_x = gyro_sum.x * inverse_count;
    stats.mean_gyro_y = gyro_sum.y * inverse_count;
    stats.mean_gyro_z = gyro_sum.z * inverse_count;
    stats.mean_gyro_norm = gyro_norm_sum * inverse_count;
    stats.max_gyro_norm = max_gyro_norm;
    return stats;
}

bool ParseBool(const string &value)
{
    return value != "false" && value != "0" && value != "no";
}

bool LoadImages(const string &association_path,
                vector<string> &rgb_paths,
                vector<string> &depth_paths,
                vector<double> &timestamps)
{
    ifstream input(association_path);
    if(!input.is_open())
    {
        cerr << "Cannot open association file: " << association_path << endl;
        return false;
    }

    string line;
    while(getline(input, line))
    {
        if(line.empty() || line[0] == '#')
            continue;

        stringstream stream(line);
        double rgb_time = 0.0;
        double depth_time = 0.0;
        string rgb_path;
        string depth_path;
        if(!(stream >> rgb_time >> rgb_path >> depth_time >> depth_path))
        {
            cerr << "Malformed association line: " << line << endl;
            return false;
        }
        if(abs(rgb_time - depth_time) > 1e-4)
        {
            cerr << "RGB/depth timestamps differ by more than 0.1 ms: "
                 << setprecision(17) << rgb_time << " vs " << depth_time << endl;
            return false;
        }

        timestamps.push_back(rgb_time);
        rgb_paths.push_back(rgb_path);
        depth_paths.push_back(depth_path);
    }
    return !timestamps.empty();
}

bool LoadImu(const string &imu_path, vector<ImuSample> &samples)
{
    ifstream input(imu_path);
    if(!input.is_open())
    {
        cerr << "Cannot open IMU file: " << imu_path << endl;
        return false;
    }

    string line;
    while(getline(input, line))
    {
        if(line.empty() || line[0] == '#')
            continue;

        replace(line.begin(), line.end(), ',', ' ');
        stringstream stream(line);
        ImuSample sample;
        if(!(stream >> sample.timestamp
                    >> sample.gyro.x >> sample.gyro.y >> sample.gyro.z
                    >> sample.acceleration.x >> sample.acceleration.y
                    >> sample.acceleration.z))
        {
            cerr << "Malformed IMU line: " << line << endl;
            return false;
        }
        if(!samples.empty() && sample.timestamp <= samples.back().timestamp)
        {
            cerr << "IMU timestamps are not strictly increasing at "
                 << setprecision(17) << sample.timestamp << endl;
            return false;
        }
        samples.push_back(sample);
    }
    return !samples.empty();
}

double MaximumGap(const vector<double> &timestamps)
{
    double maximum = 0.0;
    for(size_t index = 1; index < timestamps.size(); ++index)
        maximum = max(maximum, timestamps[index] - timestamps[index - 1]);
    return maximum;
}

double MaximumImuGap(const vector<ImuSample> &samples,
                     double start_time,
                     double end_time)
{
    double maximum = 0.0;
    for(size_t index = 1; index < samples.size(); ++index)
    {
        if(samples[index].timestamp < start_time)
            continue;
        if(samples[index - 1].timestamp > end_time)
            break;
        maximum = max(
            maximum,
            samples[index].timestamp - samples[index - 1].timestamp);
    }
    return maximum;
}

}  // namespace

int main(int argc, char **argv)
{
    if(argc < 6 || argc > 9)
    {
        cerr << endl
             << "Usage: ./rgbd_inertial_tum path_to_vocabulary path_to_settings "
             << "path_to_sequence path_to_association path_to_imu "
             << "[viewer: true|false] [realtime: true|false] "
             << "[max_imu_gap_s: 0.05]" << endl;
        return 1;
    }

    const bool use_viewer = argc >= 7 ? ParseBool(argv[6]) : true;
    const bool realtime = argc >= 8 ? ParseBool(argv[7]) : true;
    const double max_allowed_imu_gap = argc >= 9 ? stod(argv[8]) : 0.05;

    vector<string> rgb_paths;
    vector<string> depth_paths;
    vector<double> image_timestamps;
    vector<ImuSample> imu_samples;

    if(!LoadImages(argv[4], rgb_paths, depth_paths, image_timestamps))
    {
        cerr << "No valid RGB-D associations loaded." << endl;
        return 1;
    }
    if(!LoadImu(argv[5], imu_samples))
    {
        cerr << "No valid IMU samples loaded." << endl;
        return 1;
    }

    if(!is_sorted(image_timestamps.begin(), image_timestamps.end()) ||
       adjacent_find(image_timestamps.begin(), image_timestamps.end(),
                     greater_equal<double>()) != image_timestamps.end())
    {
        cerr << "Image timestamps are not strictly increasing." << endl;
        return 1;
    }
    if(imu_samples.front().timestamp > image_timestamps.front())
    {
        cerr << "IMU begins after the first corrected camera timestamp." << endl;
        return 1;
    }
    if(imu_samples.back().timestamp < image_timestamps.back())
    {
        cerr << "IMU ends before the last corrected camera timestamp." << endl;
        return 1;
    }

    const double max_image_gap = MaximumGap(image_timestamps);
    const double max_imu_gap = MaximumImuGap(
        imu_samples, image_timestamps.front(), image_timestamps.back());

    cout << fixed << setprecision(6)
         << "RGB-D frames: " << image_timestamps.size() << endl
         << "IMU samples: " << imu_samples.size() << endl
         << "Maximum RGB-D gap: " << max_image_gap << " s" << endl
         << "Maximum IMU gap: " << max_imu_gap << " s" << endl;

    if(max_image_gap > 0.5)
        cerr << "WARNING: RGB-D gap exceeds 0.5 s; tracking may be lost." << endl;
    if(max_allowed_imu_gap > 0.0 && max_imu_gap > max_allowed_imu_gap)
    {
        cerr << "Refusing to preintegrate across an IMU gap larger than "
             << max_allowed_imu_gap << " s. Fix/trim the dataset, or pass 0 "
             << "as max_imu_gap_s only for a diagnostic run." << endl;
        return 2;
    }

    ORB_SLAM3::System slam(
        argv[1], argv[2], ORB_SLAM3::System::IMU_RGBD, use_viewer);
    const float image_scale = slam.GetImageScale();

    ofstream frame_metrics("frame_metrics.csv");
    ofstream events("events.csv");
    if(!frame_metrics.is_open() || !events.is_open())
    {
        cerr << "Cannot create frame_metrics.csv or events.csv in the current directory." << endl;
        slam.Shutdown();
        return 1;
    }
    frame_metrics
        << "frame_id,camera_timestamp,source_camera_timestamp,camera_time_offset_s,"
        << "camera_dt_s,imu_count,imu_first_dt_s,imu_last_dt_s,imu_max_gap_s,"
        << "tracking_time_ms,frame_deadline_missed,tracking_state,slam_phase,map_id,map_count,"
        << "keyframes_in_map,map_points_in_map,imu_initialized,viba1_done,viba2_done,"
        << "tracked_keypoints,tracked_map_points,matches_inliers,"
        << "body_pose_valid,body_x,body_y,body_z,body_qx,body_qy,body_qz,body_qw,"
        << "velocity_valid,velocity_x,velocity_y,velocity_z,"
        << "gyro_bias_x,gyro_bias_y,gyro_bias_z,accel_bias_x,accel_bias_y,accel_bias_z,"
        << "preintegration_valid,preintegration_dt,delta_velocity_x,delta_velocity_y,"
        << "delta_velocity_z,delta_position_x,delta_position_y,delta_position_z,"
        << "inertial_scale,gravity_x,gravity_y,gravity_z,"
        << "mean_accel_x,mean_accel_y,mean_accel_z,mean_accel_norm,min_accel_norm,"
        << "max_accel_norm,mean_gyro_x,mean_gyro_y,mean_gyro_z,mean_gyro_norm,"
        << "max_gyro_norm\n";
    events << "timestamp,frame_id,event,map_id,details\n";
    frame_metrics << fixed << setprecision(9);
    events << fixed << setprecision(9);

    auto first_after_initial_frame = upper_bound(
        imu_samples.begin(), imu_samples.end(), image_timestamps.front(),
        [](double timestamp, const ImuSample &sample) {
            return timestamp < sample.timestamp;
        });
    if(first_after_initial_frame == imu_samples.begin())
    {
        cerr << "No IMU sample at or before the first image." << endl;
        slam.Shutdown();
        return 1;
    }
    size_t imu_index = static_cast<size_t>(
        distance(imu_samples.begin(), first_after_initial_frame) - 1);

    vector<float> tracking_times;
    tracking_times.reserve(image_timestamps.size());
    size_t skipped_images_without_imu = 0;
    vector<ORB_SLAM3::IMU::Point> frame_imu;
    cv::Mat rgb;
    cv::Mat depth;
    ORB_SLAM3::System::DiagnosticSnapshot previous_snapshot;
    bool have_previous_snapshot = false;

    cout << "Start processing RGB-D-Inertial sequence ..." << endl;
    for(size_t image_index = 0; image_index < image_timestamps.size(); ++image_index)
    {
        rgb = cv::imread(string(argv[3]) + "/" + rgb_paths[image_index],
                         cv::IMREAD_UNCHANGED);
        depth = cv::imread(string(argv[3]) + "/" + depth_paths[image_index],
                           cv::IMREAD_UNCHANGED);
        if(rgb.empty() || depth.empty())
        {
            cerr << "Failed to load RGB-D pair: " << rgb_paths[image_index]
                 << " / " << depth_paths[image_index] << endl;
            slam.Shutdown();
            return 1;
        }

        if(image_scale != 1.0f)
        {
            const cv::Size scaled_size(
                static_cast<int>(rgb.cols * image_scale),
                static_cast<int>(rgb.rows * image_scale));
            cv::resize(rgb, rgb, scaled_size);
            cv::resize(depth, depth, scaled_size, 0.0, 0.0, cv::INTER_NEAREST);
        }

        frame_imu.clear();
        if(image_index > 0)
        {
            while(imu_index < imu_samples.size() &&
                  imu_samples[imu_index].timestamp <= image_timestamps[image_index])
            {
                const ImuSample &sample = imu_samples[imu_index];
                frame_imu.emplace_back(
                    sample.acceleration, sample.gyro, sample.timestamp);
                ++imu_index;
            }
            if(frame_imu.size() < 2)
            {
                cerr << "Insufficient IMU measurements for frame " << image_index
                     << " at " << setprecision(17)
                     << image_timestamps[image_index] << endl;
                if(max_allowed_imu_gap == 0.0)
                {
                    cerr << "Diagnostic mode: skipping this RGB-D frame and "
                         << "waiting for IMU to resume." << endl;
                    ++skipped_images_without_imu;
                    continue;
                }
                slam.Shutdown();
                return 1;
            }
        }

        const auto start = chrono::steady_clock::now();
        slam.TrackRGBD(
            rgb, depth, image_timestamps[image_index], frame_imu,
            rgb_paths[image_index]);
        const auto finish = chrono::steady_clock::now();
        const double tracking_time =
            chrono::duration_cast<chrono::duration<double>>(finish - start).count();
        tracking_times.push_back(static_cast<float>(tracking_time));

        const double camera_dt = image_index == 0
            ? numeric_limits<double>::quiet_NaN()
            : image_timestamps[image_index] - image_timestamps[image_index - 1];
        const IntervalCoverage coverage = image_index == 0
            ? IntervalCoverage{}
            : MeasureImuCoverage(imu_samples, image_timestamps[image_index - 1],
                                 image_timestamps[image_index]);
        const IntervalImuStats imu_stats = image_index == 0
            ? IntervalImuStats{}
            : MeasureImuStats(imu_samples, image_timestamps[image_index - 1],
                              image_timestamps[image_index]);
        const double source_timestamp =
            orb3_offline::InferImageSourceTimestamp(rgb_paths[image_index]);
        const double camera_offset = std::isfinite(source_timestamp)
            ? image_timestamps[image_index] - source_timestamp
            : numeric_limits<double>::quiet_NaN();
        const ORB_SLAM3::System::DiagnosticSnapshot snapshot =
            slam.GetDiagnosticSnapshot();
        const bool missed_deadline = image_index > 0 && tracking_time > camera_dt;

        frame_metrics
            << image_index << ',' << image_timestamps[image_index] << ','
            << source_timestamp << ',' << camera_offset << ',' << camera_dt << ','
            << coverage.count << ',' << coverage.first_dt << ',' << coverage.last_dt << ','
            << coverage.maximum_gap << ',' << tracking_time * 1000.0 << ','
            << (missed_deadline ? 1 : 0) << ','
            << TrackingStateName(snapshot.tracking_state) << ',' << SlamPhaseName(snapshot)
            << ',' << snapshot.map_id << ','
            << snapshot.map_count << ',' << snapshot.keyframes_in_map << ','
            << snapshot.map_points_in_map << ',' << (snapshot.imu_initialized ? 1 : 0)
            << ',' << (snapshot.inertial_ba1_done ? 1 : 0) << ','
            << (snapshot.inertial_ba2_done ? 1 : 0) << ','
            << snapshot.tracked_keypoints << ',' << snapshot.tracked_map_points << ','
            << snapshot.matches_inliers << ','
            << (snapshot.body_pose_valid ? 1 : 0) << ','
            << snapshot.body_position_x << ',' << snapshot.body_position_y << ','
            << snapshot.body_position_z << ',' << snapshot.body_quaternion_x << ','
            << snapshot.body_quaternion_y << ',' << snapshot.body_quaternion_z << ','
            << snapshot.body_quaternion_w << ','
            << (snapshot.body_velocity_valid ? 1 : 0) << ','
            << snapshot.velocity_x << ',' << snapshot.velocity_y << ','
            << snapshot.velocity_z << ',' << snapshot.gyro_bias_x << ','
            << snapshot.gyro_bias_y << ',' << snapshot.gyro_bias_z << ','
            << snapshot.accel_bias_x << ',' << snapshot.accel_bias_y << ','
            << snapshot.accel_bias_z << ','
            << (snapshot.preintegration_valid ? 1 : 0) << ','
            << snapshot.preintegration_dt << ',' << snapshot.delta_velocity_x << ','
            << snapshot.delta_velocity_y << ',' << snapshot.delta_velocity_z << ','
            << snapshot.delta_position_x << ',' << snapshot.delta_position_y << ','
            << snapshot.delta_position_z << ',' << snapshot.inertial_scale << ','
            << snapshot.gravity_x << ',' << snapshot.gravity_y << ','
            << snapshot.gravity_z << ',' << imu_stats.mean_accel_x << ','
            << imu_stats.mean_accel_y << ',' << imu_stats.mean_accel_z << ','
            << imu_stats.mean_accel_norm << ',' << imu_stats.min_accel_norm << ','
            << imu_stats.max_accel_norm << ',' << imu_stats.mean_gyro_x << ','
            << imu_stats.mean_gyro_y << ',' << imu_stats.mean_gyro_z << ','
            << imu_stats.mean_gyro_norm << ',' << imu_stats.max_gyro_norm << '\n';

        if(!have_previous_snapshot)
        {
            events << image_timestamps[image_index] << ',' << image_index
                   << ",RUN_START," << snapshot.map_id
                   << ",state=" << TrackingStateName(snapshot.tracking_state) << '\n';
        }
        else
        {
            if(snapshot.tracking_state != previous_snapshot.tracking_state)
                events << image_timestamps[image_index] << ',' << image_index
                       << ",TRACKING_STATE_CHANGED," << snapshot.map_id << ','
                       << TrackingStateName(previous_snapshot.tracking_state) << "->"
                       << TrackingStateName(snapshot.tracking_state) << '\n';
            if(string(SlamPhaseName(snapshot)) != string(SlamPhaseName(previous_snapshot)))
                events << image_timestamps[image_index] << ',' << image_index
                       << ",SLAM_PHASE_CHANGED," << snapshot.map_id << ','
                       << SlamPhaseName(previous_snapshot) << "->"
                       << SlamPhaseName(snapshot) << '\n';
            if(snapshot.map_id != previous_snapshot.map_id)
                events << image_timestamps[image_index] << ',' << image_index
                       << ",ACTIVE_MAP_CHANGED," << snapshot.map_id << ','
                       << previous_snapshot.map_id << "->" << snapshot.map_id << '\n';
            if(snapshot.map_count > previous_snapshot.map_count)
                events << image_timestamps[image_index] << ',' << image_index
                       << ",MAP_CREATED," << snapshot.map_id << ",map_count="
                       << snapshot.map_count << '\n';
            if(!previous_snapshot.imu_initialized && snapshot.imu_initialized)
                events << image_timestamps[image_index] << ',' << image_index
                       << ",IMU_INITIALIZED," << snapshot.map_id << ",\n";
            if(!previous_snapshot.inertial_ba1_done && snapshot.inertial_ba1_done)
                events << image_timestamps[image_index] << ',' << image_index
                       << ",VIBA1_DONE," << snapshot.map_id << ",\n";
            if(!previous_snapshot.inertial_ba2_done && snapshot.inertial_ba2_done)
                events << image_timestamps[image_index] << ',' << image_index
                       << ",VIBA2_DONE," << snapshot.map_id << ",\n";
            if(snapshot.map_id == previous_snapshot.map_id &&
               previous_snapshot.keyframes_in_map > snapshot.keyframes_in_map + 2)
                events << image_timestamps[image_index] << ',' << image_index
                       << ",KEYFRAME_COUNT_DECREASED," << snapshot.map_id << ','
                       << "keyframes=" << previous_snapshot.keyframes_in_map << "->"
                       << snapshot.keyframes_in_map << '\n';
        }
        previous_snapshot = snapshot;
        have_previous_snapshot = true;
        if(image_index % 100 == 0)
        {
            frame_metrics.flush();
            events.flush();
        }

        if(realtime && image_index + 1 < image_timestamps.size())
        {
            const double frame_period =
                image_timestamps[image_index + 1] - image_timestamps[image_index];
            if(tracking_time < frame_period)
                usleep(static_cast<useconds_t>(
                    (frame_period - tracking_time) * 1e6));
        }
    }

    frame_metrics.flush();
    events.flush();
    slam.Shutdown();
    if(have_previous_snapshot)
        events << image_timestamps.back() << ',' << (image_timestamps.size() - 1)
               << ",RUN_END," << previous_snapshot.map_id << ",\n";
    events.flush();

    if(tracking_times.empty())
    {
        cerr << "No RGB-D frame was processed." << endl;
        return 1;
    }

    sort(tracking_times.begin(), tracking_times.end());
    double total_tracking_time = 0.0;
    for(float time : tracking_times)
        total_tracking_time += time;
    cout << "Median tracking time: "
         << tracking_times[tracking_times.size() / 2] << " s" << endl
         << "Mean tracking time: "
         << total_tracking_time / tracking_times.size() << " s" << endl
         << "Processed RGB-D frames: " << tracking_times.size() << endl
         << "Skipped RGB-D frames without sufficient IMU: "
         << skipped_images_without_imu << endl;

    // EuRoC output reports the IMU/body trajectory for inertial sensors.
    slam.SaveTrajectoryEuRoC("CameraTrajectory.txt");
    slam.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    return 0;
}
