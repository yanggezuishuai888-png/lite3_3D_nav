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

using namespace std;

namespace
{

struct ImuSample
{
    double timestamp;
    cv::Point3f gyro;
    cv::Point3f acceleration;
};

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

        if(realtime && image_index + 1 < image_timestamps.size())
        {
            const double frame_period =
                image_timestamps[image_index + 1] - image_timestamps[image_index];
            if(tracking_time < frame_period)
                usleep(static_cast<useconds_t>(
                    (frame_period - tracking_time) * 1e6));
        }
    }

    slam.Shutdown();

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
