#include <dirent.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sys/types.h>

#include <chrono>
#include <fstream>
#include <map>
#include <string>
#include <thread>
#include <vector>
//
#include <compressed_image_transport/compressed_publisher.h>
#include <cv_bridge/cv_bridge.h>

#include "Eigen/Eigen"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "image_transport/image_transport.h"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Imu.h"
#include <sys/stat.h> 
//
//
//

DEFINE_string(bag_file, "", "read bag_file ");
DEFINE_string(output_dir, "", "output_dir ");
//
std::set<std::string> GetTopics(rosbag::View& view) {
  std::set<std::string> topics;
  for (const auto* connection : view.getConnections()) {
    topics.insert(connection->topic);
    LOG(INFO) << "input_bag has topic:\n " << connection->topic << "\n";
  }
  return topics;
}

bool WriteOutPutImage(std::string outdir, const rosbag::MessageInstance& msg) {
  cv_bridge::CvImagePtr image_ptr;
  if (msg.isType<sensor_msgs::CompressedImage>()) {
    image_ptr =
        cv_bridge::toCvCopy(msg.instantiate<sensor_msgs::CompressedImage>());
    cv::Mat image = image_ptr->image;
    std::string topic2dir = msg.getTopic();
    for (auto& c : topic2dir) {
      if (c == '/') {
        c = '_';
      }
    }
    outdir += "/";
    outdir += topic2dir.substr(1, topic2dir.size() - 1);  // msg.getTopic();
    outdir += "/";
    mkdir(outdir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    std::string outname;
    if (topic2dir.find('1') != std::string::npos) {
      outname =
          outdir + "1_" + std::to_string(image_ptr->header.stamp.toNSec());
    }
    if (topic2dir.find('2') != std::string::npos) {
      outname =
          outdir + "2_" + std::to_string(image_ptr->header.stamp.toNSec());
    }
    outname += ".png";
    // outname += image_ptr->encoding;
    LOG(INFO) << outname;
    cv::imwrite(outname, image);
  }
}
//

//
bool WriteOutPutImu(std::fstream& file, const rosbag::MessageInstance& msg) {
  if (msg.isType<sensor_msgs::Imu>()) {
    auto imu_msg = msg.instantiate<sensor_msgs::Imu>();
    file << imu_msg->header.stamp.toNSec() << " " << imu_msg->linear_acceleration.x
         << " " << imu_msg->linear_acceleration.y << " "
         << imu_msg->linear_acceleration.z << " " << imu_msg->angular_velocity.x
         << " " << imu_msg->angular_velocity.y << " "
         << imu_msg->angular_velocity.z << "0.0"
         << " "
         << "0.0"
         << " "
         << "0.0" << std::endl;
  }
  return true;
}

void run(const rosbag::Bag& input_bag, const std::string& output_dir) {
  LOG(INFO) << "run start";
  rosbag::View view(input_bag);
  rosbag::View::const_iterator view_iterator = view.begin();
  std::fstream file(output_dir + "/imu_data.txt",
                    std::ios_base::out /*| std::ios_base::app*/);
  for (auto view_iterator = view.begin(); view_iterator != view.end();
       view_iterator++) {
    rosbag::MessageInstance msg = *view_iterator;
    WriteOutPutImu(file, msg);
    WriteOutPutImage(output_dir, msg);
  }
  file.close();
}

int main(int argc,char*argv[])
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  //
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  if (FLAGS_bag_file.empty() || FLAGS_output_dir.empty()) {
    google::ShowUsageWithFlagsRestrict(argv[0], "bage dir empty ");
    return EXIT_FAILURE;
  }
  LOG(INFO) << "Bag file : " << FLAGS_bag_file;
  LOG(INFO) << "output  dir: " <<FLAGS_output_dir;

  ros::init(argc, argv, "rosbag_to_data_parse_test");
  ros::start();
  rosbag::Bag input_bag;
  try {
    input_bag.open(FLAGS_bag_file, rosbag::bagmode::Read);
  } catch (...) {
    LOG(FATAL) << "open: " << FLAGS_bag_file << "faied.";
  }

  std::string output_dir(argv[2]);
  run(input_bag, FLAGS_output_dir);
}
