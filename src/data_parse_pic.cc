#include <dirent.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sys/types.h>
#include "unistd.h"
#include <chrono>
#include <fstream>
#include <map>
#include <string>
#include <thread>
#include <vector>
#include "fstream"
//
#include <compressed_image_transport/compressed_publisher.h>
#include <cv_bridge/cv_bridge.h>
#include <optional>
#include "Eigen/Eigen"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "image_transport/image_transport.h"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Imu.h"
//
DEFINE_string(file_dir, "", "read file dir");
DEFINE_string(out_bag_name, "", "full out_bag_name with dir ex: /home/a.bag");
//
namespace {
//
constexpr char kImagDataDirName[] = "image";

constexpr char kImagTopic0[] = "/usb_cam_1/image_raw/compressed";
constexpr char kImagTopic1[] = "/usb_cam_2/image_raw/compressed";
constexpr char kImuTopic[] = "/imu";

//
//

std::set<std::string> ReadFileFromDir(const std::string& path) {
  std::set<std::string> fp_set;
  DIR* dir = opendir(path.c_str());
  CHECK(dir);
  struct dirent* entry = nullptr;
  while ((entry = readdir(dir)) != nullptr) {
    if (std::string(entry->d_name) == ".") continue;
    if (std::string(entry->d_name) == "..") continue;
    std::string pic_name = path + std::string(entry->d_name);
    fp_set.emplace(pic_name);
  }
  closedir(dir);
  // //
  LOG(INFO) << "dir path has file size :" << fp_set.size();
  return fp_set;
  //
}

struct ImuData {
  uint64_t time;
  Eigen::Vector3d linear_acceleration;
  Eigen::Vector3d angular_velocity;

  static std::map<uint64_t, ImuData> Parse(const std::string& dir_file);
};

//
std::optional<std::pair<uint64_t, uint64_t>> init_imu_time;

std::istringstream& operator>>(std::istringstream& ifs, ImuData& imu_data) {
  uint64_t time;
  ifs >> time;
  char unuse_char;
  ifs >> unuse_char;
  imu_data.time = static_cast<uint64_t>(time);
  LOG(INFO) << "Imu time: " << imu_data.time;
  static uint64_t init_imu_data_time = 0;
  if (imu_data.time < init_imu_data_time) {
    LOG(WARNING) << "Time write unolder..";
  }
  init_imu_data_time = imu_data.time;
  //
  
  // ifs >> imu_data.linear_acceleration.x() >> imu_data.linear_acceleration.y() >>
  //     imu_data.linear_acceleration.z() >> imu_data.angular_velocity.x() >>
  //     imu_data.angular_velocity.y() >> imu_data.angular_velocity.z();
  ifs >> imu_data.linear_acceleration.x() >> unuse_char >>
      imu_data.linear_acceleration.y() >> unuse_char >>
      imu_data.linear_acceleration.z() >> unuse_char >>
      imu_data.angular_velocity.x() >> unuse_char >>
      imu_data.angular_velocity.y() >> unuse_char >>
      imu_data.angular_velocity.z() ;

 
  return ifs;
}
//

template <typename TypeName>
std::vector<TypeName> ReadFile(const std::string& txt) {
  std::ifstream file;
  file.open(txt);
  CHECK(file.good())<<txt;
  std::string line;
  std::vector<TypeName> result;
  while (std::getline(file, line)) {
    std::istringstream iss(line);
    TypeName data;
    iss >> data;
    // CHECK(file.good());
    result.push_back(data);
  }
  file.close();
  LOG(INFO) << "done";
  return result;
}
//
std::map<uint64_t, ImuData> ImuData::Parse(const std::string& file) {
  const auto imu_data = ReadFile<ImuData>(file);
  CHECK(!imu_data.empty());
  std::map<uint64_t, ImuData> result;
  for (const auto imu : imu_data) {
    LOG_IF(ERROR, !result.emplace(imu.time, imu).second)
        << "Imu time duplicate..";

  }
  return result;
}
//
sensor_msgs::ImuPtr ToRosImu(const ImuData& imu_data) {
  sensor_msgs::ImuPtr result(new sensor_msgs::Imu());
  result->header.frame_id = "imu";
  ros::Time t;
  LOG(INFO)<<imu_data.time;
  t.fromNSec(imu_data.time);
  result->header.stamp = t;
  result->angular_velocity.x = imu_data.angular_velocity.x();
  result->angular_velocity.y = imu_data.angular_velocity.y();
  result->angular_velocity.z = imu_data.angular_velocity.z();
  result->linear_acceleration.x = imu_data.linear_acceleration.x();
  result->linear_acceleration.y = imu_data.linear_acceleration.y();
  result->linear_acceleration.z = imu_data.linear_acceleration.z();
  return result;
}
//
//

//
//

//
//
//
//
uint64_t GetTimeFromName(const std::string& name) {
  CHECK(!name.empty());
  auto it = name.find_last_of('/');
  std::string outdir = name.substr(0, it + 1);
  const std::string file_name =
      name.substr(it + 1, name.size() - outdir.size());
  auto it1 = file_name.find_last_of('.');
  LOG(INFO)<<std::stol(file_name.substr(0, it1));
  return std::stol(file_name.substr(0, it1));
}
//

constexpr int yuv_width =640;
constexpr int yuv_height =480;

cv::Mat YuvToMat(const std::string &file_name)
{
  std::ifstream file(file_name, std::ios::binary);
  if (!file.is_open()) {
    LOG(FATAL) << "无法打开文件";
  }
  // 读取文件内容到一个字节向量（或者根据您的需求选择适当的数据结构）
  std::vector<char> data;
  while (!file.eof()) {
    uint8_t byte;
    file.read(reinterpret_cast<char*>(&byte), sizeof(byte));
    if (!file.eof()) {
      data.push_back(byte);
    }
  }
  file.close();

  cv::Mat yuvmat(cv::Size(yuv_width,yuv_height), CV_8UC1, (void*)data.data());
  // cv::Mat rgbmat(cv::Size(dest->width, dest->height), CV_8UC3);
  // cv::cvtColor(yuvmat, rgbmat, CV_YUV2BGR_NV12);


  // auto yuv_image = cv::imread(file, cv::IMREAD_UNCHANGED);
  // cv::Mat image;
  // cv::cvtColor(yuv_image,image, cv::COLOR_GRAY2RGB);
  // cv::imshow("image",yuvmat);
  // cv::waitKey(0);
  return yuvmat.clone();
}
//
//
struct ImageData {
  uint64_t time;
  cv::Mat images;

  static std::map<uint64_t, ImageData> Parse(const std::string& dir_file) {
    const auto image_files_name = ReadFileFromDir(dir_file);
    CHECK(!image_files_name.empty()) << "Need Image file in dir..";
    std::map<uint64_t, ImageData> result;
    //
    for (const auto& file : image_files_name) {
      LOG(INFO) << "Read Image: " << file;
      LOG_IF(ERROR,
             !result
                 .emplace(GetTimeFromName(file),
                          ImageData{GetTimeFromName(file),
                                    cv::imread(file,cv::IMREAD_GRAYSCALE)})
                 .second)
          << "Image time duplicate..";
    }
    return result;
  }
};
//
sensor_msgs::CompressedImage::Ptr ToRosCompressedImage(const ImageData& image) {
  //
  sensor_msgs::CompressedImage::Ptr image_up(
      new sensor_msgs::CompressedImage());
  image_up->format = "mono8";
  image_up->format += "; jpeg compressed ";
  //
  std_msgs::Header header;
  header.frame_id = "camera";
  ros::Time t;
  t.fromNSec(image.time);
  header.stamp = t;
  std::vector<uint8_t> encodeing;
  cv::imencode(".jpg", image.images, image_up->data);
  // cv::imshow("11", image.images);
  // cv::waitKey(0);
  image_up->header = header;
  return image_up;
}
//
}  // namespace

int sample =0;
void WriteImuData(rosbag::Bag& out_bag, uint64_t time,
                  std::map<uint64_t, ImuData>& imu_datas) {
  auto it = imu_datas.upper_bound(time);
  for (auto itor = imu_datas.begin(); itor != it; ++itor) {
    // if (sample++ < 3) continue;
    // sample = 0;
    auto imu_data = ToRosImu(itor->second);
    out_bag.write(kImuTopic, imu_data->header.stamp, imu_data);
    LOG(INFO) << "   Imu time: " << imu_data->header.stamp.toNSec();
  }
  imu_datas.erase(imu_datas.begin(), it);
}
//
void Run(rosbag::Bag& out_bag, std::map<uint64_t, ImuData>& imu_datas,
         std::map<uint64_t, ImageData> images_datas) {
  LOG(INFO) << "Run start..";
  LOG(INFO) << "Write init befor image time imu data lenth: "
            << std::distance(
                   imu_datas.begin(),
                   imu_datas.upper_bound(images_datas.begin()->first));
  //
  WriteImuData(out_bag, images_datas.begin()->first, imu_datas);
  for (const auto& image : images_datas) {
    //
    LOG(INFO) << "image time : " << image.second.time
              << " start imu t: " << imu_datas.begin()->first
              << ", end imu t: " << imu_datas.upper_bound(image.first)->first
              << " size:"
              << std::distance(imu_datas.begin(),
                               imu_datas.upper_bound(image.first));  

    WriteImuData(out_bag, image.second.time, imu_datas);

    const auto image_data = ToRosCompressedImage(image.second);
    out_bag.write(kImagTopic0, image_data->header.stamp, image_data);
    out_bag.write(kImagTopic1, image_data->header.stamp, image_data);
  }
  if (!imu_datas.empty()) {
    WriteImuData(out_bag, UINT64_MAX, imu_datas);
  }
  CHECK(imu_datas.empty());
}
//
int main(int argc, char* argv[]) {
  //
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  //
    //
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  if (FLAGS_file_dir.empty() || FLAGS_out_bag_name.empty()) {
    google::ShowUsageWithFlagsRestrict(argv[0],
                                       "File dir empty or out bag empty");
    return EXIT_FAILURE;
  }
  //
  LOG(INFO) << "Sensor data dir : " << FLAGS_file_dir;
  LOG(INFO) << "Out bag file :" << FLAGS_out_bag_name;
  //
  ros::init(argc, argv, "write_tf_to_rosbag");
  ros::start();
  rosbag::Bag out_bag;
  try {
    out_bag.open(FLAGS_out_bag_name, rosbag::bagmode::Write);
  } catch (...) {
    LOG(FATAL) << "open :" << FLAGS_out_bag_name << "faied.";
    return EXIT_FAILURE;
  }
  LOG(INFO) << "Parse image dir: " << FLAGS_file_dir + "/" + kImagDataDirName+"/";
  LOG(INFO) << "Parse imu dir: " << FLAGS_file_dir + "/"+"imu.txt";
  auto image_datas = ImageData::Parse(FLAGS_file_dir + "/" + kImagDataDirName+"/");
  auto imu_datas = ImuData::Parse(FLAGS_file_dir + "/"+"imu.txt");
  LOG(INFO) << "Start write sensor data to " << FLAGS_out_bag_name;
  Run(out_bag, imu_datas, image_datas);
  LOG(INFO)<<"Write done..";
  out_bag.close();
  // infile.close();

  return EXIT_SUCCESS;
}