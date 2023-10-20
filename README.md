####
build ---
cd /home/user/
mkdir -p catkin_ws/src  && cd  catkin_ws/src
git clone  http://192.168.1.132:8080/spatialnavigation/metaboundsviogroup/data_parse
cd ..
catkin_make -DCATKIN_WHITELIST_PACKAGES="data_parse"


####
新建一个/home/user/data/image文件夹，把照片集序列放入到image文件夹里面
把imu_data.txt 放入/home/user/data/
解析运行命令
roslaunch data_parse data_parse.launch file_dir:=/home/user/data/  out_bag_name:=/home/user/data/data.bag
#####roslaunch data_parse data_parse.launch file_dir:=/home/lyp/project/dataset/slam_test/20230828/000/data  out_bag_name:=/home/lyp/project/dataset/slam_test/20230828/000/data/data.bag

####
用bag包生成测试数据运行
roslaunch data_parse rosbag_to_data_parse_test.launch   bag_file:=/home/lyp/project/dataset/slam_test/20230828/000/2023-08-28-14-39-56.bag   output_dir:=/home/lyp/project/dataset/slam_test/20230828/000