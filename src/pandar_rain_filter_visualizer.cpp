#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <boost/lambda/bind.hpp>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pandar_pointcloud/point_types.hpp>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include<sstream>
#include <algorithm>
#include "opencv2/highgui/highgui.hpp"


typedef pcl::PointXYZI PointT;
typedef pandar_pointcloud::PointXYZIR PointXYZIR;

ros::Publisher original_pcl_pub;
ros::Publisher noise_pcl_pub;
ros::Publisher non_noise_pcl_pub;

bool file_exists(const std::string &name)
{
  std::ifstream f(name.c_str());
  return f.good();
}

bool dir_exist(const std::string &s)
{
  struct stat buffer;
  return (stat(s.c_str(), &buffer) == 0);
}

static inline double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}

void reconstruct_point_cloud(pcl::PointCloud<PointXYZIR>::Ptr original_pt_cloud, pcl::PointCloud<PointXYZIR>::Ptr noise_pt_cloud, 
                            pcl::PointCloud<PointXYZIR>::Ptr non_noise_pt_cloud, cv::Mat first_range_img, cv::Mat first_intensity_img, cv::Mat first_ret_type_img, 
                            cv::Mat last_range_img, cv::Mat last_intensity_img, cv::Mat last_ret_type_img, cv::Mat labels_first, cv::Mat labels_last,
                            cv::Mat point_cloud_first_image, cv::Mat point_cloud_last_image){
  for (int i = 0; i < 40; i++) { // range images are 40 x 1800 in size
    for (int j = 0; j < 1800; j++) {     
      PointXYZIR point_first;
      point_first.x = point_cloud_first_image.at<cv::Vec3f>(i,j)[0];
      point_first.y = point_cloud_first_image.at<cv::Vec3f>(i,j)[1];
      point_first.z = point_cloud_first_image.at<cv::Vec3f>(i,j)[2];   
      PointXYZIR point_last;
      point_last.x = point_cloud_last_image.at<cv::Vec3f>(i,j)[0];
      point_last.y = point_cloud_last_image.at<cv::Vec3f>(i,j)[1];
      point_last.z = point_cloud_last_image.at<cv::Vec3f>(i,j)[2];
      if (point_first.x != 0.0 && point_first.y != 0.0 && point_first.z != 0.0 ){
        original_pt_cloud->push_back(point_first);
        if (labels_first.at<uint8_t>(i,j) == 255){ //noise point
          noise_pt_cloud->push_back(point_first);
        }
        else{
          non_noise_pt_cloud->push_back(point_first);
        }
      }
      if (point_last.x != 0.0 && point_last.y != 0.0 && point_last.z != 0.0 ){
        original_pt_cloud->push_back(point_last);
        if (labels_last.at<uint8_t>(i,j) == 255){ //noise point
          noise_pt_cloud->push_back(point_last);
        }
        else{
          non_noise_pt_cloud->push_back(point_last);
        }      
      }
    }
  }  

  //publish point clouds
  original_pt_cloud->header.stamp = pcl_conversions::toPCL(ros::Time());
  original_pt_cloud->header.frame_id = "pandar";
  original_pt_cloud->height = 1;
  original_pcl_pub.publish(original_pt_cloud);                   
  noise_pt_cloud->header.stamp = pcl_conversions::toPCL(ros::Time());
  noise_pt_cloud->header.frame_id = "pandar";
  noise_pt_cloud->height = 1;
  noise_pcl_pub.publish(noise_pt_cloud);   
  non_noise_pt_cloud->header.stamp = pcl_conversions::toPCL(ros::Time());
  non_noise_pt_cloud->header.frame_id = "pandar";
  non_noise_pt_cloud->height = 1;
  non_noise_pcl_pub.publish(non_noise_pt_cloud);   

}

void process_range_images(const std::string particle_label_path, const std::string point_cloud_images_path ,const std::string range_images_path, int ind){
    std::stringstream first_range_name, first_intensity_name, first_ret_type_name;
    std::stringstream last_range_name, last_intensity_name, last_ret_type_name, first_label_name, last_label_name, point_cloud_first_name, point_cloud_last_name; 
    std::string ss1 = "/first_depth/";
    std::string ss2 = "/first_intensity/";
    std::string ss3 = "/first_return_type/"; 
    std::string ss4 = "/last_depth/";      
    std::string ss5 = "/last_intensity/";      
    std::string ss6 = "/last_return_type/"; 
    std::string ss7 = "/first/";    
    std::string ss8 = "/last/";    
    std::string ss9 = "/point_cloud_first_img_";   
    std::string ss10 = "/point_cloud_last_img_";   
    std::string type1 = ".png";       
    std::string type2 = ".exr";    
    first_range_name<<range_images_path<<ss1<<(ind)<<type1;
    first_intensity_name<<range_images_path<<ss2<<(ind)<<type1;
    first_ret_type_name<<range_images_path<<ss3<<(ind)<<type1;
    last_range_name<<range_images_path<<ss4<<(ind)<<type1;
    last_intensity_name<<range_images_path<<ss5<<(ind)<<type1;
    last_ret_type_name<<range_images_path<<ss6<<(ind)<<type1;
    first_label_name<<particle_label_path<<ss7<<(ind)<<type1;    
    last_label_name<<particle_label_path<<ss8<<(ind)<<type1; 
    point_cloud_first_name<<point_cloud_images_path<<ss9<<(ind)<<type2;
    point_cloud_last_name<<point_cloud_images_path<<ss10<<(ind)<<type2;

    //Reading the range images
    cv::Mat first_range_img = cv::imread(first_range_name.str(), cv::IMREAD_UNCHANGED);
    cv::Mat first_intensity_img = cv::imread(first_intensity_name.str(), cv::IMREAD_UNCHANGED);
    cv::Mat first_ret_type_img = cv::imread(first_ret_type_name.str(), cv::IMREAD_UNCHANGED);
    cv::Mat last_range_img = cv::imread(last_range_name.str(), cv::IMREAD_UNCHANGED);
    cv::Mat last_intensity_img = cv::imread(last_intensity_name.str(), cv::IMREAD_UNCHANGED);
    cv::Mat last_ret_type_img = cv::imread(last_ret_type_name.str(), cv::IMREAD_UNCHANGED);
    
    //Reading the label images
    cv::Mat labels_first = cv::imread(first_label_name.str(), cv::IMREAD_UNCHANGED);
    cv::Mat labels_last = cv::imread(last_label_name.str(), cv::IMREAD_UNCHANGED);    

    cv::Mat point_cloud_first_image = cv::imread(point_cloud_first_name.str(), cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
    cv::Mat point_cloud_last_image = cv::imread(point_cloud_last_name.str(), cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
    //Reconstruct point cloud from image and labels
    pcl::PointCloud<PointXYZIR>::Ptr noise_pt_cloud (new pcl::PointCloud<PointXYZIR>);
    pcl::PointCloud<PointXYZIR>::Ptr non_noise_pt_cloud (new pcl::PointCloud<PointXYZIR>);
    pcl::PointCloud<PointXYZIR>::Ptr original_pt_cloud (new pcl::PointCloud<PointXYZIR>);
    reconstruct_point_cloud(noise_pt_cloud, non_noise_pt_cloud, original_pt_cloud, first_range_img, first_intensity_img, first_ret_type_img,
                            last_range_img, last_intensity_img, last_ret_type_img, labels_first, labels_last, point_cloud_first_image, point_cloud_last_image);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pandar_rain_filter_visualizer");

  ros::NodeHandle private_node_handle("~");

  std::string particle_labels_path;
  std::string point_cloud_images_path;
  std::string range_images_path;
  std::stringstream label_path_name;
  private_node_handle.param<std::string>("particle_labels_path", particle_labels_path, "");
  ROS_INFO("[%s] particle_labels_path: %s", ros::this_node::getName().c_str(), particle_labels_path.c_str());

  if (!file_exists(particle_labels_path))
  {
    ROS_WARN("[%s] particle_labels_path: %s does not exist. Terminating.", ros::this_node::getName().c_str(), particle_labels_path.c_str());
    return 1;
  }


  private_node_handle.param<std::string>("point_cloud_images_path", point_cloud_images_path, "");
  ROS_INFO("[%s] point_cloud_images_path: %s", ros::this_node::getName().c_str(), point_cloud_images_path.c_str());

  if (!file_exists(point_cloud_images_path))
  {
    ROS_WARN("[%s] point_cloud_images_path: %s does not exist. Terminating.", ros::this_node::getName().c_str(), point_cloud_images_path.c_str());
    return 1;
  }


  private_node_handle.param<std::string>("range_images_path", range_images_path, "");
  ROS_INFO("[%s] range_images_path: %s", ros::this_node::getName().c_str(), range_images_path.c_str());

  if (!file_exists(range_images_path))
  {
    ROS_WARN("[%s] range_images_path: %s does not exist. Terminating.", ros::this_node::getName().c_str(), range_images_path.c_str());
    return 1;
  }


  // publishers
  original_pcl_pub = private_node_handle.advertise<sensor_msgs::PointCloud2>("/original_pt_cloud", 10);
  noise_pcl_pub = private_node_handle.advertise<sensor_msgs::PointCloud2>("/noise_pt_cloud", 10);
  non_noise_pcl_pub = private_node_handle.advertise<sensor_msgs::PointCloud2>("/non_noise_pt_cloud", 10);

  std::cout << "Reading range images and rain labels..." << std::endl;
  // Count number of range images & labels
  std::string label_first_path = "/first/"; 
  label_path_name<<particle_labels_path<<label_first_path;
  boost::filesystem::path the_path(label_path_name.str());
  int file_cnt = std::count_if(
        boost::filesystem::directory_iterator(the_path),
        boost::filesystem::directory_iterator(),
        static_cast<bool(*)(const boost::filesystem::path&)>(boost::filesystem::is_regular_file) );

  std::cout << "There are in total " << file_cnt << " range images & labels! "<< std::endl;
  size_t messages = 0;
  size_t total_messages = file_cnt;

  for (int cnt = 0; cnt < total_messages; cnt++){
    process_range_images(particle_labels_path, point_cloud_images_path ,range_images_path ,  cnt);
    std::cout << "\rProgress: (" << cnt << " / " << total_messages << ") " << std::endl;
 
  }

  std::cout << "Range and Label images generated! " << std::endl;
  return 0;
}