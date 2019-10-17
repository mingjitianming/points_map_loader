#ifndef MAP_LOADER_H
#define MAP_LOADER_H

#include <ros/ros.h>
#include <sys/stat.h>
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <map>
#include <unordered_map>
#include <algorithm>
#include <visualization_msgs/Marker.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
namespace map_loader
{

struct Centroid
{
    std::string path;
    double centroid_x;
    double centroid_y;
};

using CentroidList = std::vector<Centroid>;
using Tbl = std::vector<std::vector<std::string>>;
using OcTreeT = octomap::OcTree;

class MapLoader
{
public:
    MapLoader(double &margin, std::string &PATH, std::string map_format = "bt");
    ~MapLoader() = default;

    void init();

protected:
    bool hadCentroidFile(const std::string &path);
    bool sameDir(std::string &path1, std::string &path2);
    Tbl read_csv(const std::string &path);
    void write_csv(const std::string &path, const Tbl &tbl);
    CentroidList read_centroidlist(const std::string &path);
    void write_centroidlist(const std::string &path, const CentroidList &centroids);
    void computePCDCentroid(const std::vector<std::string> files);
    void computeBTCentroid(const std::vector<std::string> files);
    void publishMap(const geometry_msgs::PoseStamped &msg);
    void publishCurrentMap(sensor_msgs::PointCloud2 &&cloud);
    void inArea(const geometry_msgs::Pose &p, std::map<double, Centroid> &dist_centroids);
    sensor_msgs::PointCloud2 createMapFromBT(std::vector<std::string> &map_paths);
    sensor_msgs::PointCloud2 createMapFromPCD(std::vector<std::string> &map_paths);
    void inital_map(CentroidList centroids);
    void visual_centroids(std::unordered_map<std::string, std::pair<double, double>> &new_centroids,
                          std::vector<std::pair<double, double>> &last_centroids);
    bool isSpeckleNode(const octomap::OcTreeKey &key, const OcTreeT *m_octree);

private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub;
    ros::Publisher map_pub;
    ros::Publisher new_cen_pub;
    ros::Publisher last_cen_pub;
    double margin_;
    std::string map_dir_;
    std::string centroidFilePath_;
    CentroidList centroids;
    std::string map_format_;
    geometry_msgs::PoseStamped last_pose;
    ros::Time last_time;
    double update_interval;
    bool filter_speckles;
    std::unordered_map<std::string, std::shared_ptr<sensor_msgs::PointCloud2>> last_maps;
};
} // namespace map_loader

#endif
