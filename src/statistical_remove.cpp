#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read<pcl::PointXYZI>("/home/zmy/data1/bag/forklift/xibu_map/binary_pcd/binary_pcd_001.pcd", *cloud);

    std::cerr << "Cloud before filtering: " << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(0.2);
    sor.filter(*cloud_filtered);

    std::cerr << "Cloud after filtering: " << std::endl;

    // pcl::PCDWriter writer;
    // writer.writeBinary<pcl::PointXYZI>("/home/zmy/data1/bag/forklift/xibu_map/binary_pcd/binary_pcd_002.pcd", *cloud_filtered);
    pcl::io::savePCDFileBinary("/home/zmy/data1/bag/forklift/xibu_map/binary_pcd/binary_pcd_002.pcd", *cloud_filtered);

    // sor.setNegative(true);
    // sor.filter(*cloud_filtered);
    // writer.write<pcl::PointXYZI>("table_scene_lms400_outliers.pcd", *cloud_filtered, false);

    return (0);
}