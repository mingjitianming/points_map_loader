#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <fstream>

using namespace message_filters;
void callback(const geometry_msgs::PoseStamped::ConstPtr &ndt_pose, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &gnss_pose)
{
    double diff_x_2 = pow((ndt_pose->pose.position.x - gnss_pose->pose.pose.position.x + 0.38345964), 2);
    double diff_y_2 = pow((ndt_pose->pose.position.y - gnss_pose->pose.pose.position.y + 0.021318574), 2);
    std::cout<<diff_x_2<<"\n"<<diff_y_2<<std::endl;
    double diff = sqrt(diff_x_2 + diff_y_2);
    // std::ofstream of("/home/zmy/test_ws/src/map_loader/estimate/octo_data",std::ofstream::app);
    std::ofstream of("/home/zmy/test_ws/src/map_loader/estimate/pcd_data", std::ofstream::app);
    of<<ndt_pose->header.stamp.sec<<" " 
      <<diff<<"\n";
    std::cout<<"chulizhong"<<std::endl;
}





int main(int argc, char **argv)
{
    ros::init(argc,argv,"estimate");
    ros::NodeHandle nh_;
    message_filters::Subscriber<geometry_msgs::PoseStamped> ndt_sub(nh_, "/ndt_pose",1);
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> gnss_sub(nh_, "/gnss_pose_m", 1);
    typedef sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PoseWithCovarianceStamped> MySyncPolicy;
    //typedef sync_policies::ExactTime<geometry_msgs::PoseStamped, geometry_msgs::PoseWithCovarianceStamped> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),ndt_sub,gnss_sub);
    sync.registerCallback(boost::bind(&callback,_1,_2));
    std::cout<<"lalal"<<std::endl;
    ros::spin();
    return 0;
}