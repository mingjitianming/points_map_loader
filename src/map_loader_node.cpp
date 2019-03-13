#include<map_loader.h>

int main(int argc, char* argv[])
{

    ros::init(argc,argv,"map_loader_node");
    if(argc < 3)
    {
        ROS_ERROR_STREAM("Usage:");
        ROS_ERROR_STREAM("rosrun map_file points_map_loader margin_m map_folder [optional]map_format");
        return EXIT_FAILURE;
    }
    double margin = pow(atoi(argv[1]),2);
    std::string PATH = argv[2];
    PATH+="/";    
    std::string map_format = "bt";
    if(argc == 4)
    map_format = argv[3];

    map_loader::MapLoader maploader(margin,PATH,map_format);
    
    ros::spin();
    return 0;
}