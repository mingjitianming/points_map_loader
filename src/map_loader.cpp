/*
   2019-01-17

*/
#include <map_loader.h>

namespace map_loader{
MapLoader::MapLoader(double &margin, std::string &PATH, std::string map_format) : margin_(margin), map_dir_(PATH), map_format_(map_format)
{
    std::cout<<map_format_<<std::endl;
    nh_ = ros::NodeHandle("~");
    std::string home = std::getenv("HOME");
    centroidFilePath_ = (home) + "/Autoware/ros/src/data/packages/map_file/nodes/points_map_loader/Centroidlist.txt";
    filter_speckles = false;
    nh_.param("filterSpeckles", filter_speckles, filter_speckles);
    last_time = ros::Time::now();
    init();
    }

    bool MapLoader::hadCentroidFile(const std::string& path)
    {
        struct stat st; // form <sys/stat.h>
        return (stat(path.c_str(), &st) == 0);        
    }

    bool MapLoader::sameDir(std::string& path1, std::string& path2)   
    {
        std::ifstream ifs(path2.c_str());
        if (!ifs.good())
            return false;

        std::string line;
        std::getline(ifs, line);
        std::string tmp_path;
        // std::stringstream str(line);
        // std::getline(str, tmp_path,',');
        int index = line.find_last_of('/');
        tmp_path = line.substr(0,++index);
        if (line.find(path1) == -1)
            return false;
        else if (line.find("." + map_format_) == -1)
            return false;
        return true;
    }

    Tbl MapLoader::read_csv(const std::string &path)
    {
        std::ifstream ifs(path.c_str());
        std::string line;
        Tbl ret;
        while (std::getline(ifs, line))
        {
            std::istringstream iss(line);
            std::string col;
            std::vector<std::string> cols;
            while (std::getline(iss, col, ','))
                cols.push_back(col);
            ret.push_back(cols);
        }
        return ret;
    }

    void MapLoader::write_csv(const std::string &path, const Tbl &tbl)
    {
        std::ofstream ofs(path.c_str());
        for (const std::vector<std::string> &cols : tbl)
        {
            std::string line;
            for (size_t i = 0; i < cols.size(); ++i)
            {
                if (i == 0)
                    line += cols[i];
                else
                    line += "," + cols[i];
            }
            ofs << line << std::endl;
        }
    }

    CentroidList MapLoader::read_centroidlist(const std::string &path)
    {
        Tbl tbl = read_csv(path);
        CentroidList ret;
        for (const std::vector<std::string> &cols : tbl)
        {
            Centroid centroid;
            centroid.path = cols[0];
            centroid.centroid_x = std::stod(cols[1]);
            centroid.centroid_y = std::stod(cols[2]);
            ret.push_back(centroid);
        }
        return ret;
    }

    void MapLoader::write_centroidlist(const std::string &path, const CentroidList &centroids)
    {
        Tbl tbl;
        for (const Centroid &centroid : centroids)
        {
            std::vector<std::string> cols;
            cols.push_back(centroid.path);
            cols.push_back(std::to_string(centroid.centroid_x));
            cols.push_back(std::to_string(centroid.centroid_y));

            tbl.push_back(cols);
        }
        write_csv(path, tbl);
    }

    void MapLoader::computePCDCentroid(const std::vector<std::string> files)
    {
        Eigen::Vector4f centroid_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::string file : files)
        {
            Centroid centroid;
            centroid.path = map_dir_ + file;

            if (pcl::io::loadPCDFile(centroid.path.c_str(), *cloud) != -1)
            {
                std::cout << "map.path: " << centroid.path.c_str() << std::endl;
                pcl::compute3DCentroid(*cloud, centroid_);
                centroid.centroid_x = centroid_[0];
                centroid.centroid_y = centroid_[1];
            }
            cloud->clear();
            centroids.push_back(centroid);
        }
        write_centroidlist(centroidFilePath_, centroids);
    }

    void MapLoader::computeBTCentroid(const std::vector<std::string> files)
    {
        
        OcTreeT* m_octree = new OcTreeT(0.2);
     //    OcTreeT* m_octree;
        for(auto file : files)
        {
        Centroid centroid;
        centroid.path = map_dir_ + file;
        std::cout <<"map path: "<< centroid.path<<std::endl; 
        double x_sum=0, y_sum=0;
        int num_oct = 0;
        if (!m_octree->readBinary(centroid.path))
          {
            std::cout<<file<<" load failed"<<std::endl;
            return;
          }
       
        for (OcTreeT::iterator it = m_octree->begin(m_octree->getTreeDepth()),
                                end = m_octree->end(); it != end; ++it)
            {
                if(m_octree->isNodeOccupied(*it))
                {
                    // double size = it.getSize();
                    x_sum += it.getX();
                    y_sum += it.getY();
                    ++num_oct;
                }
            }
            m_octree->clear();
            centroid.centroid_x = x_sum/num_oct;
            centroid.centroid_y = y_sum/num_oct;
            centroids.push_back(centroid);
        }
        write_centroidlist(centroidFilePath_, centroids);
    }

    void MapLoader::publishCurrentMap(sensor_msgs::PointCloud2&& cloud)
    {
      //  std::cout << "map_pub: " << map_pub.getNumSubscribers()<<std::endl;
        if (cloud.width != 0) // && map_pub.getNumSubscribers() > 0
        {
            cloud.header.frame_id = "/map";
            map_pub.publish(cloud);
        //    std::cout<<"map published"<<std::endl;
        }
    }

    sensor_msgs::PointCloud2 MapLoader::createMapFromPCD(std::vector<std::string>& map_paths)
    {
        sensor_msgs::PointCloud2 map_,part_;
        if (!last_maps.empty())
        {
            for (auto last_map : last_maps)
            {
                if (map_.width == 0)
                {
                    map_ = *(last_map.second);
                    continue;
                }
                map_.width += last_map.second->width;
                map_.row_step += last_map.second->row_step;
                map_.data.insert(map_.data.end(), last_map.second->data.begin(), last_map.second->data.end());
            }
        }
        for (const std::string &path : map_paths)
        {
            if (map_.width == 0)
            {
                if (pcl::io::loadPCDFile(path.c_str(), map_) == -1)
                    std::cerr << "load failed " << path << std::endl;
                std::shared_ptr<sensor_msgs::PointCloud2> tmp = std::make_shared<sensor_msgs::PointCloud2>(map_);
                last_maps.emplace(path, tmp);
            }
            else
            {
                if (pcl::io::loadPCDFile(path.c_str(), part_) == -1)
                    std::cerr << "load failed " << path << std::endl;
                std::shared_ptr<sensor_msgs::PointCloud2> tmp = std::make_shared<sensor_msgs::PointCloud2>(part_);
                last_maps.emplace(path, tmp);

                map_.width += part_.width;
                map_.row_step += part_.row_step;
                map_.data.insert(map_.data.end(), part_.data.begin(), part_.data.end());
            }
            std::cerr << "load " << path << std::endl;
            if (!ros::ok())
                break;
        }
        return map_;
    }

    sensor_msgs::PointCloud2 MapLoader::createMapFromBT(std::vector<std::string>& map_paths)
    { 
        sensor_msgs::PointCloud2 map_, part_;
        if (!last_maps.empty())
        {
            for (auto last_map : last_maps)
            {
                if (map_.width == 0)
                {
                    map_ = *(last_map.second);
                    continue;
                }
                map_.width += last_map.second->width;
                map_.row_step += last_map.second->row_step;
                map_.data.insert(map_.data.end(), last_map.second->data.begin(), last_map.second->data.end());
            }
        }

        OcTreeT* octree = new OcTreeT(0.2);
        for(auto path : map_paths)
        {
            if (!octree->readBinary(path))
                std::cout << path << " load failed" << std::endl;
            pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
            for(OcTreeT::iterator it = octree->begin(octree->getTreeDepth()), end = octree->end(); 
                it !=end; ++it )
                {
                    if(octree->isNodeOccupied(*it))
                    {
                        if (filter_speckles && (it.getDepth() == octree->getTreeDepth() + 1) && isSpeckleNode(it.getKey(), octree))
                        {
                            ROS_DEBUG("Ignoring single speckle");
                            continue;
                        }
                        double x = it.getX();
                        double y = it.getY();
                        double z = it.getZ();
                        pcl_cloud.push_back(pcl::PointXYZ(x,y,z));
                    }
                }
            octree->clear();

            if(map_.width == 0)
            {
                pcl::toROSMsg(pcl_cloud, map_);
                std::shared_ptr<sensor_msgs::PointCloud2> tmp = std::make_shared<sensor_msgs::PointCloud2>(map_);
                last_maps.emplace(path, tmp);
            }
            else
            {
                pcl::toROSMsg(pcl_cloud, part_);
                std::shared_ptr<sensor_msgs::PointCloud2> tmp = std::make_shared<sensor_msgs::PointCloud2>(part_);
                last_maps.emplace(path, tmp);

                map_.width += part_.width;
                map_.row_step += part_.row_step;
                map_.data.insert(map_.data.end(), part_.data.begin(), part_.data.end());
            }
            std::cerr << "load " << path << std::endl;
            if (!ros::ok())
                break;
        }
        return map_;
    }

    bool MapLoader::isSpeckleNode(const octomap::OcTreeKey &nKey, const OcTreeT *m_octree)
    {
        octomap::OcTreeKey key;
        bool neighborFound = false;
        for (key[2] = nKey[2] - 1; !neighborFound && key[2] <= nKey[2] + 1; ++key[2])
        {
            for (key[1] = nKey[1] - 1; !neighborFound && key[1] <= nKey[1] + 1; ++key[1])
            {
                for (key[0] = nKey[0] - 1; !neighborFound && key[0] <= nKey[0] + 1; ++key[0])
                {
                    if (key != nKey)
                    {
                        octomap::OcTreeNode *node = m_octree->search(key);
                        if (node && m_octree->isNodeOccupied(node))
                        {
                            // we have a neighbor => break!
                            neighborFound = true;
                        }
                    }
                }
            }
        }
        return neighborFound;
    }
    

    void MapLoader::inArea(const geometry_msgs::Pose& p, std::map<double, Centroid>& dist_centroids)
    {
        for(auto centroid : centroids)
        {
            double diff_x = centroid.centroid_x - p.position.x;
            double diff_y = centroid.centroid_y - p.position.y;
            double diff_2 = pow(diff_x,2) + pow(diff_y,2);
            if (diff_2 < margin_)
            {
                double robot_yaw = tf::getYaw(p.orientation);
                double diff_v = atan2(diff_y, diff_x);
                diff_v = (diff_v < 0) ? diff_v + 6.28 : diff_v;
                double diff = fabs(diff_v - robot_yaw);
                diff = (diff <= 3.14) ? diff : fabs(diff - 6.28);
                if(diff_2<3600)
                    dist_centroids.emplace(diff_2,centroid);
                else if(diff<0.78 || diff>2.35)
                    dist_centroids.emplace(diff_2,centroid);
            }
        }
        return ;
    }

        void MapLoader::publishMap(const geometry_msgs::PoseStamped &msg)
    {
        double diff_time = msg.header.stamp.sec - last_time.toSec();
        double diff_pose_2 = pow((msg.pose.position.x - last_pose.pose.position.x), 2) + pow((msg.pose.position.y - last_pose.pose.position.y), 2);
        if ((diff_time < update_interval) || diff_pose_2 < 10 || (diff_time < 3*update_interval && diff_pose_2 > 2500))
        {
           // std::cout << "时间差： "<<update_interval<< '\n';
            return;
        }

        last_time = ros::Time::now();
        last_pose = msg;
        ros::Time start_time = ros::Time::now();
        geometry_msgs::Pose p;
        p.position.x = msg.pose.position.x;
        p.position.y = msg.pose.position.y;
        p.orientation = msg.pose.orientation;

        std::vector<std::string> map_paths;
        std::map<double, Centroid> dist_centroids;
        std::unordered_map<std::string,std::pair<double, double>> xy_markers;
        std::vector<std::pair<double, double>> last_xy_markers;

        inArea(p, dist_centroids);
        
        if(dist_centroids.size()==0)
        {
           // std::cout << "x:" << p.position.x << "\n"
           //           << "y:" << p.position.y<<std::endl;
            ROS_WARN_STREAM("without map in margin*margin");
            return;
        }
        map_paths.clear();
        xy_markers.clear();
        if(dist_centroids.size()>8)
        {
            auto it = dist_centroids.begin();
            for(int i=0; i<8; ++i)
            {
                map_paths.emplace_back(it->second.path);
                xy_markers.emplace(it->second.path,std::make_pair(it->second.centroid_x,it->second.centroid_y));
                ++it;
            }
        }
        else
        {
            for(auto it : dist_centroids)
            {
                map_paths.emplace_back(it.second.path);
                xy_markers.emplace(it.second.path,std::make_pair(it.second.centroid_x,it.second.centroid_y));
            }
        }

        std::cout << "map_paths   " << map_paths.size() << std::endl;

        if (!last_maps.empty())
        {
            std::vector<std::string> erase_tmp;
            int num_path = map_paths.size();
            for (auto last_path : last_maps)
            {
                std::vector<std::string>::iterator it_map_paths = find(map_paths.begin(), map_paths.end(), last_path.first);
                if (it_map_paths != map_paths.end())
                {
                    map_paths.erase(it_map_paths);
                    last_xy_markers.emplace_back(xy_markers.at(last_path.first));
                    xy_markers.erase(last_path.first);
                }
                else
                {
                    erase_tmp.push_back(last_path.first);
                }
            }
            for (auto it : erase_tmp)
            {
                last_maps.erase(it);
            }
            std::cout << "last_maps数量： " << last_maps.size() << std::endl;
            if (last_maps.size() == num_path)
                return;
        }

        if(map_format_ == "bt")
            publishCurrentMap(createMapFromBT(map_paths));
        else if(map_format_ == "pcd")
            publishCurrentMap(createMapFromPCD(map_paths));

        visual_centroids(xy_markers, last_xy_markers);

        ros::Time end_time = ros::Time::now();
        update_interval = (end_time - start_time).toSec() ;
        return;
    }


    void MapLoader::visual_centroids(std::unordered_map<std::string, std::pair<double, double>> &new_centroids,
                                     std::vector<std::pair<double, double>> &last_centroids)
    {
        if(new_cen_pub.getNumSubscribers() > 0)
        {
            visualization_msgs::Marker visual_centroids;
            visual_centroids.header.frame_id = "/map";
            visual_centroids.header.stamp = ros::Time::now();
            visual_centroids.ns = "map_centroid";
            visual_centroids.action = visualization_msgs::Marker::ADD;
            visual_centroids.id = 0;
            visual_centroids.type = visualization_msgs::Marker::POINTS;
            visual_centroids.scale.x = 3;
            visual_centroids.scale.y = 3;

            visual_centroids.color.g = 1.0;
            visual_centroids.color.a = 1.0;

            for (auto marker : new_centroids)
            {
                geometry_msgs::Point p;
                p.x = marker.second.first;
                p.y = marker.second.second;
                visual_centroids.points.push_back(p);
            }

            new_cen_pub.publish(visual_centroids);
        }
        if(last_cen_pub.getNumSubscribers() > 0)
        {
            visualization_msgs::Marker last_visual_centroids;
            last_visual_centroids.header.frame_id = "/map";
            last_visual_centroids.header.stamp = ros::Time::now();
            last_visual_centroids.ns = "map_centroid";
            last_visual_centroids.action = visualization_msgs::Marker::ADD;
            last_visual_centroids.id = 0;
            last_visual_centroids.type = visualization_msgs::Marker::POINTS;
            last_visual_centroids.scale.x = 3;
            last_visual_centroids.scale.y = 3;

            last_visual_centroids.color.b = 1.0;
            last_visual_centroids.color.a = 1.0;

            for (auto marker : last_centroids)
            {
                geometry_msgs::Point p;
                p.x = marker.first;
                p.y = marker.second;
                last_visual_centroids.points.push_back(p);
            }

            last_cen_pub.publish(last_visual_centroids);
        }
    }


    void MapLoader::inital_map(CentroidList centroids)
    {
        std::map<double, std::string> find_close;
        for (auto centroid : centroids)
        {
            double dist = pow(centroid.centroid_x, 2) + pow(centroid.centroid_y, 2);
            find_close.emplace(dist, centroid.path);
        }

        std::vector<std::string> close;
        int num_init_pcd = 5;
        std::map<double, std::string>::iterator it = find_close.begin();
        for (int i = 0; i < ((find_close.size() > num_init_pcd) ? num_init_pcd : find_close.size()); ++i)
        {
            close.emplace_back(it->second);
            ++it;
        }

        if(map_format_ == "bt")
            publishCurrentMap(createMapFromBT(close));
        else if(map_format_ == "pcd")
            publishCurrentMap(createMapFromPCD(close));
    }


    void MapLoader::init()
    {
        struct dirent *ptr;
        DIR *dir;
        dir = opendir(map_dir_.c_str());
        std::vector<std::string> files;

        while ((ptr = readdir(dir)) != NULL)
        {
            //跳过'.'和'..'两个目录
            if (ptr->d_name[0] == '.' || ptr->d_type == 4)
                continue;
            char *end = ptr->d_name;
            while ((*end) != '.')
            {
                ++end;
            }
            if (map_format_ == "pcd")
            {
                if (*(++end) == 'p' && *(++end) == 'c' && *(++end) == 'd')
                    files.push_back(ptr->d_name);
            }
            else if (map_format_ == "bt")
            {
                if (*(++end) == 'b' && *(++end) == 't')
                    files.push_back(ptr->d_name);
            }
        }
        closedir(dir);
        std::cout << "flies: " << files.size() << std::endl;

        if(hadCentroidFile(centroidFilePath_) && sameDir(map_dir_, centroidFilePath_))
        {
            centroids = read_centroidlist(centroidFilePath_);
            std::cout<<"地图数量:  "<<centroids.size()<<std::endl;
            if(centroids.size() != files.size())
            {
                if(centroids.size() > files.size())
                {
                    std::cout<<"地图减少了 "<<centroids.size()-files.size()<<" 个"<<std::endl;
                    CentroidList temp_list;
                    for(auto centroid : centroids)
                    {
                        if(std::find(files.begin(),files.end(),centroid.path) != files.end())
                            temp_list.emplace_back(centroid);
                    }
                    centroids = temp_list;
                    write_centroidlist(centroidFilePath_, centroids);
                }
                else
                {
                    std::cout << "地图增加了 " << files.size() - centroids.size()<< " 个" << std::endl;
                    for(auto centroid : centroids)
                    {
                        auto it = std::find(files.begin(), files.end(), centroid.path);
                         if (it != files.end())
                            files.erase(it);
                    }
                    if (map_format_ == "pcd")
                        computePCDCentroid(files);
                    else if (map_format_ == "bt")
                        computeBTCentroid(files);
                }    
            }
        }
        else
        {
           // std::cout << "create CentroidList.txt" << std::endl;
            clock_t start, end;
            start = clock();
            if (map_format_ == "pcd")
               computePCDCentroid(files);
            else if(map_format_ == "bt")
               computeBTCentroid(files);
            end = clock();
            std::cout << "load all time: " << (double)(end - start) / CLOCKS_PER_SEC << "S" << std::endl;
        }        
        
        pose_sub = nh_.subscribe("/ndt_pose",5,&MapLoader::publishMap,this);
        map_pub = nh_.advertise<sensor_msgs::PointCloud2>("/points_map",1,true);
        new_cen_pub = nh_.advertise<visualization_msgs::Marker>("new_map_centroids",0);
        last_cen_pub = nh_.advertise<visualization_msgs::Marker>("last_map_centroids", 0);

        inital_map(centroids);
    }

}   // end namespace map_loader
