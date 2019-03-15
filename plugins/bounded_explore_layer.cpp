#include <frontier_exploration/bounded_explore_layer.h>

#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/footprint.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <cmath>

#include <frontier_exploration/Frontier.h>
#include <frontier_exploration/UpdateBoundaryPolygon.h>
#include <frontier_exploration/GetNextFrontier.h>
#include <frontier_exploration/frontier_search.h>
#include <frontier_exploration/geometry_tools.h>

PLUGINLIB_EXPORT_CLASS(frontier_exploration::BoundedExploreLayer, costmap_2d::Layer)

namespace frontier_exploration
{

    using costmap_2d::LETHAL_OBSTACLE;
    using costmap_2d::NO_INFORMATION;
    using costmap_2d::FREE_SPACE;

    BoundedExploreLayer::BoundedExploreLayer(){}

    BoundedExploreLayer::~BoundedExploreLayer(){
        polygonService_.shutdown();
        frontierService_.shutdown();
        delete dsrv_;
        dsrv_ = 0;
    }

    void BoundedExploreLayer::onInitialize(){

        ros::NodeHandle nh_("~/" + name_);
        frontier_cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("frontiers",5);
       // frontier_next_goal = nh_.advertise<geometry_msgs::PoseStamped>("next_goal",1);//TODO write a publiser to publish the next pose array.
        configured_ = false;
        marked_ = false;

        bool explore_clear_space;
        nh_.param("explore_clear_space", explore_clear_space, true);
        if(explore_clear_space){
            default_value_ = NO_INFORMATION;
        }else{
            default_value_ = FREE_SPACE;
        }

        matchSize();

        nh_.param<bool>("resize_to_boundary", resize_to_boundary_, true);
        nh_.param<std::string>("frontier_travel_point", frontier_travel_point_, "closest");

        polygonService_ = nh_.advertiseService("update_boundary_polygon", &BoundedExploreLayer::updateBoundaryPolygonService, this);
        frontierService_ = nh_.advertiseService("get_next_frontier", &BoundedExploreLayer::getNextFrontierService, this);

        dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh_);
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
                    &BoundedExploreLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

    }


    void BoundedExploreLayer::matchSize(){
        Costmap2D* master = layered_costmap_->getCostmap();
        resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
                  master->getOriginX(), master->getOriginY());
    }


    void BoundedExploreLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level){
        enabled_ = config.enabled;
    }

    bool BoundedExploreLayer::getNextFrontierService(frontier_exploration::GetNextFrontier::Request &req, frontier_exploration::GetNextFrontier::Response &res){
        return getNextFrontier(req.start_pose, res.next_frontier);
    }

    bool BoundedExploreLayer::getNextFrontier(geometry_msgs::PoseStamped start_pose, geometry_msgs::PoseArray &next_frontier){

        //wait for costmap to get marked with boundary
        ros::Rate r(10);
        while(!marked_){
            ros::spinOnce();
            r.sleep();
        }

        if(start_pose.header.frame_id != layered_costmap_->getGlobalFrameID()){
            //error out if no transform available
            if(!tf_listener_.waitForTransform(layered_costmap_->getGlobalFrameID(), start_pose.header.frame_id,ros::Time::now(),ros::Duration(10))) {
                ROS_ERROR_STREAM("Couldn't transform from "<<layered_costmap_->getGlobalFrameID()<<" to "<< start_pose.header.frame_id);
                return false;
            }
            geometry_msgs::PoseStamped temp_pose = start_pose;
            tf_listener_.transformPose(layered_costmap_->getGlobalFrameID(),temp_pose,start_pose);
        }

        //initialize frontier search implementation
        FrontierSearch frontierSearch(*(layered_costmap_->getCostmap()));
        //get list of frontiers from search implementation
        std::list<Frontier> frontier_list = frontierSearch.searchFrom(start_pose.pose.position);

        if(frontier_list.size() == 0){
            ROS_DEBUG_STREAM("No frontiers found, exploration complete");
            return false;
        }

        //create placeholder for selected frontier
        Frontier selected;
        selected.min_distance = std::numeric_limits<double>::infinity();

        //pointcloud for visualization purposes
        pcl::PointCloud<pcl::PointXYZI> frontier_cloud_viz;
        pcl::PointXYZI frontier_point_viz(50);
        int max;
        int num_frontier_list = 0;
        num_frontier_list = frontier_list.size();
        int num_frontier_sorted = 0;
        float distance[num_frontier_list];
        int index_=0;
        BOOST_FOREACH(Frontier frontier, frontier_list){
            //collect all distance in distance[]
            distance[index_] = frontier.min_distance;
            index_ += 1;

            if(frontier.min_distance > MIN_DISTANCE){
                num_frontier_sorted += 1;
                //load frontier into visualization poitncloud
                frontier_point_viz.x = frontier.middle.x;  //changed to show middle point
                frontier_point_viz.y = frontier.middle.y;
                frontier_cloud_viz.push_back(frontier_point_viz);
                //check if this frontier is the nearest to robot
                if (frontier.min_distance < selected.min_distance ){
                    selected = frontier;
                    max = frontier_cloud_viz.size()-1;
                    ROS_WARN_STREAM("the current smallest distance is " << selected.min_distance);
                }
            }
        }


        //color selected frontier
        frontier_cloud_viz[max].intensity = 100;

        //publish visualization point cloud
        sensor_msgs::PointCloud2 frontier_viz_output;
        pcl::toROSMsg(frontier_cloud_viz,frontier_viz_output);
        frontier_viz_output.header.frame_id = layered_costmap_->getGlobalFrameID();
        frontier_viz_output.header.stamp = ros::Time::now();
        frontier_cloud_pub.publish(frontier_viz_output);

        //collect all frontiers,that is outside the smallest range, in distance_sorted.
        float distance_sorted[num_frontier_sorted];
        int index_distance_sorted=0;
        for(int i=0;i<num_frontier_list;++i){
            if(distance[i]>MIN_DISTANCE){
                distance_sorted[index_distance_sorted] = distance[i];
                index_distance_sorted+=1;
            }
        }

        //range the distance from small to big, and storage the distance in distance_sorted.
        for(int i=0;i<num_frontier_sorted-1;++i){
            for(int j=0;j<num_frontier_sorted-1-i;++j){
                if (distance_sorted[j] > distance_sorted[j+1]){
                    //swap j and j + 1
                    float temp = distance_sorted[j];
                    distance_sorted[j] =  distance_sorted[j + 1];
                    distance_sorted[j + 1] = temp;
                }
            }
        }

        //print the whole list of distance_sorted
        ROS_WARN_STREAM("the whole list of distance_sorted is ");
        for(int i=0;i<num_frontier_sorted;++i){
            ROS_WARN_STREAM(" The "<< i << "is "<<distance_sorted[i]);
        }
        
        //set goal pose to next frontier
        next_frontier.header.frame_id = layered_costmap_->getGlobalFrameID();
        next_frontier.header.stamp = ros::Time::now();
        next_frontier.poses.resize(num_frontier_sorted);

        //storage all the pose for sorted frontier in next_frontier.
        ROS_WARN_STREAM("now sort the frontier_list in the right order ");
        BOOST_FOREACH(Frontier frontier, frontier_list){
            ROS_INFO_STREAM("the min_distance of current frontier is " << frontier.min_distance);
            //storage the corresponded frontier into right range
            for(int i=0;i<num_frontier_sorted;++i){
                if(((float)frontier.min_distance - distance_sorted[i] < 0.0001) && ((float)frontier.min_distance - distance_sorted[i] > -1*0.0001)){
                    ROS_INFO_STREAM("current delta distance is : "<<(float)frontier.min_distance - distance_sorted[i]);
                    ROS_INFO_STREAM("current (float)min_distance is:"<<(float)frontier.min_distance<< "  and the distance_sorted"<<i<<"is : "<<distance_sorted[i]);
                    ROS_WARN_STREAM("the rank of "<< frontier.min_distance << "is " << i);
                    if(frontier_travel_point_ == "closest"){
                        ROS_DEBUG("Using closest point");
                        next_frontier.poses[i].position = frontier.initial;
                        next_frontier.poses[i].orientation = tf::createQuaternionMsgFromYaw( yawOfVector( frontier.initial_nbr, frontier.initial) );
                        ROS_INFO_STREAM("the current yawOfVector is :"<< yawOfVector( frontier.initial_nbr, frontier.initial));
                    }else if(frontier_travel_point_ == "middle"){
                        ROS_DEBUG("Using middle point");
                        next_frontier.poses[i].position = frontier.middle;
                        next_frontier.poses[i].orientation = tf::createQuaternionMsgFromYaw( yawOfVector(frontier.middle_nbr, frontier.middle));
                        ROS_INFO_STREAM("the current yawOfVector is :"<< yawOfVector( frontier.middle_nbr, frontier.middle));
                    }else if(frontier_travel_point_ == "centroid"){
                        ROS_DEBUG("Using centroid point");
                        next_frontier.poses[i].position = frontier.centroid;
                        next_frontier.poses[i].orientation = tf::createQuaternionMsgFromYaw( yawOfVector(frontier.centroid_nbr, frontier.centroid) );//
                        ROS_INFO_STREAM("the current yawOfVector is :"<< yawOfVector(frontier.centroid_nbr, frontier.centroid));
                    }else{
                        ROS_ERROR("Invalid 'frontier_travel_point' parameter, falling back to 'closest'");
                        next_frontier.poses[i].position = frontier.initial;
                        next_frontier.poses[i].orientation = tf::createQuaternionMsgFromYaw( yawOfVector(frontier.initial_nbr, frontier.initial) );
                    }
                    ROS_WARN_STREAM("the quanternion for the" << i <<"is: x: " << next_frontier.poses[i].orientation.x <<" y: " <<next_frontier.poses[i].orientation.y<<" z:"<<next_frontier.poses[i].orientation.z<<" w: "<<next_frontier.poses[i].orientation.w);
                }
            }
            

        }
   
        //for closest and middle point, we use local gradient as orientation
        //next_frontier.pose.orientation = tf::createQuaternionMsgFromYaw( yawOfVector(start_pose.pose.position, next_frontier.pose.position) );
        //frontier_next_goal.publish(next_frontier.pose);
        return true;

    }

    bool BoundedExploreLayer::updateBoundaryPolygonService(frontier_exploration::UpdateBoundaryPolygon::Request &req, frontier_exploration::UpdateBoundaryPolygon::Response &res){

        return updateBoundaryPolygon(req.explore_boundary);

    }

    void BoundedExploreLayer::reset(){

        //reset costmap_ char array to default values
        marked_ = false;
        configured_ = false;
        memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));

    }

    bool BoundedExploreLayer::updateBoundaryPolygon(geometry_msgs::PolygonStamped polygon_stamped){

        //clear existing boundary, if any
        polygon_.points.clear();

        //error if no transform available between polygon and costmap
        if(!tf_listener_.waitForTransform(layered_costmap_->getGlobalFrameID(), polygon_stamped.header.frame_id,ros::Time::now(),ros::Duration(10))) {
            ROS_ERROR_STREAM("Couldn't transform from "<<layered_costmap_->getGlobalFrameID()<<" to "<< polygon_stamped.header.frame_id);
            return false;
        }

        //Transform all points of boundary polygon into costmap frame
        geometry_msgs::PointStamped in, out;
        in.header = polygon_stamped.header;
        BOOST_FOREACH(geometry_msgs::Point32 point32, polygon_stamped.polygon.points){
            in.point = costmap_2d::toPoint(point32);
            tf_listener_.transformPoint(layered_costmap_->getGlobalFrameID(),in,out);
            polygon_.points.push_back(costmap_2d::toPoint32(out.point));
        }

        //if empty boundary provided, set to whole map
        if(polygon_.points.empty()){
            geometry_msgs::Point32 temp;
            temp.x = getOriginX();
            temp.y = getOriginY();
            polygon_.points.push_back(temp);
            temp.y = getSizeInMetersY();
            polygon_.points.push_back(temp);
            temp.x = getSizeInMetersX();
            polygon_.points.push_back(temp);
            temp.y = getOriginY();
            polygon_.points.push_back(temp);
        }

        if(resize_to_boundary_){
            updateOrigin(0,0);

            //Find map size and origin by finding min/max points of polygon
            double min_x = std::numeric_limits<double>::infinity();
            double min_y = std::numeric_limits<double>::infinity();
            double max_x = -std::numeric_limits<double>::infinity();
            double max_y = -std::numeric_limits<double>::infinity();

            BOOST_FOREACH(geometry_msgs::Point32 point, polygon_.points){
                min_x = std::min(min_x,(double)point.x);
                min_y = std::min(min_y,(double)point.y);
                max_x = std::max(max_x,(double)point.x);
                max_y = std::max(max_y,(double)point.y);
            }

            //resize the costmap to polygon boundaries, don't change resolution
            int size_x, size_y;
            worldToMapNoBounds(max_x - min_x, max_y - min_y, size_x, size_y);
            layered_costmap_->resizeMap(size_x, size_y, layered_costmap_->getCostmap()->getResolution(), min_x, min_y);
            matchSize();
        }

        configured_ = true;
        marked_ = false;
        return true;
    }


    void BoundedExploreLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y){

        //check if layer is enabled and configured with a boundary
        if (!enabled_ || !configured_){ return; }

        //update the whole costmap
        *min_x = getOriginX();
        *min_y = getOriginY();
        *max_x = getSizeInMetersX()+getOriginX();
        *max_y = getSizeInMetersY()+getOriginY();

    }

    void BoundedExploreLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
        //check if layer is enabled and configured with a boundary
        if (!enabled_ || !configured_){ return; }

        //draw lines between each point in polygon
        MarkCell marker(costmap_, LETHAL_OBSTACLE);

        //circular iterator
        for(int i = 0, j = polygon_.points.size()-1; i < polygon_.points.size(); j = i++){

            int x_1, y_1, x_2, y_2;
            worldToMapEnforceBounds(polygon_.points[i].x, polygon_.points[i].y, x_1,y_1);
            worldToMapEnforceBounds(polygon_.points[j].x, polygon_.points[j].y, x_2,y_2);

            raytraceLine(marker,x_1,y_1,x_2,y_2);
        }
        //update the master grid from the internal costmap
        mapUpdateKeepObstacles(master_grid, min_i, min_j, max_i, max_j);


    }

    void BoundedExploreLayer::mapUpdateKeepObstacles(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
        if (!enabled_)
            return;

        unsigned char* master = master_grid.getCharMap();
        unsigned int span = master_grid.getSizeInCellsX();

        for (int j = min_j; j < max_j; j++)
        {
            unsigned int it = span*j+min_i;
            for (int i = min_i; i < max_i; i++)
            {
                //only update master grid if local costmap cell is lethal/higher value, and is not overwriting a lethal obstacle in the master grid
                if(master[it] != LETHAL_OBSTACLE && (costmap_[it] == LETHAL_OBSTACLE || costmap_[it] > master[it])){
                    master[it] = costmap_[it];
                }
                it++;
            }
        }
        marked_ = true;
    }
}
