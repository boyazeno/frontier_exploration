#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <frontier_exploration/geometry_tools.h>

#include <frontier_exploration/ExploreTaskAction.h>
#include <frontier_exploration/ExploreTaskActionGoal.h>
#include <frontier_exploration/GetNextFrontier.h>
#include <frontier_exploration/ExploreRange.h>
#include <frontier_exploration/UpdateBoundaryPolygon.h>
#include <costmap_2d/footprint.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>

#include <move_base_msgs/MoveBaseAction.h>

#include <boost/foreach.hpp>
#define SMALL_RANGE 2.5
#define MULTIPLIER 1.5
#define PI 3.1415926


namespace frontier_exploration{

/**
 * @brief Client for FrontierExplorationServer that receives control points from rviz, and creates boundary polygon for frontier exploration
 */
class FrontierExplorationClient{

private:

    ros::NodeHandle nh_;

    ros::ServiceServer service_;


    /**
     * @brief Publish markers for visualization of points for boundary polygon.
     */
    
    /**
     * @brief call back function of send_Points_For_Exploration_Server
     * @param point Req Res request and response from service
     */
    bool pointCb(frontier_exploration::ExploreRange::Request &req, frontier_exploration::ExploreRange::Response &res){
      
       // double average_distance = polygonPerimeter(input_.polygon) / input_.polygon.points.size();
        //extracte information from request into range, middle point; response is string of feedback.
        double range = req.range;
        geometry_msgs::Point mpoint = req.center;
        double local_range = SMALL_RANGE;
        geometry_msgs::PointStamped point;
        geometry_msgs::PolygonStamped input_;
        bool keep=false;
        bool finished_before_timeout;
        
        actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction> exploreClient("explore_server", true);
        exploreClient.waitForServer();

        /*the local detect range is not bigger than 2*gloable range*/        
        while(local_range < 2*range){
                
        if(keep == false){
        //do exploration for one time   
        ROS_INFO("Sending goal");
        frontier_exploration::ExploreTaskGoal goal;
        //define the center point of local polygon
        point.header.frame_id = "base_link";
        point.header. stamp = ros::Time::now();
        point.point.x = point.point.y = point.point.z=0.0;
        //generate local polygon, and stored in input_
        input_.header.frame_id = "base_link";
        input_.header.stamp = ros::Time::now();
        input_.polygon.points.resize(18);
        for(int i=0; i<18; ++i){
            geometry_msgs::Point32 temp;
            temp.x = local_range*cos(i*2*PI/18);
            temp.y = local_range*sin(i*2*PI/18);
            temp.z = 0.0;
            input_.polygon.points[i]= temp;
        }
        goal.explore_center = point;
        goal.explore_boundary = input_;
        exploreClient.sendGoal(goal);  //here can use a callback function to get the current location.
        //check whether the action is finished or time out.           
        finished_before_timeout = exploreClient.waitForResult(ros::Duration(20.0));
        }
        else{
            //keep the last motion for 10 secs. 
        ROS_INFO("keep the last motion for 10 secs");
        ros::Duration(10.0).sleep();
        keep = false;
        finished_before_timeout = false;
        }
        
   
        if (finished_before_timeout&&exploreClient.getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("current goal is success,local rangel will be larger");
            //the action is successfully finished, so the local detect range will be larger
            local_range = local_range * MULTIPLIER;
            keep = false;
            continue;
        }
        else if(finished_before_timeout&&exploreClient.getState()==actionlib::SimpleClientGoalState::ACTIVE)
        {
            keep = true;
            actionlib::SimpleClientGoalState state = exploreClient.getState();
            ROS_INFO("current goal state is %s",state.toString().c_str());
        }
        else if(finished_before_timeout&&exploreClient.getState()==actionlib::SimpleClientGoalState::ABORTED)
        {
            keep = false;
            actionlib::SimpleClientGoalState state = exploreClient.getState();
            ROS_INFO("current goal state is %s",state.toString().c_str());
            //action is failed or timeout, which means the local region is not fully detected. should send the same detect range (base_link) again to detect another place with the same range.
            local_range = local_range * 1.1;
        }
        else
        {
            keep = false;
            actionlib::SimpleClientGoalState state = exploreClient.getState();
            ROS_INFO("current goal state is %s",state.toString().c_str());
            //action is failed or timeout, which means the local region is not fully detected. should send the same detect range (base_link) again to detect another place with the same range.
            local_range = SMALL_RANGE;
            
        }
        
        }
        
        
    }

public:

    /**
     * @brief Constructor for the client.
     */
    FrontierExplorationClient() :
        nh_()
    {
        service_ = nh_.advertiseService("send_Points_For_Exploration_Server", &FrontierExplorationClient::pointCb, this);
        ROS_INFO("Start sanding goal...");
    }    

};

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "explore_client");

    frontier_exploration::FrontierExplorationClient client;
    ros::spin();
    return 0;
}
