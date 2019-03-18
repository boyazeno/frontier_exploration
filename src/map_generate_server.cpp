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
#include <geometry_msgs/Twist.h>

#include <move_base_msgs/MoveBaseAction.h>

#include <boost/foreach.hpp>
#define SMALL_RANGE 2.5
#define MULTIPLIER 1.5
#define PI 3.1415926

namespace frontier_exploration{
    /**
     * @brief server to receive the range of exploration and send request to explore_server. After action finished, save the map and return the path.
     **/
    class mapGenerateServer{
    private:
        ros::NodeHandle nh_;
        ros::ServiceServer service_;
        actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction> exploreClient;
        double duration;
        float range;



        /**
        * @brief call back function of map_generate. This will generate a client for explore_server
        * @param point Req Res request and response from service
        */
        bool callBackServer(frontier_exploration::ExploreRange::Request &req, frontier_exploration::ExploreRange::Response &res){
            
            //TODO
            //rotate one time
            mapGenerateServer::rotate_for_once();

            //send the goal to explore_server
            geometry_msgs::PolygonStamped input_;
            geometry_msgs::PointStamped center_point;
            
            center_point.header.stamp = ros::Time::now();
            center_point.header.frame_id = "base_link";
            center_point.point.z = 0.0;
            center_point.point.x = req.center.x;
            center_point.point.y = req.center.y;
    
            input_.header.stamp = ros::Time::now();
            input_.header.frame_id = "base_link";
            input_.polygon.points.resize(18);
            for(int i=0; i<18;++i){
                geometry_msgs::Point32 point;
                point.x = req.range*sin(i*2*PI/18);
                point.y = req.range*cos(i*2*PI/18);
                point.z = 0;
                input_.polygon.points[i]=point;
            } 
            range = req.range;
            ROS_INFO_STREAM("waiting for explore_server...");
            exploreClient.waitForServer();
            ROS_INFO_STREAM("connection to explore_server success!");
            ROS_INFO("Sending goal");
            frontier_exploration::ExploreTaskGoal goal;
            goal.explore_center = center_point;
            goal.explore_boundary = input_;
            //exploreClient.sendGoal(goal, &doneCb, &activeCb,&feedbackCb); 
            exploreClient.sendGoal(goal, boost::bind(&mapGenerateServer::doneActionCb, this, _1, _2),0,boost::bind(&mapGenerateServer::feedbackActionCb, this, _1));
            nh_.param("duration_until_finishing",duration,240.0);
            bool finished_before_timeout = exploreClient.waitForResult(ros::Duration(duration));
              if (finished_before_timeout)
            {
                actionlib::SimpleClientGoalState state = exploreClient.getState();
                ROS_INFO("Action finished: %s",state.toString().c_str());
                res.feedback = true;
            }
            else
            {
                ROS_INFO("Action did not finish before the time out.");
                res.feedback = false;
            }
            
            return true;
        }

        bool doneActionCb(const actionlib::SimpleClientGoalState& state, const frontier_exploration::ExploreTaskResultConstPtr& result){
            //TODO
            //save map
            
            
            ROS_WARN_STREAM("finished exploration in range of "<<range);
            return true;
        }

        bool feedbackActionCb(const frontier_exploration::ExploreTaskFeedbackConstPtr feedback)
        {
            geometry_msgs::PoseStamped base_position;
            ROS_INFO("Got Feedback of position: x : %6.2f  y: %6.2f  z: %6.2f", feedback->base_position.pose.position.x, feedback->base_position.pose.position.y, feedback->base_position.pose.position.z);
            return true;
        }

        bool rotate_for_once(){
            //ros::Subscriber odom_sub = n.advertise<nav_msgs::Odometry>("odom", 1);
            ros::Publisher cmd_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
            ros::Rate loop_rate(1.0);
            geometry_msgs::Twist vel;
            vel.linear.x=vel.linear.y=vel.linear.z=0.0;
            vel.angular.x=vel.angular.y=0.0;
            vel.angular.z=0.2;
            for(int i=0; i<= 32; ++i){
                cmd_pub.publish(vel);
                loop_rate.sleep();
            }
            vel.angular.z=0.0;
            cmd_pub.publish(vel);
            return true;
        }


    public:
        mapGenerateServer() :
            nh_(),
            exploreClient("explore_server", true)
        {
            service_ = nh_.advertiseService("map_generate", &mapGenerateServer::callBackServer, this);
            ROS_INFO_STREAM("mapGenerateServer waiting for goal...");
        }

        

        

    };

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_generate_server");

    frontier_exploration::mapGenerateServer Server;
    ros::spin();
    return 0;
}