#include <ros/ros.h>
#include <ros/time.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <map>

using namespace std;

class FollowJointTrajectoryAction
{
public:

   FollowJointTrajectoryAction(std::string name) :
      as_(nh_, name, false),
      action_name_(name)
   {  
      as_.registerGoalCallback(boost::bind(&FollowJointTrajectoryAction::goalCB, this));
      as_.registerPreemptCallback(boost::bind(&FollowJointTrajectoryAction::preemptCB, this));
      sub_joint_state_ = nh_.subscribe<sensor_msgs::JointState>("/ar607/joint_states", 1, &FollowJointTrajectoryAction::JointStateCallback, this);
      pub_move_arm_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/ar607/joint_path_command", 1, this);
      as_.start();
      ROS_INFO("action start");
   }
      
   ~FollowJointTrajectoryAction(void)
   {
   }

   void goalCB()
   {
      ROS_INFO("Goal Recieived");
      // Change first trajectory position to the current position
      std::map<std::string, double> goal_map;

      goal_ = as_.acceptNewGoal()->trajectory;
      
      for(auto i = 0; i < goal_.joint_names.size(); i++)
         goal_map[goal_.joint_names[i]] = goal_.points[goal_.points.size()-1].positions[i];

      // Send the Trajectory & Wait the Execution
      ROS_INFO("Moving...");
      pub_move_arm_.publish(goal_);

      // Wait the Execution
      double start_pos_tol = 25e-4;
      std::map<std::string, double> cur_map;
      do {
      ros::spinOnce();
      for(auto itr = goal_map.begin(); itr != goal_map.end(); itr++)
         cur_map[itr->first] = js_map_[itr->first];
      ros::spinOnce();
      }
      while( !isWithinRange(cur_map, goal_map, start_pos_tol) );
      //	ros::Duration(0.1).sleep();
      result_.error_code = result_.SUCCESSFUL;
      as_.setSucceeded(result_);
      ROS_INFO("Task Done !!");
   } 

   void preemptCB()
   {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      // set the action state to preempted
      as_.setPreempted();
   }

   void JointStateCallback(const sensor_msgs::JointState::ConstPtr& js)
   {
      for(int i = 0; i < js->name.size(); i++) {
         js_map_[js->name[i]] = js->position[i];
      }
   }

   bool isWithinRange(std::map<std::string, double> lhs,
                        std::map<std::string, double> rhs,
                        double full_range)
   {
      bool ret = true;
      double threshold = fabs(full_range/2.0);
      
      if(lhs.size()!=rhs.size())
         return false;
      
      for(auto lhs_itr=lhs.begin(); lhs_itr!=lhs.end(); lhs_itr++){
         if(fabs(lhs_itr->second - rhs[lhs_itr->first]) >= threshold)
         ret = false;
      }
      return ret;
   }

protected: 
   ros::NodeHandle nh_;
   actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
   std::string action_name_;
   control_msgs::FollowJointTrajectoryResult result_;  
   trajectory_msgs::JointTrajectory goal_;
   ros::Subscriber sub_joint_state_;
   ros::Publisher pub_move_arm_;
   std::map<std::string, double> js_map_;
};

int main(int argc, char** argv)
{
   ros::init(argc, argv, "action_server");

   FollowJointTrajectoryAction arm("/ar607/ar607_controller/follow_joint_trajectory");
   FollowJointTrajectoryAction hand("/ar607/hand_controller/follow_joint_trajectory");
   ros::spin();

   return 0;
}
