#include <itri_driver/itri_trajectory_action.h>
#include <itri_driver/itri_utils.h>
#include <industrial_robot_client/utils.h>
#include <industrial_utils/param_utils.h>
#include <industrial_utils/utils.h>

using namespace std;
using itri_driver::itri_utils::getJointGroups;

namespace itri_driver
{
namespace itri_trajectory_action
{

const double ITRI_JointTrajectoryAction::WATCHDOG_PERIOD_ = 1.5;
const double ITRI_JointTrajectoryAction::DEFAULT_GOAL_THRESHOLD_ = 0.01;

ITRI_JointTrajectoryAction::ITRI_JointTrajectoryAction(string ns, string name, vector<string> joints) :
    action_server_(node_, ns + "/" + name + "/" + "follow_joint_trajectory",
                   boost::bind(&ITRI_JointTrajectoryAction::goalCB, this, _1),
                   boost::bind(&ITRI_JointTrajectoryAction::cancelCB, this, _1), false),
                   has_active_goal_(false), controller_alive_(false), has_moved_once_(false), name_(name),
                   joint_names_(joints)
{
  ros::NodeHandle pn("~");

  pn.param("constraints/goal_threshold", goal_threshold_, DEFAULT_GOAL_THRESHOLD_);

  // if (!industrial_utils::param::getListParam("topic_list/" + ns + "/" + name + "/joints", joint_names_))
  //   ROS_ERROR_NAMED(name_, "Failed to initialize joint_names.");

  // The controller joint names parameter includes empty joint names for those joints not supported
  // by the controller.  These are removed since the trajectory action should ignore these.
  std::remove(joint_names_.begin(), joint_names_.end(), std::string());
  ROS_INFO_STREAM_NAMED(name_, "Filtered joint names to " << joint_names_.size() << " joints");

  pub_trajectory_command_ = node_.advertise<trajectory_msgs::JointTrajectory>(ns + "/" + name + "/joint_path_command", 1);
  sub_trajectory_state_ = node_.subscribe(ns + "/" + name + "/feedback_states", 1, &ITRI_JointTrajectoryAction::controllerStateCB, this);
  sub_robot_status_ = node_.subscribe(ns + "/" + name + "/robot_status", 1, &ITRI_JointTrajectoryAction::robotStatusCB, this);

  watchdog_timer_ = node_.createTimer(ros::Duration(WATCHDOG_PERIOD_), &ITRI_JointTrajectoryAction::watchdog, this, true);
  action_server_.start();
}

ITRI_JointTrajectoryAction::~ITRI_JointTrajectoryAction()
{
}

void ITRI_JointTrajectoryAction::robotStatusCB(const industrial_msgs::RobotStatusConstPtr &msg)
{
  last_robot_status_ = msg; //caching robot status for later use.
  has_moved_once_ = has_moved_once_ ? true : (last_robot_status_->in_motion.val == industrial_msgs::TriState::TRUE);
}

void ITRI_JointTrajectoryAction::watchdog(const ros::TimerEvent &e)
{
  // Some debug logging
  if (!last_trajectory_state_)
  {
    ROS_DEBUG_NAMED(name_, "Waiting for subscription to joint trajectory state");
  }

  ROS_WARN_NAMED(name_, "Trajectory state not received for %f seconds", WATCHDOG_PERIOD_);
  controller_alive_ = false;


  // Aborts the active goal if the controller does not appear to be active.
  if (has_active_goal_)
  {
    // last_trajectory_state_ is null if the subscriber never makes a connection
    if (!last_trajectory_state_)
    {
      ROS_WARN_NAMED(name_, "Aborting goal because we have never heard a controller state message.");
    }
    else
    {
      ROS_WARN_STREAM_NAMED(name_,
          "Aborting goal because we haven't heard from the controller in " << WATCHDOG_PERIOD_ << " seconds");
    }

    abortGoal();
  }
}

void ITRI_JointTrajectoryAction::goalCB(JointTractoryActionServer::GoalHandle gh)
{
  ROS_INFO_STREAM_NAMED(name_, "Received new goal");

  // reject all goals as long as we haven't heard from the remote controller
  if (!controller_alive_)
  {
    ROS_ERROR_NAMED(name_, "Joint trajectory action rejected: waiting for (initial) feedback from controller");
    control_msgs::FollowJointTrajectoryResult rslt;
    rslt.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    gh.setRejected(rslt, "Waiting for (initial) feedback from controller");

    // no point in continuing: already rejected
    return;
  }

  if (!gh.getGoal()->trajectory.points.empty())
  {
    if (industrial_utils::isSimilar(joint_names_, gh.getGoal()->trajectory.joint_names))
    {

      // Cancels the currently active goal.
      if (has_active_goal_)
      {
        ROS_WARN_NAMED(name_, "Received new goal, canceling current goal");
        abortGoal();
      }

      gh.setAccepted();
      active_goal_ = gh;
      has_active_goal_ = true;
      time_to_check_ = ros::Time::now() +
          ros::Duration(active_goal_.getGoal()->trajectory.points.back().time_from_start.toSec() / 2.0);
      has_moved_once_ = false;

      ROS_INFO_STREAM_NAMED(name_, "Publishing trajectory");

      current_traj_ = active_goal_.getGoal()->trajectory;
      pub_trajectory_command_.publish(current_traj_);

    }
    else
    {
      ROS_ERROR_NAMED(name_, "Joint trajectory action failing on invalid joints");
      control_msgs::FollowJointTrajectoryResult rslt;
      rslt.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
      gh.setRejected(rslt, "Joint names do not match");
    }
  }
  else
  {
    ROS_ERROR_NAMED(name_, "Joint trajectory action failed on empty trajectory");
    control_msgs::FollowJointTrajectoryResult rslt;
    rslt.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    gh.setRejected(rslt, "Empty trajectory");
  }

  // Adding some informational log messages to indicate unsupported goal constraints
  if (gh.getGoal()->goal_time_tolerance.toSec() > 0.0)
  {
    ROS_WARN_STREAM_NAMED(name_, "Ignoring goal time tolerance in action goal, may be supported in the future");
  }
  if (!gh.getGoal()->goal_tolerance.empty())
  {
    ROS_WARN_STREAM_NAMED(name_,
        "Ignoring goal tolerance in action, using paramater tolerance of " << goal_threshold_ << " instead");
  }
  if (!gh.getGoal()->path_tolerance.empty())
  {
    ROS_WARN_STREAM_NAMED(name_, "Ignoring goal path tolerance, option not supported by ROS-Industrial drivers");
  }
}

void ITRI_JointTrajectoryAction::cancelCB(JointTractoryActionServer::GoalHandle gh)
{
  ROS_DEBUG_NAMED(name_, "Received action cancel request");
  if (active_goal_ == gh)
  {
    // Stops the controller.
    trajectory_msgs::JointTrajectory empty;
    empty.joint_names = joint_names_;
    pub_trajectory_command_.publish(empty);

    // Marks the current goal as canceled.
    active_goal_.setCanceled();
    has_active_goal_ = false;
  }
  else
  {
    ROS_WARN_NAMED(name_, "Active goal and goal cancel do not match, ignoring cancel request");
  }
}

void ITRI_JointTrajectoryAction::controllerStateCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg)
{
  ROS_DEBUG_STREAM_NAMED(name_, "Checking controller state feedback");

  last_trajectory_state_ = msg;
  controller_alive_ = true;

  watchdog_timer_.stop();
  watchdog_timer_.start();

  if (!has_active_goal_)
  {
    //ROS_DEBUG_NAMED(name_, "No active goal, ignoring feedback");
    return;
  }
  if (current_traj_.points.empty())
  {
    ROS_INFO_NAMED(name_, "Current trajectory is empty, ignoring feedback");
    return;
  }

  if (!industrial_utils::isSimilar(joint_names_, msg->joint_names))
  {
    ROS_ERROR_NAMED(name_, "Joint names from the controller don't match our joint names.");
    return;
  }

  if (!has_moved_once_ && (ros::Time::now() < time_to_check_))
  {
    ROS_INFO_NAMED(name_, "Waiting to check for goal completion until halfway through trajectory");
    return;
  }

  // Checking for goal constraints
  // Checks that we have ended inside the goal constraints and has motion stopped

  ROS_DEBUG_STREAM_NAMED(name_, "Checking goal constraints");
  if (withinGoalConstraints(last_trajectory_state_, current_traj_))
  {
    if (last_robot_status_)
    {
      // Additional check for motion stoppage since the controller goal may still
      // be moving.  The current robot driver calls a motion stop if it receives
      // a new trajectory while it is still moving.  If the driver is not publishing
      // the motion state (i.e. old driver), this will still work, but it warns you.
      if (last_robot_status_->in_motion.val == industrial_msgs::TriState::FALSE)
      {
        ROS_INFO_NAMED("joint_trajectory_action.controllerStateCB", "Inside goal constraints - stopped moving - return success for action");
        active_goal_.setSucceeded();
        has_active_goal_ = false;
      }
      else if (last_robot_status_->in_motion.val == industrial_msgs::TriState::UNKNOWN)
      {
        ROS_INFO_NAMED(name_, "Inside goal constraints, return success for action");
        ROS_WARN_NAMED(name_, "Robot status in motion unknown, the robot driver node and controller code should be updated");
        active_goal_.setSucceeded();
        has_active_goal_ = false;
      }
      else
      {
        ROS_DEBUG_NAMED(name_, "Within goal constraints but robot is still moving");
      }
    }
    else
    {
      ROS_INFO_NAMED(name_, "Inside goal constraints, return success for action");
      ROS_WARN_NAMED(name_, "Robot status is not being published the robot driver node and controller code should be updated");
      active_goal_.setSucceeded();
      has_active_goal_ = false;
    }
  }
}

void ITRI_JointTrajectoryAction::abortGoal()
{
  // Stops the controller.
  trajectory_msgs::JointTrajectory empty;
  pub_trajectory_command_.publish(empty);

  // Marks the current goal as aborted.
  active_goal_.setAborted();
  has_active_goal_ = false;
}

bool ITRI_JointTrajectoryAction::withinGoalConstraints(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg,
                                                  const trajectory_msgs::JointTrajectory &traj)
{
  bool rtn = false;
  if (traj.points.empty())
  {
    ROS_WARN_NAMED(name_, "Empty joint trajectory passed to check goal constraints, return false");
    rtn = false;
  }
  else
  {
    int last_point = traj.points.size() - 1;

    if (industrial_robot_client::utils::isWithinRange(last_trajectory_state_->joint_names,
                                                      last_trajectory_state_->actual.positions, traj.joint_names,
                                                      traj.points[last_point].positions, goal_threshold_))
    {
      rtn = true;
    }
    else
    {
      rtn = false;
    }
  }
  return rtn;
}

} //itri_driver
} //itri_trajectory_action

using itri_driver::itri_trajectory_action::ITRI_JointTrajectoryAction;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "itri_trajectory_action");

  std::map<int, RobotGroup> robot_groups;
  getJointGroups("topic_list", robot_groups);

  for (int i = 0; i < robot_groups.size(); i++)
  {
    new ITRI_JointTrajectoryAction(robot_groups[i].get_ns(), robot_groups[i].get_name(), robot_groups[i].get_joint_names());
  }

  // ITRI_JointTrajectoryAction arm("", "right_arm_controller");
  // ITRI_JointTrajectoryAction arm("ar607", "ar607_controller");
  // ITRI_JointTrajectoryAction hand("ar607", "hand_controller");

  ros::spin();

  return 0;
}
