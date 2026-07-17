#include <fstream>

#include <ros/package.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <aeplanner_evaluation/Coverage.h>
#include <std_srvs/Empty.h>

#include <actionlib/client/simple_action_client.h>
#include <aeplanner/Node.h>
#include <aeplanner/aeplannerAction.h>
#include <rpl_exploration/FlyToAction.h>
#include <rrtplanner/rrtAction.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <nav_msgs/Path.h>
#include <tf2/utils.h>

// --- DIAGNOSTIC (logging only): cache latest UAV position to detect
//     freeze (Mode B) vs motion. Serviced by an AsyncSpinner on the default
//     callback queue; the action clients use their own spin threads. ---
namespace
{
double g_uav_x = 0.0, g_uav_y = 0.0, g_uav_z = 0.0;
bool g_uav_pose_valid = false;
int g_diag_srv_failures = 0;  // [baseline-repair] best_node failures skipped (never completed)
}  // namespace

void diagOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  g_uav_x = msg->pose.pose.position.x;
  g_uav_y = msg->pose.pose.position.y;
  g_uav_z = msg->pose.pose.position.z;
  g_uav_pose_valid = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "exploration");
  ros::NodeHandle nh;
  ROS_INFO("Started exploration");

  // Open logfile;
  std::string path = ros::package::getPath("rpl_exploration");
  std::ofstream logfile, pathfile;
  logfile.open(path + "/data/logfile.csv");
  pathfile.open(path + "/data/path.csv");

  // --- DIAGNOSTIC (logging only) ---
  std::ofstream diagfile;
  diagfile.open(path + "/data/diag.csv");
  diagfile << "iteration,elapsed_s,is_clear,frontier_count,rrt_path_len,"
           << "uav_x,uav_y,uav_z,verdict" << std::endl;

  ros::Publisher pub(nh.advertise<mrs_msgs::ReferenceStamped>("/uav1/control_manager/reference", 1000));

  ros::ServiceClient coverage_srv = nh.serviceClient<aeplanner_evaluation::Coverage>("/get_coverage");

  // --- DIAGNOSTIC (logging only) ---
  ros::AsyncSpinner diag_spinner(1);
  diag_spinner.start();
  ros::Subscriber diag_odom_sub =
      nh.subscribe("/uav1/control_manager/control_reference", 1, diagOdomCallback);

  // wait for fly_to server to start
  // ROS_INFO("Waiting for fly_to action server");
  actionlib::SimpleActionClient<rpl_exploration::FlyToAction> ac("fly_to", true);
  // ac.waitForServer();  // will wait for infinite time
  // BRINGUP FIX (Session 2 §F2): do not send goals into a not-yet-advertised
  // action server — the init-motion fly_to goal was dropped and waitForResult
  // blocked forever. Readiness wait only; init-motion content unchanged.
  while (ros::ok() && !ac.waitForServer(ros::Duration(5.0)))
    ROS_WARN("[bringup] waiting for fly_to action server...");
  // ROS_INFO("Fly to ction server started!");

  // wait for aep server to start
  ROS_INFO("Waiting for aeplanner action server");
  actionlib::SimpleActionClient<aeplanner::aeplannerAction> aep_ac("make_plan", true);
  aep_ac.waitForServer();  // will wait for infinite time
  ROS_INFO("aeplanner action server started!");

  // wait for fly_to server to start
  ROS_INFO("Waiting for rrt action server");
  actionlib::SimpleActionClient<rrtplanner::rrtAction> rrt_ac("rrt", true);
  // rrt_ac.waitForServer(); //will wait for infinite time
  // BRINGUP FIX (Session 2 §F2): wait for the rrt action server before first use.
  while (ros::ok() && !rrt_ac.waitForServer(ros::Duration(5.0)))
    ROS_WARN("[bringup] waiting for rrt action server...");
  ROS_INFO("rrt Action server started!");

  // Get current pose
  nav_msgs::Odometry::ConstPtr init_pose =
      ros::topic::waitForMessage<nav_msgs::Odometry>("/uav1/control_manager/control_reference");
  // double init_yaw = tf2::getYaw(init_pose->pose.orientation);
  double init_yaw = tf::getYaw(init_pose->pose.pose.orientation);
  // Up 2 meters and then forward one meter
  double initial_positions[8][4] = {
    { init_pose->pose.pose.position.x, init_pose->pose.pose.position.y, init_pose->pose.pose.position.z, init_yaw },
    { init_pose->pose.pose.position.x + 1.0 * std::cos(init_yaw),
      init_pose->pose.pose.position.y + 1.0 * std::sin(init_yaw), init_pose->pose.pose.position.z, init_yaw },
  };

  // This is the initialization motion, necessary that the known free space
  // allows the planning of initial paths.
  ROS_INFO("Starting the planner: Performing initialization motion");
  // geometry_msgs::PoseStamped last_pose;
  mrs_msgs::ReferenceStamped last_pose;

  for (int i = 0; i < 2; ++i)
  {
    rpl_exploration::FlyToGoal goal;
    goal.pose.reference.position.x = initial_positions[i][0];
    goal.pose.reference.position.y = initial_positions[i][1];
    goal.pose.reference.position.z = initial_positions[i][2];
    goal.pose.reference.heading = init_yaw;
    last_pose.reference = goal.pose.reference;

    ROS_INFO_STREAM("Goal: Fly to (" << goal.pose.reference.position.x << ", " << goal.pose.reference.position.y << ", "
                                     << goal.pose.reference.position.z << ", " << goal.pose.reference.heading << ") ");

    ROS_INFO_STREAM("Sending initial goal...");
    ac.sendGoal(goal);

    ac.waitForResult(ros::Duration(0));
  }

  // Start planning: The planner is called and the computed path sent to the
  // controller.
  int iteration = 0;
  int actions_taken = 1;

  ros::Time start = ros::Time::now();
  while (ros::ok())
  {
    ROS_INFO_STREAM("Planning iteration " << iteration);
    aeplanner::aeplannerGoal aep_goal;
    aep_goal.header.stamp = ros::Time::now();
    aep_goal.header.seq = iteration;
    aep_goal.header.frame_id = "/uav1/world_origin";
    aep_goal.actions_taken = actions_taken;
    aep_ac.sendGoal(aep_goal);

    // [hb] §Y1 rpl-side watchdog (log-only): if make_plan takes > 30 s, the
    // planner is stuck INSIDE aeplanner::execute() (its [hb] lines localize it).
    ros::WallTime aep_wait_start = ros::WallTime::now();
    int aep_wait_warns = 0;
    while (!aep_ac.waitForResult(ros::Duration(0.05)))
    {
      pub.publish(last_pose);
      const double waited = (ros::WallTime::now() - aep_wait_start).toSec();
      if (waited > 30.0 * (aep_wait_warns + 1))
      {
        ++aep_wait_warns;
        ROS_WARN("[hb] WAITING >%ds for aeplanner result (iter %d)",
                 30 * aep_wait_warns, iteration);
      }
    }

    // --- DIAGNOSTIC snapshot for this iteration (logging only) ---
    const bool diag_is_clear = aep_ac.getResult()->is_clear;
    const int diag_frontier_count =
        static_cast<int>(aep_ac.getResult()->frontiers.poses.size());
    int diag_rrt_path_len = -1;  // -1 => RRT not invoked this iteration
    std::string diag_verdict = "NORMAL_NBV";

    ros::Duration fly_time;
    if (aep_ac.getResult()->is_clear)
    {
      actions_taken = 0;

      ros::Time s = ros::Time::now();
      // geometry_msgs::PoseStamped goal_pose = aep_ac.getResult()->pose;
      mrs_msgs::ReferenceStamped goal_pose;
      goal_pose.reference.position = aep_ac.getResult()->pose.pose.position;
      goal_pose.reference.heading = tf::getYaw(aep_ac.getResult()->pose.pose.orientation);
      // Write path to file
      pathfile << goal_pose.reference.position.x << ", " << goal_pose.reference.position.y << ", "
               << goal_pose.reference.position.z << ", n" << std::endl;

      last_pose.reference = goal_pose.reference;
      rpl_exploration::FlyToGoal goal;
      goal.pose = goal_pose;
      ac.sendGoal(goal);

      ROS_INFO("[hb] pre flyto_nbv.waitForResult");             // [hb] §Y2
      // [baseline-repair §V6] bound the fly wait: a waypoint the tracker cannot
      // reach must not freeze the planner (Session-4 fly-hang). Cancel + continue
      // — AEP's own leg-skip, mirroring §X2. Planning logic untouched.
      if (!ac.waitForResult(ros::Duration(20.0)))
      {
        ROS_WARN("[baseline-repair] fly waypoint not reached in 20s — cancel + "
                 "continue");
        ac.cancelGoal();
      }
      ROS_INFO("[hb] post flyto_nbv.waitForResult");

      fly_time = ros::Time::now() - s;
    }
    else
    {
      // [baseline-repair] A best_node service failure is NOT a completion. Skip
      // this cycle's frontier decision and continue the loop (last_pose kept
      // being published by the busy-wait above); a fresh make_plan runs next
      // cycle. Completion may fire ONLY on a genuine empty (service_failed=false).
      if (aep_ac.getResult()->service_failed)
      {
        ++g_diag_srv_failures;
        diag_verdict = "SRV_FAILED_SKIP";
        diagfile << iteration << ", " << (ros::Time::now() - start).toSec() << ", "
                 << diag_is_clear << ", " << diag_frontier_count << ", "
                 << diag_rrt_path_len << ", " << g_uav_x << ", " << g_uav_y << ", "
                 << g_uav_z << ", " << diag_verdict << std::endl;
        diagfile.flush();
        ROS_ERROR("[baseline-repair] best_node service failed "
                  "(diag_srv_failures=%d) - skipping frontier decision, NOT "
                  "completing", g_diag_srv_failures);
        iteration++;
        continue;
      }

      rrtplanner::rrtGoal rrt_goal;
      rrt_goal.start.header.stamp = ros::Time::now();
      rrt_goal.start.header.frame_id = "/uav1/world_origin";
      rrt_goal.start.pose.position = last_pose.reference.position;
      rrt_goal.start.pose.orientation = tf::createQuaternionMsgFromYaw(last_pose.reference.heading);
      if (!aep_ac.getResult()->frontiers.poses.size())
      {
        // --- DIAGNOSTIC: Mode A — false-positive completion ---
        diag_verdict = "MODE_A_COMPLETE_NO_FRONTIERS";
        diagfile << iteration << ", " << (ros::Time::now() - start).toSec() << ", "
                 << diag_is_clear << ", " << diag_frontier_count << ", "
                 << diag_rrt_path_len << ", " << g_uav_x << ", " << g_uav_y << ", "
                 << g_uav_z << ", " << diag_verdict << std::endl;
        diagfile.flush();
        ROS_WARN("Exploration complete!");
        ROS_WARN("[DIAG] MODE A: is_clear=false and frontier set EMPTY. "
                 "Inspect pig_gain.csv: if global best gain fell below the "
                 "threshold (16) while volume remained, this is threshold-driven "
                 "false completion, not a true local minimum.");
        break;
      }
      for (auto it = aep_ac.getResult()->frontiers.poses.begin(); it != aep_ac.getResult()->frontiers.poses.end(); ++it)
      {
        rrt_goal.goal_poses.poses.push_back(*it);
      }

      rrt_ac.sendGoal(rrt_goal);
      ROS_INFO("[hb] pre rrt_ac.waitForResult");                // [hb] §Y2
      while (!rrt_ac.waitForResult(ros::Duration(0.05)))
      {
        pub.publish(last_pose);
      }
      ROS_INFO("[hb] post rrt_ac.waitForResult");
      nav_msgs::Path path = rrt_ac.getResult()->path;

      // --- DIAGNOSTIC: capture RRT path length; flag Mode B ---
      diag_rrt_path_len = static_cast<int>(path.poses.size());
      if (diag_rrt_path_len == 0)
      {
        diag_verdict = "MODE_B_FRONTIERS_BUT_NO_RRT_PATH";
        ROS_WARN("[DIAG] MODE B: %d frontier(s) reported but RRT returned an "
                 "EMPTY path. UAV will not move this iteration. Likely "
                 "unknown-space blocking the route or min_nodes too low.",
                 diag_frontier_count);
      }

      ros::Time s = ros::Time::now();
      for (int i = path.poses.size() - 1; i >= 0; --i)
      {
        // geometry_msgs::Pose goal_pose = path.poses[i].pose;
        mrs_msgs::ReferenceStamped goal_pose;
        goal_pose.reference.position = path.poses[i].pose.position;
        goal_pose.reference.heading = tf::getYaw(path.poses[i].pose.orientation);
        // Write path to file
        // pathfile << goal_pose.position.x << ", " << goal_pose.position.y << ", " << goal_pose.position.z << ", f"
        //          << std::endl;
        pathfile << goal_pose.reference.position.x << ", " << goal_pose.reference.position.y << ", "
                 << goal_pose.reference.position.z << ", f" << std::endl;
        // last_pose.pose = goal_pose;
        last_pose.reference = goal_pose.reference;
        rpl_exploration::FlyToGoal goal;
        // goal.pose.pose = goal_pose;
        goal.pose = goal_pose;
        ac.sendGoal(goal);

        ROS_INFO("[hb] pre flypath_leg %zu/%zu.waitForResult",  // [hb] §Y2
                 path.poses.size() - i, path.poses.size());
        // [baseline-repair §V6] bound the fly wait (see the flyto_nbv site).
        if (!ac.waitForResult(ros::Duration(20.0)))
        {
          ROS_WARN("[baseline-repair] fly waypoint not reached in 20s — cancel + "
                   "continue");
          ac.cancelGoal();
        }
        ROS_INFO("[hb] post flypath_leg %zu/%zu.waitForResult",
                 path.poses.size() - i, path.poses.size());
      }
      actions_taken = -1;
      fly_time = ros::Time::now() - s;
    }

    ros::Duration elapsed = ros::Time::now() - start;

    // --- DIAGNOSTIC row (every completed iteration) ---
    diagfile << iteration << ", " << elapsed.toSec() << ", " << diag_is_clear
             << ", " << diag_frontier_count << ", " << diag_rrt_path_len << ", "
             << g_uav_x << ", " << g_uav_y << ", " << g_uav_z << ", "
             << diag_verdict << std::endl;
    diagfile.flush();

    ROS_INFO_STREAM("Iteration: " << iteration << "  "
                                  << "Time: " << elapsed << "  "
                                  << "Sampling: " << aep_ac.getResult()->sampling_time.data << "  "
                                  << "Planning: " << aep_ac.getResult()->planning_time.data << "  "
                                  << "Collision check: " << aep_ac.getResult()->collision_check_time.data << "  "
                                  << "Flying: " << fly_time << " "
                                  << "Tree size: " << aep_ac.getResult()->tree_size);

    logfile << iteration << ", " << elapsed << ", " << aep_ac.getResult()->sampling_time.data << ", "
            << aep_ac.getResult()->planning_time.data << ", " << aep_ac.getResult()->collision_check_time.data << ", "
            << fly_time << ", " << aep_ac.getResult()->tree_size << std::endl;

    iteration++;
  }

  pathfile.close();
  logfile.close();
  diagfile.close();
}