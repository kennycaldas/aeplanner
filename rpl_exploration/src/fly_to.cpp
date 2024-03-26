#include <rpl_exploration/FlyToAction.h>  // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace rpl_exploration
{
class FlyTo
{
private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  actionlib::SimpleActionServer<rpl_exploration::FlyToAction> as_;

  tf::TransformListener listener;

public:
  FlyTo()
    : pub_(nh_.advertise<mrs_msgs::ReferenceStamped>("fly_to_cmd", 1000))
    , as_(nh_, "fly_to", boost::bind(&FlyTo::execute, this, _1, &as_), false)
  {
    ROS_INFO("Starting fly to server");
    as_.start();
  }
  void execute(const rpl_exploration::FlyToGoalConstPtr& goal,
               actionlib::SimpleActionServer<rpl_exploration::FlyToAction>* as)
  {
    ROS_INFO_STREAM("Got new goal: Fly to (" << goal->pose.reference.position.x << ", "
                                             << goal->pose.reference.position.y << ", "
                                             << goal->pose.reference.position.z << ") ");

    ros::Rate r(20);
    geometry_msgs::Point p = goal->pose.reference.position;

    float distance_to_goal = 9001;  // Distance is over 9000
    float yaw_diff = M_PI;

    tf::StampedTransform transform;
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));

    double goal_yaw;
    goal_yaw = goal->pose.reference.heading;

    // Check if target is reached...
    do
    {
      ROS_INFO_STREAM("Publishing goal to (" << p.x << ", " << p.y << ", " << p.z << ") ");
      pub_.publish(goal->pose);

      listener.waitForTransform("/uav1/world_origin", "/uav1/fcu", ros::Time(0), ros::Duration(10.0));
      listener.lookupTransform("/uav1/world_origin", "/uav1/fcu", ros::Time(0), transform);

      geometry_msgs::Point q;
      q.x = (float)transform.getOrigin().x();
      q.y = (float)transform.getOrigin().y();
      q.z = (float)transform.getOrigin().z();

      double current_yaw;
      current_yaw = tf::getYaw(transform.getRotation());

      ROS_INFO_STREAM("Current position: (" << q.x << ", " << q.y << ", " << q.z << ") ");
      geometry_msgs::Point d;
      d.x = p.x - q.x;
      d.y = p.y - q.y;
      d.z = p.z - q.z;

      distance_to_goal = sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
      ROS_INFO_STREAM("Distance to goal: " << distance_to_goal);
      yaw_diff = fabs(atan2(sin(goal_yaw - current_yaw), cos(goal_yaw - current_yaw)));

      r.sleep();
    } while (distance_to_goal > 0.8 or yaw_diff > 0.6 * M_PI);

    as->setSucceeded();
  }
};

}  // namespace rpl_exploration

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fly_to_server");

  rpl_exploration::FlyTo fly_to;

  ros::spin();
  return 0;
}