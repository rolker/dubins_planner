#ifndef DUBINS_PLANNER_DUBINS_PLANNER_H
#define DUBINS_PLANNER_DUBINS_PLANNER_H

#include <project11_navigation/interfaces/task_to_task_workflow.h>
#include <future>
#include <ompl/geometric/SimpleSetup.h>
#include <std_msgs/Header.h>

namespace dubins_planner
{

class DubinsPlanner: public project11_navigation::TaskToTaskWorkflow
{
public:
  void configure(std::string name, project11_navigation::Context::Ptr context) override;
  void setGoal(const std::shared_ptr<project11_navigation::Task>& input) override;
  bool running() override;
  bool getResult(std::shared_ptr<project11_navigation::Task>& output) override;
private:
  std::vector<geometry_msgs::PoseStamped> plannerThread(ompl::geometric::SimpleSetupPtr planner_setup, std_msgs::Header header, double speed);
  void publishPlan();

  project11_navigation::Context::Ptr context_;
  std::shared_ptr<project11_navigation::Task> input_task_ = nullptr;
  std::shared_ptr<project11_navigation::Task> output_task_ = nullptr;

  ros::Time task_update_time_;

  std::future<std::vector<geometry_msgs::PoseStamped> > plan_ready_;
  std::vector<geometry_msgs::PoseStamped> plan_;

  ompl::geometric::SimpleSetupPtr planner_setup_;
  std_msgs::Header start_header_;
  double speed_;

  ros::Publisher plan_publisher_;

  std::string output_task_type_ = "follow_trajectory";
  std::string output_task_name_ = "navigation_trajectory";
};

} // namespace dubins_planner

#endif
