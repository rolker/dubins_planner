#include "dubins_planner.h"

#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <tf2/utils.h>

PLUGINLIB_EXPORT_CLASS(dubins_planner::DubinsPlanner, project11_navigation::TaskToTaskWorkflow);

namespace dubins_planner
{

// Used by OMPL to check is a state is valid. In this case, we
// consult a costmap_2d.
class StateValidityChecker: public ompl::base::StateValidityChecker
{
public:
  StateValidityChecker(costmap_2d::Costmap2D costmap, const ompl::base::SpaceInformationPtr &si) :
       ompl::base::StateValidityChecker(si),
       costmap_(costmap)
  {
  }
 
  virtual bool isValid(const ompl::base::State *state) const
  {
    return getCost(state) != costmap_2d::LETHAL_OBSTACLE;
  }

  unsigned char getCost(const ompl::base::State *state) const
  {
    auto s = static_cast<const ompl::base::DubinsStateSpace::StateType*>(state);
    unsigned int x,y;
    if(costmap_.worldToMap(s->getX(), s->getY(), x, y))
      return costmap_.getCost(x,y);
    return costmap_2d::LETHAL_OBSTACLE;
  }

  costmap_2d::Costmap2D costmap_;
};

// Used by OMPL to calculate the cost of a state. This looks it up in a
// costmap_2d and normalizes the result.
class CostOptimizationObjective: public ompl::base::StateCostIntegralObjective
{
public:
  CostOptimizationObjective(const ompl::base::SpaceInformationPtr &si):
    ompl::base::StateCostIntegralObjective(si, true)
  {
    checker_ = std::dynamic_pointer_cast<StateValidityChecker>( si->getStateValidityChecker());
    if(!checker_)
      ROS_WARN_STREAM("checker is null!");
  }

  ompl::base::Cost stateCost(const ompl::base::State* s) const
  {
    return ompl::base::Cost(checker_->getCost(s)/254.0);
  }

  std::shared_ptr<StateValidityChecker> checker_;
};

ompl::base::OptimizationObjectivePtr getPathLengthObjective(const ompl::base::SpaceInformationPtr& si)
{
  return ompl::base::OptimizationObjectivePtr(new ompl::base::PathLengthOptimizationObjective(si));
}

ompl::base::OptimizationObjectivePtr getCostObjective(const ompl::base::SpaceInformationPtr& si)
{
  return ompl::base::OptimizationObjectivePtr(new CostOptimizationObjective(si));
}

ompl::base::ValidStateSamplerPtr allocateValidStateSampler(const ompl::base::SpaceInformation* si)
{
  return std::make_shared<ompl::base::GaussianValidStateSampler>(si);
}

//
// DubinsPlanner below, helper classes and functions above
//

void DubinsPlanner::configure(std::string name, project11_navigation::Context::Ptr context)
{
  context_ = context;
  ROS_INFO_STREAM("Initializing DubinsPlanner plugin with name " << name);
  ros::NodeHandle nh("~/" + name);
  nh.param("output_task_type", output_task_type_, output_task_type_);
  nh.param("output_task_name", output_task_name_, output_task_name_);

  plan_publisher_ = nh.advertise<nav_msgs::Path>("plan", 1);
}

void DubinsPlanner::setGoal(const project11_navigation::Task::Ptr& input)
{
  input_task_ = input;
  task_update_time_ = ros::Time();
  if(input_task_ && input_task_->message().poses.size() < 2)
    input_task_->setDone();
  output_task_.reset();
  plan_.clear();
  plan_ready_ = std::future<std::vector<geometry_msgs::PoseStamped> >();
  planner_setup_.reset();
  publishPlan();
}

bool DubinsPlanner::running()
{
  bool done = true;
  if(input_task_ && !input_task_->done())
  {
    if(input_task_->message().poses.size() < 2)
    {
      input_task_->setDone();
      output_task_.reset();
      return false;
    }
    
    done = false;

    if(plan_ready_.valid() && plan_ready_.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready)
    {
      plan_ = plan_ready_.get();
      ROS_INFO_STREAM("A path of length: " << plan_.size() << " was found" );
      if(!plan_.empty())
      {
        ROS_DEBUG_STREAM("Start: " << plan_.front().header.stamp << " end: " << plan_.back().header.stamp);
        for(auto t: input_task_->children().tasks())
          if(t->message().type == output_task_type_ && t->message().id == input_task_->getChildID(output_task_name_))
          {
            output_task_ = t;
            break;
          }
        if(!output_task_)
        {
          output_task_ = input_task_->createChildTaskBefore(project11_navigation::Task::Ptr(),output_task_type_);
          input_task_->setChildID(output_task_, output_task_name_);
        }
        auto out_msg = output_task_->message();
        out_msg.curved_trajectories.clear();
        out_msg.poses = plan_;
        output_task_->update(out_msg);

        publishPlan();
      }
    }

    if(!plan_.empty())
    {
      // is plan time based?
      if(!plan_.front().header.stamp.isZero() && plan_.back().header.stamp > plan_.front().header.stamp)
      {
        auto odom = context_->getOdometry();
        auto p = plan_.begin();
        while(p != plan_.end() && p->header.stamp < odom.header.stamp)
          p++;
        if(p != plan_.end())
        {
          double dt = std::max(1.0, (p->header.stamp-odom.header.stamp).toSec());
          double dx = p->pose.position.x - odom.pose.pose.position.x;
          double dy = p->pose.position.y - odom.pose.pose.position.y;
          double distance = sqrt(dx*dx + dy*dy);
          double needed_speed = distance/dt;
          ROS_DEBUG_STREAM("distance: " << distance << " time left: " << dt << " needed speed: " << needed_speed);
          if(distance > 15 && needed_speed > speed_*10.0)
          {
            ROS_INFO_STREAM("Can't keep up, resetting the planner. " << "distance: " << distance << " time left: " << dt << " needed speed: " << needed_speed);
            plan_.clear();
            planner_setup_.reset();
            plan_ready_ = std::future<std::vector<geometry_msgs::PoseStamped> >();

          }
        }
      }

    }

    if(plan_.empty() && !planner_setup_)
    {
      auto caps = context_->getRobotCapabilities();

      speed_ = caps.default_velocity.linear.x;
      double radius = caps.getTurnRadiusAtSpeed(speed_);
      if( radius <= 0.0)
        ROS_WARN_STREAM_THROTTLE(1.0, "Radius is not > zero: " << radius);

      auto start = input_task_->message().poses[0]; 
      auto goal = input_task_->message().poses[1];

      std::string map_frame = context_->environment().mapFrame(); 

      if(start.header.frame_id != map_frame)
      {
        try
        {
          context_->tfBuffer().transform(start, start, map_frame);
          start.header.frame_id = map_frame;
        }
        catch(const std::exception& e)
        {
          ROS_WARN_STREAM(e.what());
          return !done;
        }
      }
      if(goal.header.frame_id != map_frame)
      {
        try
        {
          context_->tfBuffer().transform(goal, goal, map_frame);
          goal.header.frame_id = map_frame;
        }
        catch(const std::exception& e)
        {
          ROS_WARN_STREAM(e.what());
          return !done;
        }
      }

      ROS_DEBUG_STREAM("radius: " << radius);
      auto space = std::make_shared<ompl::base::DubinsStateSpace>(radius);
  
      ompl::base::RealVectorBounds bounds(2);
      // \todo, replace with context envorionment bounds
      // double minx = costmap->getOriginX();
      // double miny = costmap->getOriginY();
      // double maxx = minx + costmap->getSizeInMetersX();
      // double maxy = miny + costmap->getSizeInMetersY();
      // bounds.setLow(0, minx);
      // bounds.setLow(1, miny);
      // bounds.setHigh(0, maxx);
      // bounds.setHigh(1, maxy);
      space->setBounds(bounds);

      planner_setup_ = std::make_shared<ompl::geometric::SimpleSetup>(space);

      auto si = planner_setup_->getSpaceInformation();

      // replace with abstact p11 nav state checker
      // auto state_checker = std::make_shared<StateValidityChecker>(*costmap, si);

      // planner_setup_->setStateValidityChecker(state_checker);

      si->setValidStateSamplerAllocator(allocateValidStateSampler);

      ompl::base::ScopedState<> start_state(space), goal_state(space);

      start_state[0] = start.pose.position.x;
      start_state[1] = start.pose.position.y;
      start_state[2] = tf2::getYaw(start.pose.orientation);

      goal_state[0] = goal.pose.position.x;
      goal_state[1] = goal.pose.position.y;
      goal_state[2] = tf2::getYaw(goal.pose.orientation);

      planner_setup_->setStartAndGoalStates(start_state, goal_state);

      // 1 meter is what percent of space size?
      // double diagonal_size = sqrt(pow(costmap->getSizeInMetersX(),2.0) + pow(costmap->getSizeInMetersY(),2.0));
      // double resolution = 1.0/diagonal_size;
      // ROS_INFO_STREAM("resolution as proportion:" << resolution);

      // si->setStateValidityCheckingResolution(resolution*costmap->getResolution());

      //auto planner = std::make_shared<ompl::geometric::RRTstar>(si);
      auto planner = std::make_shared<ompl::geometric::CForest>(si);
      auto thread_count = planner->getNumThreads();
      if (thread_count > 3)
        thread_count -= 2;
      planner->setNumThreads(thread_count);
      planner_setup_->setPlanner(planner);

      start_header_ = start.header;

      ompl::base::OptimizationObjectivePtr distanceOptimization(new ompl::base::PathLengthOptimizationObjective(si));

      ompl::base::OptimizationObjectivePtr costOptimization(new CostOptimizationObjective(si));

      planner_setup_->getProblemDefinition()->setOptimizationObjective(1.0*distanceOptimization + 5.0*costOptimization);

      planner_setup_->setup();
      planner_setup_->print();

      ROS_INFO_STREAM("longest segment length: " << space->getLongestValidSegmentLength() << " longest segment not validated: " << space->getLongestValidSegmentFraction()*space->getLongestValidSegmentLength());

      task_update_time_ = input_task_->lastUpdateTime();
    }

    if(planner_setup_ && !plan_ready_.valid())
    {
      plan_ready_ = std::async(&DubinsPlanner::plannerThread, this, planner_setup_, start_header_, speed_);
    }
  }

  return !done;
}

bool DubinsPlanner::getResult(project11_navigation::Task::Ptr& output)
{
  if(output_task_)
  {
    output = output_task_;
    return true;
  }
  return false;
}

std::vector<geometry_msgs::PoseStamped> DubinsPlanner::plannerThread(ompl::geometric::SimpleSetupPtr planner_setup, std_msgs::Header header, double speed)
{
  auto solved = planner_setup->solve(2.0);
  if(solved)
  {
    planner_setup->simplifySolution();
    auto path = planner_setup->getSolutionPath();
    //ROS_INFO_STREAM("A path of length: " << path.length() << " was found" );
    if(path.length() > 2.0)
      path.interpolate( path.length()/2);
    std::vector<geometry_msgs::PoseStamped> ret;
    for(auto state: path.getStates())
    {
      geometry_msgs::PoseStamped p;
      p.header = header;
      p.pose.position.x = static_cast<const ompl::base::DubinsStateSpace::StateType*>(state)->getX(); 
      p.pose.position.y = static_cast<const ompl::base::DubinsStateSpace::StateType*>(state)->getY();
      if(speed > 0.0 && !ret.empty())
      {
        double dx = p.pose.position.x - ret.back().pose.position.x;
        double dy = p.pose.position.y - ret.back().pose.position.y;
        double distance = sqrt(dx*dx + dy*dy);
        p.header.stamp = ret.back().header.stamp+ros::Duration(distance/speed);
      }
      ret.push_back(p);
    }

    return ret;
  }

  return std::vector<geometry_msgs::PoseStamped>();
}

void DubinsPlanner::publishPlan()
{
  nav_msgs::Path path;
  if(!plan_.empty())
  {
    path.header = plan_.front().header;
    path.poses = plan_;
  }
  plan_publisher_.publish(path);
}


} // namespace dubins_planner
