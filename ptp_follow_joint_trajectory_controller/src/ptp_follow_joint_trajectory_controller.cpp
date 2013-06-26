/*
 * Copyright (c) 2013, HiveGround Co., Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the HiveGround Co., Ltd., nor the name of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Mahisorn Wongphati
 *
 */

#include <ptp_follow_joint_trajectory_controller/ptp_follow_joint_trajectory_controller.h>


using namespace hiveground_controllers;

// These functions are pulled from the spline_smoother package.
// They've been moved here to avoid depending on packages that aren't
// mature yet.

inline void generatePowers(int n, double x, double* powers)
{
  powers[0] = 1.0;
  for (int i = 1; i <= n; i++)
  {
    powers[i] = powers[i - 1] * x;
  }
}

inline void getQuinticSplineCoefficients(double start_pos, double start_vel, double start_acc, double end_pos,
                                              double end_vel, double end_acc, double time, std::vector<double>& coefficients)
{
  coefficients.resize(6);

  if (time == 0.0)
  {
    coefficients[0] = end_pos;
    coefficients[1] = end_vel;
    coefficients[2] = 0.5 * end_acc;
    coefficients[3] = 0.0;
    coefficients[4] = 0.0;
    coefficients[5] = 0.0;
  }
  else
  {
    double T[6];
    generatePowers(5, time, T);

    coefficients[0] = start_pos;
    coefficients[1] = start_vel;
    coefficients[2] = 0.5 * start_acc;
    coefficients[3] = (-20.0 * start_pos + 20.0 * end_pos - 3.0 * start_acc * T[2] + end_acc * T[2]
        - 12.0 * start_vel * T[1] - 8.0 * end_vel * T[1]) / (2.0 * T[3]);
    coefficients[4] = (30.0 * start_pos - 30.0 * end_pos + 3.0 * start_acc * T[2] - 2.0 * end_acc * T[2]
        + 16.0 * start_vel * T[1] + 14.0 * end_vel * T[1]) / (2.0 * T[4]);
    coefficients[5] = (-12.0 * start_pos + 12.0 * end_pos - start_acc * T[2] + end_acc * T[2] - 6.0 * start_vel * T[1]
        - 6.0 * end_vel * T[1]) / (2.0 * T[5]);
  }
}

/**
 * \brief Samples a quintic spline segment at a particular time
 */
inline void sampleQuinticSpline(const std::vector<double>& coefficients, double time, double& position,
                                    double& velocity, double& acceleration)
{
  // create powers of time:
  double t[6];
  generatePowers(5, time, t);

  position = t[0] * coefficients[0] + t[1] * coefficients[1] + t[2] * coefficients[2] + t[3] * coefficients[3]
      + t[4] * coefficients[4] + t[5] * coefficients[5];

  velocity = t[0] * coefficients[1] + 2.0 * t[1] * coefficients[2] + 3.0 * t[2] * coefficients[3]
      + 4.0 * t[3] * coefficients[4] + 5.0 * t[4] * coefficients[5];

  acceleration = 2.0 * t[0] * coefficients[2] + 6.0 * t[1] * coefficients[3] + 12.0 * t[2] * coefficients[4]
      + 20.0 * t[3] * coefficients[5];
}

inline void getCubicSplineCoefficients(double start_pos, double start_vel, double end_pos, double end_vel, double time,
                                            std::vector<double>& coefficients)
{
  coefficients.resize(4);

  if (time == 0.0)
  {
    coefficients[0] = end_pos;
    coefficients[1] = end_vel;
    coefficients[2] = 0.0;
    coefficients[3] = 0.0;
  }
  else
  {
    double T[4];
    generatePowers(3, time, T);

    coefficients[0] = start_pos;
    coefficients[1] = start_vel;
    coefficients[2] = (-3.0 * start_pos + 3.0 * end_pos - 2.0 * start_vel * T[1] - end_vel * T[1]) / T[2];
    coefficients[3] = (2.0 * start_pos - 2.0 * end_pos + start_vel * T[1] + end_vel * T[1]) / T[3];
  }
}

bool PTPFollowJointTrajectoryController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh)
{
  nh_ = controller_nh;

  // get all joint states from the hardware interface
  const std::vector<std::string>& joint_names = hw->getNames();
  for (unsigned i = 0; i < joint_names.size(); i++)
    ROS_INFO_NAMED("PTPFollowJointController", "Got joint %s", joint_names[i].c_str());

  XmlRpc::XmlRpcValue joints;
  if (!controller_nh.getParam("joints", joints))
  {
    ROS_ERROR("No joints array setting");
    return false;
  }
  if (joints.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Joints array setting setting type mismatched");
    return false;
  }

  for (size_t i = 0; i < joints.size(); i++)
  {
    std::string joint_name;
    ROS_ASSERT(joints[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    joint_name = static_cast<std::string>(joints[i]);

    for(size_t j = 0; j < joint_names.size(); j++)
    {
      if(joint_name == joint_names[i])
      {
        ROS_INFO("Add joint %s to PTPFollowJointController", joint_name.c_str());
        joints_.push_back(hw->getHandle(joint_name));
        break;
      }
    }
  }

  q.resize(joints_.size());
  qd.resize(joints_.size());
  qdd.resize(joints_.size());

  // Creates a dummy trajectory
  boost::shared_ptr<SpecifiedTrajectory> traj_ptr(new SpecifiedTrajectory(1));
  SpecifiedTrajectory &traj = *traj_ptr;
  traj[0].duration = 0.0;
  traj[0].splines.resize(joints_.size());
  for (size_t j = 0; j < joints_.size(); ++j)
    traj[0].splines[j].coef[0] = 0.0;
  current_trajectory_box_.set(traj_ptr);


  action_server_follow_.reset(new FJTAS(controller_nh, "ptp_follow_joint_trajectory",
                                        boost::bind(&PTPFollowJointTrajectoryController::goalCBFollow, this, _1),
                                        boost::bind(&PTPFollowJointTrajectoryController::cancelCBFollow, this, _1),
                                        false));
  action_server_follow_->start();
  return true;
}

void PTPFollowJointTrajectoryController::starting(const ros::Time& time)
{
  ROS_INFO(__FUNCTION__);

  last_time_  = time;

  // Creates a "hold current position" trajectory.
  boost::shared_ptr<SpecifiedTrajectory> hold_ptr(new SpecifiedTrajectory(1));
  SpecifiedTrajectory &hold = *hold_ptr;
  hold[0].start_time = last_time_.toSec() - 0.001;
  hold[0].duration = 0.0;
  hold[0].splines.resize(joints_.size());
  for (size_t j = 0; j < joints_.size(); ++j)
  {
    hold[0].splines[j].coef[0] = joints_[j].getPosition();
  }
  current_trajectory_box_.set(hold_ptr);

}

void PTPFollowJointTrajectoryController::update(const ros::Time& time, const ros::Duration& period)
{
  ros::Duration dt = time - last_time_;
  last_time_ = time;

  boost::shared_ptr<RTGoalHandleFollow> current_active_goal_follow(rt_active_goal_follow_);

  boost::shared_ptr<const SpecifiedTrajectory> traj_ptr;
  current_trajectory_box_.get(traj_ptr);
  if (!traj_ptr)
    ROS_FATAL("The current trajectory can never be null");

  // Only because this is what the code originally looked like.
  const SpecifiedTrajectory &traj = *traj_ptr;

  // ------ Finds the current segment

  // Determines which segment of the trajectory to use.  (Not particularly realtime friendly).
  int seg = -1;
  while (seg + 1 < (int)traj.size() && traj[seg + 1].start_time < time.toSec())
  {
    ++seg;
  }

  if (seg == -1)
  {
    if (traj.size() == 0)
      ROS_ERROR("No segments in the trajectory");
    else
      ROS_ERROR("No earlier segments.  First segment starts at %.3lf (now = %.3lf)", traj[0].start_time, time.toSec());
    return;
  }

  // ------ Trajectory Sampling

  for (size_t i = 0; i < q.size(); ++i)
  {
    sampleSplineWithTimeBounds(traj[seg].splines[i].coef, traj[seg].duration, time.toSec() - traj[seg].start_time, q[i],
                               qd[i], qdd[i]);
  }

  //apply command
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    joints_[i].setCommand(q[i]);
  }

  // ------ Determines if the goal has failed or succeeded
  if (traj[seg].gh_follow && traj[seg].gh_follow == current_active_goal_follow)
  {
    if (seg == (int)traj.size() - 1)
    {
      double ds = 0;
      for (size_t i = 0; i < joints_.size(); i++)
      {
        ds += traj.back().gh_follow->gh_.getGoal()->trajectory.points.back().positions[i] - q[i];
      }

      if (fabs(ds) < 0.00001)
      {
        rt_active_goal_follow_.reset();
        traj[seg].gh_follow->preallocated_result_->error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
        traj[seg].gh_follow->setSucceeded(traj[seg].gh_follow->preallocated_result_);
      }
    }
  }
}

void PTPFollowJointTrajectoryController::stopping(const ros::Time& time)
{

}

template <class Enclosure, class Member>
static boost::shared_ptr<Member> share_member(boost::shared_ptr<Enclosure> enclosure, Member &member)
{
  actionlib::EnclosureDeleter<Enclosure> d(enclosure);
  boost::shared_ptr<Member> p(&member, d);
  return p;
}

void PTPFollowJointTrajectoryController::goalCBFollow(GoalHandleFollow gh)
{
  std::vector<std::string> joint_names(joints_.size());
  for (size_t j = 0; j < joints_.size(); ++j)
  {
    joint_names[j] = joints_[j].getName();
  }

  // Ensures that the joints in the goal match the joints we are commanding.
  if (!setsEqual(joint_names, gh.getGoal()->trajectory.joint_names))
  {
    ROS_ERROR("Joints on incoming goal don't match our joints");
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
    gh.setRejected(result);
    return;
  }

  preemptActiveGoal();

  gh.setAccepted();

  boost::shared_ptr<RTGoalHandleFollow> rt_gh(new RTGoalHandleFollow(gh));

  // Sends the trajectory along to the controller
  goal_handle_timer_ = nh_.createTimer(ros::Duration(0.01), &RTGoalHandleFollow::runNonRT, rt_gh);

  commandTrajectory(share_member(gh.getGoal(), gh.getGoal()->trajectory), rt_gh);

  rt_active_goal_follow_ = rt_gh;
  goal_handle_timer_.start();
}

void PTPFollowJointTrajectoryController::cancelCBFollow(GoalHandleFollow gh)
{
  boost::shared_ptr<RTGoalHandleFollow> current_active_goal(rt_active_goal_follow_);
  if (current_active_goal && current_active_goal->gh_ == gh)
  {
    rt_active_goal_follow_.reset();

    trajectory_msgs::JointTrajectory::Ptr empty(new trajectory_msgs::JointTrajectory);
    empty->joint_names.resize(joints_.size());
    for (size_t j = 0; j < joints_.size(); ++j)
      empty->joint_names[j] = joints_[j].getName();
    commandTrajectory(empty);

    // Marks the current goal as canceled.
    current_active_goal->gh_.setCanceled();
  }
}

void PTPFollowJointTrajectoryController::preemptActiveGoal()
{
  boost::shared_ptr<RTGoalHandleFollow> current_active_goal_follow(rt_active_goal_follow_);
  if (current_active_goal_follow)
  {
    rt_active_goal_follow_.reset();
    current_active_goal_follow->gh_.setCanceled();
  }
}
void PTPFollowJointTrajectoryController::commandTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr &msg,
                                                                   boost::shared_ptr<RTGoalHandleFollow> gh_follow)
{
  ros::Time time = last_time_ + ros::Duration(0.01);
  ROS_DEBUG("Figuring out new trajectory at %.3lf, with data from %.3lf", time.toSec(), msg->header.stamp.toSec());

  boost::shared_ptr<SpecifiedTrajectory> new_traj_ptr(new SpecifiedTrajectory);
  SpecifiedTrajectory &new_traj = *new_traj_ptr;

  //hold position
  if (msg->points.empty())
  {
    starting(ros::Time::now());
    return;
  }

  // ------ Correlates the joints we're commanding to the joints in the message
  std::vector<int> lookup(joints_.size(), -1); // Maps from an index in joints_ to an index in the msg
  for (size_t j = 0; j < joints_.size(); ++j)
  {
    for (size_t k = 0; k < msg->joint_names.size(); ++k)
    {
      if (msg->joint_names[k] == joints_[j].getName())
      {
        lookup[j] = k;
        break;
      }
    }

    if (lookup[j] == -1)
    {
      ROS_ERROR("Unable to locate joint %s in the commanded trajectory.", joints_[j].getName().c_str());
      if (gh_follow)
      {
        gh_follow->preallocated_result_->error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
        gh_follow->setAborted(gh_follow->preallocated_result_);
      }
      return;
    }
  }

  // ------ Grabs the trajectory that we're currently following.

  boost::shared_ptr<const SpecifiedTrajectory> prev_traj_ptr;
  current_trajectory_box_.get(prev_traj_ptr);
  if (!prev_traj_ptr)
  {
    ROS_FATAL("The current trajectory can never be null");
    return;
  }
  const SpecifiedTrajectory &prev_traj = *prev_traj_ptr;

  // ------ Copies over the segments from the previous trajectory that are still useful.

  // Useful segments are still relevant after the current time.
  int first_useful = -1;
  while (first_useful + 1 < (int)prev_traj.size() && prev_traj[first_useful + 1].start_time <= time.toSec())
  {
    ++first_useful;
  }

  // Useful segments are not going to be completely overwritten by the message's splines.
  int last_useful = -1;
  double msg_start_time;
  if (msg->header.stamp == ros::Time(0.0))
    msg_start_time = time.toSec();
  else
    msg_start_time = msg->header.stamp.toSec();

  while (last_useful + 1 < (int)prev_traj.size() && prev_traj[last_useful + 1].start_time < msg_start_time)
  {
    ++last_useful;
  }

  if (last_useful < first_useful)
    first_useful = last_useful;

  // Copies over the old segments that were determined to be useful.
  for (int i = std::max(first_useful, 0); i <= last_useful; ++i)
  {
    new_traj.push_back(prev_traj[i]);
  }

  // We always save the last segment so that we know where to stop if
  // there are no new segments.
  if (new_traj.size() == 0)
    new_traj.push_back(prev_traj[prev_traj.size() - 1]);

  // ------ Determines when and where the new segments start

  // Finds the end conditions of the final segment
  Segment &last = new_traj[new_traj.size() - 1];
  std::vector<double> prev_positions(joints_.size());
  std::vector<double> prev_velocities(joints_.size());
  std::vector<double> prev_accelerations(joints_.size());

  double t = (msg->header.stamp == ros::Time(0.0) ? time.toSec() : msg->header.stamp.toSec()) - last.start_time;
  ROS_DEBUG("Initial conditions at %.3f for new set of splines:", t);
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    sampleSplineWithTimeBounds(last.splines[i].coef, last.duration, t, prev_positions[i], prev_velocities[i],
                               prev_accelerations[i]);
    ROS_DEBUG("    %.2lf, %.2lf, %.2lf  (%s)", prev_positions[i], prev_velocities[i], prev_accelerations[i],
              joints_[i].getName().c_str());
  }

  // ------ Tacks on the new segments

  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> accelerations;

  std::vector<double> durations(msg->points.size());
  if (msg->points.size() > 0)
    durations[0] = msg->points[0].time_from_start.toSec();
  for (size_t i = 1; i < msg->points.size(); ++i)
    durations[i] = (msg->points[i].time_from_start - msg->points[i - 1].time_from_start).toSec();

  for (size_t i = 0; i < msg->points.size(); ++i)
  {
    Segment seg;

    if (msg->header.stamp == ros::Time(0.0))
      seg.start_time = (time + msg->points[i].time_from_start).toSec() - durations[i];
    else
      seg.start_time = (msg->header.stamp + msg->points[i].time_from_start).toSec() - durations[i];
    seg.duration = durations[i];
    seg.gh_follow = gh_follow;
    seg.splines.resize(joints_.size());

    // Checks that the incoming segment has the right number of elements.

    if (msg->points[i].accelerations.size() != 0 && msg->points[i].accelerations.size() != joints_.size())
    {
      ROS_ERROR("Command point %d has %d elements for the accelerations", (int )i,
                (int )msg->points[i].accelerations.size());
      return;
    }
    if (msg->points[i].velocities.size() != 0 && msg->points[i].velocities.size() != joints_.size())
    {
      ROS_ERROR("Command point %d has %d elements for the velocities", (int )i, (int )msg->points[i].velocities.size());
      return;
    }
    if (msg->points[i].positions.size() != joints_.size())
    {
      ROS_ERROR("Command point %d has %d elements for the positions", (int )i, (int )msg->points[i].positions.size());
      return;
    }

    // Re-orders the joints in the command to match the internal joint order.

    accelerations.resize(msg->points[i].accelerations.size());
    velocities.resize(msg->points[i].velocities.size());
    positions.resize(msg->points[i].positions.size());
    for (size_t j = 0; j < joints_.size(); ++j)
    {
      if (!accelerations.empty())
        accelerations[j] = msg->points[i].accelerations[lookup[j]];
      if (!velocities.empty())
        velocities[j] = msg->points[i].velocities[lookup[j]];
      if (!positions.empty())
        positions[j] = msg->points[i].positions[lookup[j]];
    }

    // Converts the boundary conditions to splines.

    for (size_t j = 0; j < joints_.size(); ++j)
    {
      if (prev_accelerations.size() > 0 && accelerations.size() > 0)
      {
        getQuinticSplineCoefficients(prev_positions[j], prev_velocities[j], prev_accelerations[j], positions[j],
                                     velocities[j], accelerations[j], durations[i], seg.splines[j].coef);
      }
      else if (prev_velocities.size() > 0 && velocities.size() > 0)
      {
        getCubicSplineCoefficients(prev_positions[j], prev_velocities[j], positions[j], velocities[j], durations[i],
                                   seg.splines[j].coef);
        seg.splines[j].coef.resize(6, 0.0);
      }
      else
      {
        seg.splines[j].coef[0] = prev_positions[j];
        if (durations[i] == 0.0)
          seg.splines[j].coef[1] = 0.0;
        else
          seg.splines[j].coef[1] = (positions[j] - prev_positions[j]) / durations[i];
        seg.splines[j].coef[2] = 0.0;
        seg.splines[j].coef[3] = 0.0;
        seg.splines[j].coef[4] = 0.0;
        seg.splines[j].coef[5] = 0.0;
      }
    }

    // Pushes the splines onto the end of the new trajectory.

    new_traj.push_back(seg);

    // Computes the starting conditions for the next segment

    prev_positions = positions;
    prev_velocities = velocities;
    prev_accelerations = accelerations;
  }

  // ------ Commits the new trajectory

  if (!new_traj_ptr)
  {
    ROS_ERROR("The new trajectory was null!");
    return;
  }

  current_trajectory_box_.set(new_traj_ptr);
  ROS_DEBUG("The new trajectory has %d segments", (int )new_traj.size());
}

bool PTPFollowJointTrajectoryController::setsEqual(const std::vector<std::string> &a, const std::vector<std::string> &b)
{
  if (a.size() != b.size())
    return false;

  for (size_t i = 0; i < a.size(); ++i)
  {
    if (count(b.begin(), b.end(), a[i]) != 1)
      return false;
  }

  for (size_t i = 0; i < b.size(); ++i)
  {
    if (count(a.begin(), a.end(), b[i]) != 1)
      return false;
  }

  return true;
}

void PTPFollowJointTrajectoryController::sampleSplineWithTimeBounds(const std::vector<double>& coefficients, double duration, double time,
                                                                             double& position, double& velocity, double& acceleration)
{
  if (time < 0)
   {
     double _;
     sampleQuinticSpline(coefficients, 0.0, position, _, _);
     velocity = 0;
     acceleration = 0;
   }
   else if (time > duration)
   {
     double _;
     sampleQuinticSpline(coefficients, duration, position, _, _);
     velocity = 0;
     acceleration = 0;
   }
   else
   {
     sampleQuinticSpline(coefficients, time, position, velocity, acceleration);
   }
}

PLUGINLIB_EXPORT_CLASS( hiveground_controllers::PTPFollowJointTrajectoryController, controller_interface::ControllerBase)
