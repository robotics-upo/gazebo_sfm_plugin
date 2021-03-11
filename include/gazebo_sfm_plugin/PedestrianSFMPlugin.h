/***********************************************************************/
/**                                                                    */
/** PedestrianSFMPlugin.h                                              */
/**                                                                    */
/** Copyright (c) 2021, Service Robotics Lab (SRL).                    */
/**                     http://robotics.upo.es                         */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** Noé Pérez-Higueras (maintainer)                                    */
/** email: noeperez@upo.es                                             */
/**                                                                    */
/** This software may be modified and distributed under the terms      */
/** of the BSD license. See the LICENSE file for details.              */
/**                                                                    */
/** http://www.opensource.org/licenses/BSD-3-Clause                    */
/**                                                                    */
/***********************************************************************/

#ifndef GAZEBO_PLUGINS_PEDESTRIANSFMPLUGIN_HH_
#define GAZEBO_PLUGINS_PEDESTRIANSFMPLUGIN_HH_

// C++
#include <string>
#include <vector>
#include <algorithm>

// Gazebo
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"

// Social Force Model
#include <lightsfm/sfm.hpp>

namespace gazebo
{
class GZ_PLUGIN_VISIBLE PedestrianSFMPlugin : public ModelPlugin
{
  /// \brief Constructor
public:
  PedestrianSFMPlugin();

  /// \brief Load the actor plugin.
  /// \param[in] _model Pointer to the parent model.
  /// \param[in] _sdf Pointer to the plugin's SDF elements.
public:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  // Documentation Inherited.
public:
  virtual void Reset();

  /// \brief Function that is called every update cycle.
  /// \param[in] _info Timing information
private:
  void OnUpdate(const common::UpdateInfo &_info);

  /// \brief Helper function to choose a new target location
  // private: void ChooseNewTarget();

  // private: void InitializePedestrians();

  /// \brief Helper function to avoid obstacles. This implements a very
  /// simple vector-field algorithm.
  /// \param[in] _pos Direction vector that should be adjusted according
  /// to nearby obstacles.
private:
  void HandleObstacles();

  // private: void HandleGroup();

private:
  void HandlePedestrians();

  // private: ignition::math::Vector3d ComputeForce();

private:
  sfm::Agent sfmActor;

private:
  std::vector<std::string> groupNames;

private:
  std::vector<sfm::Agent> myGroup;

private:
  std::vector<sfm::Agent> otherActors;

private:
  double peopleDistance;

  /// \brief ordered list of goal locations
  // private: std::queue<ignition::math::Vector3d> goals;

  /// \brief Current target location
  // private: ignition::math::Vector3d currentGoalIndex;
  // ignition::math::Vector3d currentGoal;

  // private: bool cyclicTrajectory;


  //

  //-------------------------------------------------

  /// \brief Pointer to the parent actor.
private:
  physics::ActorPtr actor;

  /// \brief Pointer to the world, for convenience.
private:
  physics::WorldPtr world;

  /// \brief Pointer to the sdf element.
private:
  sdf::ElementPtr sdf;

  /// \brief Velocity of the actor
private:
  ignition::math::Vector3d velocity;

  /// \brief List of connections
private:
  std::vector<event::ConnectionPtr> connections;

  /// \brief Time scaling factor. Used to coordinate translational motion
  /// with the actor's walking animation.
private:
  double animationFactor = 1.0;

  /// \brief Time of the last update.
private:
  common::Time lastUpdate;

  /// \brief List of models to ignore. Used for vector field
private:
  std::vector<std::string> ignoreModels;

  /// \brief Custom trajectory info.
private:
  physics::TrajectoryInfoPtr trajectoryInfo;
};
}
#endif
