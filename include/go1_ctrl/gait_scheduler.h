#pragma once
#include "go1_ctrl/types.h"
#include <array>


namespace go1 {
struct GaitParameters {
  double period;
  // touchdown offsets: phase_i = wrap(cycle - offset_i).
  std::array<double, 4> offset, duty;
};
struct WalkFeedback {
  // all vectors in the same gravity-aligned local/world frame (meters)
  std::array<Vec3, 4> feet;
  Vec3 com = Vec3::Zero();
  Mat3 world_R_body = Mat3::Identity();
  std::array<bool, 4> contact{{false,false,false,false}};
  double stamp = -1;
};
class GaitScheduler {
	
 public:
  GaitScheduler();
  // explicit restart (legacy main interface)
  // runtime changes use request()
  
  void start(GaitType g, double t); // initialize
  // EXAMPLE: scheduler.start(GraitType::TROT, s.t);
  
  GaitState update(double t); // enter gait mode
  void configure(GaitType g, const GaitParameters& p); // configuration and runtime gait requests
  void request(GaitType g); // request runetime gait change
  void setWalkFeedback(const WalkFeedback& f) { feedback_ = f; } 
  // default true: missing/stale feedback holds Walk before its first lift
  
  void setGroundWalk(bool enabled) { ground_walk_ = enabled; }
  void setTransitionTime(double seconds); // gait transition
  GaitType activeGait() const { return gait_; }
  bool transitioning() const { return pending_ || stage_ != RUN; }
  bool walkBlocked() const { return blocked_; }
  const GaitParameters& parameters() const { return current_; }
  static GaitParameters defaults(GaitType g);
  
 private:
  enum Stage { RUN, DRAIN, BLEND }; 
	  // private transition stages:
	  // RUN: execute gait normally 
	  // DRAIN: suppress new lifts, let existing swings finish
	  // BLEND: hold all planned contacts, smoothly changing parameters
	  
  std::array<GaitParameters,4> table_; // stored param set
  GaitParameters current_, from_, target_; 
		// current_: param currently used
		// from_: param captured at beginning of blending
		// target_: param of requested gait
		
  GaitType gait_ = GaitType::TROT, requested_ = GaitType::TROT;
  Stage stage_ = RUN;
  
  // timing vars
  double cycle_ = 0, // accumilated cycle ct
		 last_ = 0,  // time processed by scheduler
		 blend_start_ = 0, // time when blending began
		 transition_time_ = 0.6; // blending duration

  bool started_ = false, pending_ = false, ground_walk_ = true, blocked_ = false; // state flags
  std::array<bool,4> held_{{false,false,false,false}}; // per-leg holds 
  WalkFeedback feedback_; // stored feedback/CoM ref
  Vec3 shift_ = Vec3::Zero(); // stored desire CoM displacement (body frame)
  
  void step(double dt, double now); // advance internal state by small time interval
  
  GaitState sample() const; // in substeps to reduce chance of skipping timing events
  
  bool prepareWalk(double next, double now); // prep support ref and decide if it may advance
};
} // namespace go1
