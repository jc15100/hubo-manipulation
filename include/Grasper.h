/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * 
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/** @file Grasper.h
 *  @author Juan C. Garcia
 */
#include <Hubo_Control.h>
#include <fstream>
#include <iterator>
#include <vector>

#ifndef GRASPER_H
#define GRASPER_H

#define OPEN_HAND true
#define CLOSE_HAND false

// Useful Macros for Printing
#define PRINT(x) std::cout << #x << " = " << x << std::endl;
#define ECHO(x) std::cout << x << std::endl;

class Grasper{
 public:
  Grasper (Eigen::VectorXd handDofs, double tolerance, double gain);
  virtual ~Grasper();
  
  /// Functions to handle Hubo's manipulation tasks
  void tryGrasping(Hubo_Control &hubo, Eigen::Isometry3d &objectTrans, Eigen::Vector3d objectGCP, int arm);  

  /// Functions to handle trajectories from DART
  int countFileLines(const char* filename);
  void loadTrajectoryInfo(const char* filename, std::vector <Vector6d, Eigen::aligned_allocator<Vector6d> > &trajectory, int count);
  
private:
  void getFingersEncValues(Hubo_Control &hubo, Eigen::VectorXd &dof, Eigen::VectorXd &values);
  Eigen::VectorXd proportionalGraspController(double gain, Eigen::VectorXd &vals);
  void openCloseHand(Eigen::VectorXd &torques, Eigen::VectorXd &dofs, Hubo_Control &hubo, bool action);
  
  Eigen::Vector3d robotGCP;
  Vector6d currentArmConfig, targetArmConfig;
  Eigen::Isometry3d currentEndEffectorTrans;
  Eigen::VectorXd handDofs, fingerEncoderValues, torques, desiredEncoderValues;
  bool graspingNow;
  double tolerance, gain;
};

#endif
