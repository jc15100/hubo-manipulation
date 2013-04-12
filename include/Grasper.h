#include "Hubo_Control.h"
#include <fstream>
#include <iterator>
#include <vector>

#ifndef GRASPER_H
#define GRASPER_H

class Grasper{
 public:
  Grasper();
  virtual ~Grasper();
  
  int countFileLines(const char* filename);

  void loadTrajectoryInfo(const char* filename, std::vector <Vector6d, Eigen::aligned_allocator<Vector6d> > &trajectory, int count);
  
  void getFingersEncValues(Hubo_Control &hubo, Eigen::VectorXd &dof, Eigen::VectorXd &values);
  
  Eigen::VectorXd proportionalGraspController(double gain, Eigen::VectorXd &vals, Eigen::VectorXd desired);
  
  void openCloseHand(Eigen::VectorXd &torques, Eigen::VectorXd &dofs, Hubo_Control &hubo, bool action);
  
};

#endif
