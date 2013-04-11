#include "Hubo_Control.h"
#include <fstream>
#include <iterator>
#include <vector>

#define OPEN_HAND true
#define CLOSE_HAND false

using namespace std;

/// Pre-calculated finger joint limits to avoid self-destructing collision!
/*------------------------
-0.398068
-0.660379
-0.864015
-0.398068
-0.660379
--------------------------*/
/// Pre-calculated transformation matrix for target location; table with foam
/*
0.924661  -0.368975  0.0941248   0.421227
  0.371466   0.928395 -0.0098302  -0.169458
-0.0837579  0.0440537   0.995512  -0.142446
0          0          0          1*/

int countFileLines(const char* filename);
void loadTrajectoryInfo(const char* filename, vector <Vector6d, Eigen::aligned_allocator<Vector6d> > &trajectory, int count);
void getFingersEncValues(Hubo_Control &hubo, Eigen::VectorXd &dof, Eigen::VectorXd &values);
Eigen::VectorXd proportionalGraspController(double gain, Eigen::VectorXd &vals, Eigen::VectorXd desired);
void openCloseHand(Eigen::VectorXd &torques, Eigen::VectorXd &dofs, Hubo_Control &hubo, bool action);


int main( int argc, char **argv ) {
  assert(argc > 1);
  
  Hubo_Control hubo;
  Eigen::VectorXd dofs(5); dofs << RF1, RF2, RF3, RF4, RF5;
  Eigen::VectorXd vals(5);
  Eigen::VectorXd torques(5);
  Eigen::VectorXd desiredGrasp(5); desiredGrasp << -0.39, -0.66, -0.86, -0.39, -0.66;
  double sum;

  double ptime, dt;
  int step;
  double k = 6;
  double tolerance = 0.01;
  // transformation matrix for reaching table with foam
  Eigen::Isometry3d trans;
  trans(0,0) = 0.924661; trans(0,1) = -0.368975;  trans(0,2) = 0.0941248; trans(0,3) = 0.421227;
  trans(1,0) = 0.371466; trans(1,1) =  0.928395;  trans(1,2) = -0.0098302; trans(1,3) = -0.169458;
  trans(2,0) = -0.0837579; trans(2,1) = 0.0440537; trans(2,2) =  0.995512; trans(2,3) = -0.142446;
  trans(3,0) = 0; trans(3,1) = 0; trans(3,2) = 0; trans(3,3) = 1;
  
  Eigen::Isometry3d cTrans;
  Vector6d armAngles;
  Vector6d current;  
  Eigen::VectorXd desiredLoc(3); desiredLoc << trans(0,3), trans(1,3), trans(2,3);
  Eigen::VectorXd loc(3);
  bool grasping = false;

  // load trajectory
  int configs = countFileLines(argv[1]);
  vector <Vector6d, Eigen::aligned_allocator<Vector6d> > trajectory(configs);
  loadTrajectoryInfo(argv[1], trajectory, configs);
  
  // first open hand
  openCloseHand(torques, dofs, hubo, OPEN_HAND);
  hubo.sendControls();

  while(!daemon_sig_quit){
    hubo.update();
    dt = hubo.getTime() - ptime;

    hubo.getRightArmAngles(current);
    hubo.huboArmIK(armAngles, trans, current, RIGHT);
    
    hubo.huboArmFK(cTrans, current, RIGHT);
    loc = cTrans.translation();
    
    //only do processing if new info has arrived
    if(dt > 0){
      // compute desired encoded values
      getFingersEncValues(hubo, dofs, vals);

      // go to next point in trajectory if reached current
      step = ((current - trajectory[step]).norm() <= tolerance) ? step+1 : step;
      step = (step >= configs) ? 0 : step;

      hubo.setArmAngles(RIGHT, trajectory[step]);
      
      // once object has been reached, close hand; calculate torques as object is held
      if(step > 29 || grasping){//((desiredLoc - loc).norm() <= tolerance || grasping){
	// compute grasp torques                                                                                                       
	torques = proportionalGraspController(k, vals, desiredGrasp);
	cout << torques.transpose() << endl;

	grasping = true;
	openCloseHand(torques, dofs, hubo, CLOSE_HAND);
      }
      hubo.sendControls();
    }
  }
}

void getFingersEncValues(Hubo_Control &hubo, Eigen::VectorXd &dof, Eigen::VectorXd& values){  
  for(int i = 0; i < dof.size(); i++)
     values(i) = hubo.getJointAngleState(dof(i));
}

Eigen::VectorXd proportionalGraspController(double gain, Eigen::VectorXd &vals, Eigen::VectorXd desired){
  return -gain*(desired - vals);
}

void openCloseHand(Eigen::VectorXd &torques, Eigen::VectorXd &dofs, Hubo_Control &hubo, bool action){
  for(int i=0; i < dofs.size(); i++){
    // set torques to -0.6 if action = OPEN_HAND = true
    torques(i) = (action) ? -0.6: torques(i);
    // apply limits before sending values
    torques(i) = (torques(i) > 1) ? 1 : (torques(i) < -1) ? -1 : torques(i);
    hubo.passJointAngle(dofs(i), torques(i));
  }
}

int countFileLines(const char* filename){
  ifstream file(filename);
  int count = 0;
  string line;
  if(file.is_open()){
    while(!file.eof()){
      getline(file, line, '\n');
      count++;
    }
  }
  file.close();
  return count;
}

void loadTrajectoryInfo(const char* filename, vector<Vector6d, Eigen::aligned_allocator<Vector6d> > &trajectory, int line_count){
  ifstream trajectoryFile(filename);
  string jointValues;
  int i = 0;
  int ret = 0;

  //cout << line_count << endl;
  if(trajectoryFile.is_open()){
    while( i+1 < line_count){
      getline(trajectoryFile, jointValues);
      ret = sscanf(jointValues.c_str(), "%lf %lf %lf %lf %lf %lf", &trajectory[i][0], &trajectory[i][1], &trajectory[i][2], &trajectory[i][3], &trajectory[i][4], &trajectory[i][5]);
      //cout << jointValues << endl;
      cout << trajectory[i][0] << " " << trajectory[i][1] << " " << trajectory[i][2] << " " << trajectory[i][3] << " " << trajectory[i][4] << " " << trajectory[i][5] << endl;
      assert(ret == 6);
      i++;
    }
    trajectoryFile.close();
  }
  else{
    cout << "Error opening trajectory file!" << endl;
  }
}
