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

/** @file Grasper.cpp
 *  @author Juan C. Garcia
 */
#include "Grasper.h"
using namespace std;

Grasper::Grasper(Eigen::VectorXd dofs, double t, double g) {
    desiredEncoderValues.resize(5);
    desiredEncoderValues << -0.3, -0.6, -0.8, -0.4, -0.6;
    handDofs = dofs;
    tolerance = t; 
    gain = g;
    graspingNow = false;
}

Grasper::~Grasper() {}

void Grasper::tryGrasping(Hubo_Control &hubo, Eigen::Isometry3d &objectTrans, Eigen::Vector3d objectGCP, int arm) {
    // compute grasping point in Hubo's frame
    robotGCP = objectTrans * objectGCP;
    PRINT(robotGCP.transpose()); 

    // translate objectTrans by grasping point in robot's frame
    objectTrans.translate(Eigen::Vector3d(robotGCP(0), robotGCP(1), robotGCP(2)));
    while (true) {
        hubo.update();

        // get current joint config
        hubo.getArmAngles(RIGHT, currentArmConfig);
        PRINT(currentArmConfig.transpose());
        
        // compute joing angles for target grasp & set arm to such config
        bool notReachable = hubo.huboArmIK(targetArmConfig, objectTrans, currentArmConfig, RIGHT);
        hubo.setArmAngles(RIGHT, targetArmConfig);
        PRINT(notReachable);
        
        // compute current end-effector location & compute norms to determine if target has been reached
        hubo.huboArmFK(currentEndEffectorTrans, currentArmConfig, RIGHT);
        PRINT((robotGCP - currentEndEffectorTrans.translation()).norm());         

        /* Hubo will compute and apply torques to the fingers (i.e. close hand) if any of the following 3 cases are true: 
         * (1) IK failed to find a solution, so just give it a try!
         * (2) The object has already been grasped, keep applying torques!
         * (3) The object has been reached through IK, perform initial grasp */
        fingerEncoderValues.resize(5);
        this->getFingersEncValues(hubo, handDofs, fingerEncoderValues);
        if (notReachable || graspingNow || ((robotGCP - currentEndEffectorTrans.translation()).norm() <= tolerance)) {
            ECHO("Object reached: Grasping now... ");
            graspingNow = true;
            
            // compute grasp torques & apply them
            torques = this->proportionalGraspController(gain, fingerEncoderValues);
            PRINT(torques);
            this->openCloseHand(torques, handDofs, hubo, false);
        }
        hubo.sendControls();
    }
}

void Grasper::getFingersEncValues(Hubo_Control &hubo, Eigen::VectorXd &dof, Eigen::VectorXd& values) {
    for (int i = 0; i < dof.size(); i++)
        values(i) = hubo.getJointAngleState(dof(i));
}

Eigen::VectorXd Grasper::proportionalGraspController(double gain, Eigen::VectorXd &vals) {
    return -gain * (desiredEncoderValues - vals);
}

void Grasper::openCloseHand(Eigen::VectorXd &torques, Eigen::VectorXd &dofs, Hubo_Control &hubo, bool action) {
    for (int i = 0; i < dofs.size(); i++) {
        // set torques to -0.6 if action = OPEN_HAND = true
        torques(i) = (action) ? -0.6 : torques(i);
        // apply limits before sending values
        torques(i) = (torques(i) > 1) ? 1 : (torques(i) < -1) ? -1 : torques(i);

        hubo.passJointAngle(dofs(i), torques(i));
    }
}

int Grasper::countFileLines(const char* filename) {
    ifstream file(filename);
    int count = 0;
    string line;
    if (file.is_open()) {
        while (!file.eof()) {
            getline(file, line, '\n');
            count++;
        }
    }
    file.close();
    return count;
}

void Grasper::loadTrajectoryInfo(const char* filename, vector<Vector6d, Eigen::aligned_allocator<Vector6d> > &trajectory, int line_count) {
    ifstream trajectoryFile(filename);
    string jointValues;
    int i = 0;
    int ret = 0;

    //cout << line_count << endl;
    if (trajectoryFile.is_open()) {
        while (i + 1 < line_count) {
            getline(trajectoryFile, jointValues);
            ret = sscanf(jointValues.c_str(), "%lf %lf %lf %lf %lf %lf", &trajectory[i][0], &trajectory[i][1], &trajectory[i][2], &trajectory[i][3], &trajectory[i][4], &trajectory[i][5]);
            //cout << jointValues << endl;
            cout << trajectory[i][0] << " " << trajectory[i][1] << " " << trajectory[i][2] << " " << trajectory[i][3] << " " << trajectory[i][4] << " " << trajectory[i][5] << endl;
            assert(ret == 6);
            i++;
        }
        trajectoryFile.close();
    } else {
        cout << "Error opening trajectory file!" << endl;
    }
}
