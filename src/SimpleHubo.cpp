#include <Hubo_Control.h>
#include "manipulation_instruction_t.h"
#include "Grasper.h"
using namespace std;

int main(int argc, char** argv) {
    Hubo_Control hubo;
    Grasper hubo_grasp;

    // Dofs of Right hand
    Eigen::VectorXd handDofs(5);
    handDofs << RF1, RF2, RF3, RF4, RF5;
    Eigen::VectorXd torques(5);
    Eigen::VectorXd vals(5);

    // Desired encoder values for a firm grasp
    Eigen::VectorXd desiredGrasp(5); desiredGrasp << -0.39, -0.66, -0.86, -0.39, -0.66;

    // 4, 0, 9 - grasping point in bottle's frame
    Eigen::Vector3d graspingPoint, trueGraspingPoint;
    graspingPoint << 0.04, 0.0, 0.09;
    Vector6d current, target;

    Eigen::Isometry3d objectTrans = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d trans = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d endffTrans;

    bool grasping = false;
    double tolerance = 0.01;

    double k, dt, ptime, currentAngle;
    manipulation_instruction_t cmd;

    // Setup ach for vision_computer command
    ach_status r = ACH_OK;
    ach_channel_t manip_chan;

    r = ach_open(&manip_chan, "hubo-manip", NULL);
    if (r != ACH_OK) {
        fprintf(stderr, "\nUnable to open fastrak channel '%s', error: (%d) %s\n",
                "hubo-manip", r, ach_result_to_string((ach_status_t) r));
        return -1;
    } else {
        fprintf(stderr, "Connected to ACH!\n");
        ach_flush(&manip_chan);
    }

    // first open hand & rotate neck
    hubo_grasp.openCloseHand(torques, handDofs, hubo, true);
    hubo.sendControls();

    while (!daemon_sig_quit) {
        hubo.update();
        dt = hubo.getTime() - ptime;
        ptime = hubo.getTime();

        if (dt > 0) {
            r = ACH_OK;
            size_t fs;
            r = ach_get(&manip_chan, &cmd, sizeof (cmd), &fs, NULL, ACH_O_WAIT);
            if(r == ACH_STALE_FRAMES){
                //fprintf(stderr, "%s\n", ach_result_to_string(r));
                usleep(100000);
                continue;
            }
            else if(r == ACH_OK){
                fprintf(stderr, "New Command: %f\n", cmd.targetJoints.data[REB]);
            }
            else if(r == ACH_MISSED_FRAME){
                fprintf(stderr, "Missed Command: %f\n", cmd.targetJoints.data[REB]);
            }
            else{
                fprintf(stderr, "%s\n", ach_result_to_string(r));
            }

            if(cmd.controlMode == JOINT_VECTOR){
                for (int i = 0; i <= RWP; i++) {
                    if(cmd.targetJoints.data[i] != cmd.targetJoints.data[i]){
                        fprintf(stderr, "NaN: %f", cmd.targetJoints.data[i]);
                        continue;
                    }
                    if(cmd.incrementalMode){
                        currentAngle = hubo.getJointAngle(i);
                        //hubo.setJointAngle(i, currentAngle + cmd.targetJoints.data[i]);
                    }
                    else{
                        //if (i == REB)
                        {
                            hubo.setJointAngle(i, cmd.targetJoints.data[i]);
                            fprintf(stderr, "Sent Command: %f\n", cmd.targetJoints.data[i]);
                        }
                    }
                }
            }
            else if(cmd.controlMode == END_EFFECTOR && cmd.poseMode == QUATERNION){
                Vector6d qLTarget, qRTarget, qLCurrent, qRCurrent;
                hubo.getArmAngles(LEFT, qLCurrent);
                //hubo.getArmAngles(RIGHT, qRCurrent);
                //std::cout << Eigen::Vector3d(cmd.targetPoseRight.x,cmd.targetPoseRight.y,cmd.targetPoseRight.z).matrix() << std::endl;

                //hubo.huboArmIK(qLTarget, cmd.targetPoseLeft, qLCurrent, LEFT);
                Eigen::Isometry3d targetPoseLeft = Eigen::Isometry3d::Identity();
                targetPoseLeft.translate(Eigen::Vector3d(
                        cmd.targetPoseLeft.x, cmd.targetPoseLeft.y, cmd.targetPoseLeft.z));
                targetPoseLeft.rotate(Eigen::Quaterniond(cmd.targetPoseLeft.w,
                        cmd.targetPoseLeft.i, cmd.targetPoseLeft.j, cmd.targetPoseLeft.k));

                Eigen::Isometry3d fk;
                Eigen::Isometry3d cb;
                double ld = 300 / 1000;
                double ad = -M_PI / 4;

                cb(0, 0) = cos(ad); cb(0, 1) = 0; cb(0, 2) = sin(ad); cb(0, 3) = ld * cos(ad);
                cb(1, 0) = 0; cb(1, 1) = 1; cb(1, 2) = 0; cb(1, 3) = 0;
                cb(2, 0) = -sin(ad); cb(2, 1) = 0; cb(2, 2) = cos(ad); cb(2, 3) = -ld * sin(ad);
                cb(3, 0) = 0; cb(3, 1) = 0; cb(3, 2) = 0; cb(3, 3) = 1;
                cb.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()));

                //std::cout << targetPoseRight.matrix() << std::endl;
                hubo.huboArmIK(qLTarget, targetPoseLeft, qLCurrent, LEFT, cb);

                // Display for diagnostics
                std::cout << targetPoseLeft.matrix() << std::endl;
                std::cout << "q: " << qLTarget.matrix().transpose() << std::endl;

                hubo.huboArmFK(fk, qLTarget, LEFT, cb);

                std::cout << fk.matrix() << std::endl;

                hubo.setArmAngles(LEFT, qLTarget, false);
                //hubo.setArmAngles(RIGHT, qRTarget, false);
            }
            else if(cmd.controlMode == OBJECT_POSE){
                cout << "Received OBJECT_POSE !" << endl;
                objectTrans.translate(Eigen::Vector3d(cmd.targetPoseRight.x, cmd.targetPoseRight.y, cmd.targetPoseRight.z));
                objectTrans.rotate(Eigen::Quaterniond(cmd.targetPoseRight.w, cmd.targetPoseRight.i, cmd.targetPoseRight.j, cmd.targetPoseRight.k));

                // compute grasping point in Hubo's frame
                trueGraspingPoint = objectTrans * graspingPoint;
                cout << "grasping point: " << trueGraspingPoint.transpose() << endl;

                // translate objectTrans by grasping point in robot's frame
                objectTrans.translate(Eigen::Vector3d(trueGraspingPoint(0), trueGraspingPoint(1), trueGraspingPoint(2)));
               
                while (true) {
                    hubo.update();
                    
                    // get current joint config
                    hubo.getArmAngles(RIGHT, current);
                    
                    cout << "Joint angles: " << current.transpose() << endl;
                    
                    // get joing angles for target grasp
                    hubo.huboArmIK(target, objectTrans, current, RIGHT);
                    
                     // set arm to desired joint angles
                    hubo.setArmAngles(RIGHT, target);
                    
                    // compute current end-effector location
                    hubo.huboArmFK(endffTrans, current, RIGHT);
                    
                    // print norm
                    cout << "norm: " << (trueGraspingPoint - endffTrans.translation()).norm() << endl;
                    cout << trueGraspingPoint.transpose() << endl;
                    cout << endffTrans.translation().transpose() << endl;
                    
                    // compute current encoded values
                    hubo_grasp.getFingersEncValues(hubo, handDofs, vals);
                    
                    if ((trueGraspingPoint - endffTrans.translation()).norm() <= tolerance || grasping) {
                        cout << "Object reached: Grasping!" << endl;
                        grasping = true;
                        // compute grasp torques
                        torques = hubo_grasp.proportionalGraspController(k, vals, desiredGrasp);
                        cout << torques.transpose() << endl;
                        hubo_grasp.openCloseHand(torques, handDofs, hubo, false);
                    }
                    hubo.sendControls();
                }   
            }
            else if(cmd.controlMode == HOME_JOINTS){
                for(int i = 0; i <= RWP; i++){
                    if(cmd.targetJoints.data[i] != cmd.targetJoints.data[i] ||
                            cmd.targetJoints.data[i] == 0.0){
                        // ignore NaN and 0
                        continue;
                    }
                    else{
                        //hubo.homeJoint(i,false);
                        //fprintf(stderr, "Homing joint %i.\n", i);
                    }
                }
            }
            hubo.sendControls();
        }
        //usleep(100);
    }
}