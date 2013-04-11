#include <iostream>
#include <Hubo_Control.h>
#include "manipulation_instruction_t.h"

int main(int argc, char** argv)
{
    Hubo_Control hubo;
    double dt, ptime, currentAngle;
    manipulation_instruction_t cmd;

    // Setup ach
    ach_status r = ACH_OK;
    ach_channel_t manip_chan;

    r = ach_open(&manip_chan, "hubo-manip", NULL);
    if ( r != ACH_OK)
    {
        fprintf(stderr, "\nUnable to open fastrak channel '%s', error: (%d) %s\n",
            "hubo-manip", r, ach_result_to_string((ach_status_t)r));
        return -1;
    }
    else
    {
        fprintf(stderr, "Connected to ACH!\n");
        ach_flush(&manip_chan);
    }
 
    while (true)
    {
        hubo.update();
        dt = hubo.getTime() - ptime;
        ptime = hubo.getTime();
        
        //if(dt > 0)
        {
            r = ACH_OK;
            size_t fs;
            r = ach_get(&manip_chan, &cmd, sizeof(cmd), &fs, NULL, ACH_O_WAIT);
            if (r == ACH_STALE_FRAMES)
            {
                //fprintf(stderr, "%s\n", ach_result_to_string(r));
                usleep(100000);
                continue;
            }
            else if (r == ACH_OK)
            {
                fprintf(stderr, "New Command: %f\n", cmd.targetJoints.data[REB]);
            }
            else if (r == ACH_MISSED_FRAME)
            {
                fprintf(stderr, "Missed Command: %f\n", cmd.targetJoints.data[REB]);
            }
            else
            {
                fprintf(stderr, "%s\n", ach_result_to_string(r));
            }

            if (cmd.controlMode == JOINT_VECTOR)
            { 
                for (int i = 0; i <= RWP; i++)
                {
                    if (cmd.targetJoints.data[i] != cmd.targetJoints.data[i])
                    {
                        fprintf(stderr, "NaN: %f", cmd.targetJoints.data[i]);
                        continue;
                    }
                    if (cmd.incrementalMode)
                    {
                        currentAngle = hubo.getJointAngle(i);
                        //hubo.setJointAngle(i, currentAngle + cmd.targetJoints.data[i]);
                    }
                    else
                    {
                        //if (i == REB)
                        {
                            hubo.setJointAngle(i, cmd.targetJoints.data[i]);
                            fprintf(stderr, "Sent Command: %f\n", cmd.targetJoints.data[i]);
                        }
                    }
                }
            }
            else if (cmd.controlMode == END_EFFECTOR && cmd.poseMode == QUATERNION)
            {
                Vector6d qLTarget, qRTarget, qLCurrent, qRCurrent;
                hubo.getArmAngles(LEFT, qLCurrent);
                //hubo.getArmAngles(RIGHT, qRCurrent);
                //std::cout << Eigen::Vector3d(cmd.targetPoseRight.x,cmd.targetPoseRight.y,cmd.targetPoseRight.z).matrix() << std::endl;

                //hubo.huboArmIK(qLTarget, cmd.targetPoseLeft, qLCurrent, LEFT);
                Eigen::Isometry3d targetPoseLeft = Eigen::Isometry3d::Identity();
                targetPoseLeft.translate( Eigen::Vector3d(
                    cmd.targetPoseLeft.x,cmd.targetPoseLeft.y,cmd.targetPoseLeft.z));
                targetPoseLeft.rotate(Eigen::Quaterniond(cmd.targetPoseLeft.w,
                    cmd.targetPoseLeft.i,cmd.targetPoseLeft.j,cmd.targetPoseLeft.k));
                
                Eigen::Isometry3d fk;
                Eigen::Isometry3d cb;
                double ld = 300/1000;
                double ad = -M_PI/4;
                cb(0,0) = cos(ad);  cb(0,1) = 0;    cb(0,2) = sin(ad);  cb(0,3) = ld*cos(ad);
                cb(1,0) = 0;        cb(1,1) = 1;    cb(1,2) = 0;        cb(1,3) = 0;
                cb(2,0) = -sin(ad); cb(2,1) = 0;    cb(2,2) = cos(ad);  cb(2,3) = -ld*sin(ad);
                cb(3,0) = 0;        cb(3,1) = 0;    cb(3,2) = 0;        cb(3,3) = 1;
                cb.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX()));
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
            else if (cmd.controlMode == HOME_JOINTS)
            {
                for (int i = 0; i <= RWP; i++)
                {
                    if (cmd.targetJoints.data[i] != cmd.targetJoints.data[i] ||
                        cmd.targetJoints.data[i] == 0.0)
                    {
                        // ignore NaN and 0
                        continue;
                    }
                    else
                    {
                        //hubo.homeJoint(i,false);
                        //fprintf(stderr, "Homing joint %i.\n", i);
                    }
                }
            }
            hubo.sendControls();
        }
        usleep(100);
    }
}

    
