#! /usr/bin/perl -w

#  Georgia Tech, Humanoid Robotics Lab
#
#  Name: setup_MANCV_pipeline.pl
#  Overview: 
#  	This script will setup the Vision & Manipulation Pipeline. Please be aware that it #	must be run from the Observer Computer and that it will open 5 terminal 
#       windows.For some cases, you might need to input your User password for the machines #	it's login into.
#
#  Dependencies: Perl
#
#  Example Usage: ./setup_MANCV_pipeline.pl your_username
#
#  Created by: Juan Carlos Garcia on 5/9/13.
#  Modified by: Juan Carlos Garcia on 7/12/2013

use strict;
use warnings;
use POSIX;

# specify the motion_pc & vision_pc DNS here
my $motion_pc = "hubo-linux";
my $vision_pc = "hubo-vision";

# username must be passed as first argument
my $user = $ARGV[0];

# Note: the only ways I know to send ssh the password are (1) sshpass and (2) expect. Both would required additional package installations. Although it might be tedious to input your password for most of opened terminal, this will allow you to keep track of what the app is doing
#my $password = $ARGV[1];

# ***************** Setup of motion_pc ******************** #
# 1. ssh into motion-pc, setup hubo-manip ach channel & start manipulation demo (opens 1 terminal window)
my $first_command = "ssh -t ".$user."@".$motion_pc." sudo service hubo-motion ach; sudo ach -1 -C \"hubo-manip\" -m 10 -n 3000 -o 666; ~/juanc/hubo-manipulation/bin/hubo-manipulation; bash";
# execute first command list on 1 terminal
system('gnome-terminal', '-t', "motion pc", '-x', 'bash' ,'-c', $first_command);

# ***************** Setup of vision_pc ******************** #
# 2. ssh into vision_pc & start forwarding agent (opens 1 terminal window)
my $second_command = "ssh -t ".$user."@".$vision_pc." sudo ./fuerte_workspace/sandbox/HuboApplication/scripts/setup_ach_forwarding.bash; bash";
# execute second command on 1 terminal
system('gnome-terminal', '-t', "vision pc (1)", '-x', 'bash', '-c', $second_command);

# 3. ssh into vision_pc & launch ROS App for Hubo Vision (opens 1 terminal window)
my $third_command = "ssh -t ".$user."@".$vision_pc." roslaunch HuboApplication embedded_hubo.launch; bash";
# execute third command on 1 terminal
system('gnome-terminal', '-t', "vision pc (2)", '-x', 'bash', '-c', $third_command);

# ***************** Setup Observer Computer ****************** #
# setup appropriate environment variable
system("export ROS_MASTER_URI=http://hubo-vision:11311");

# start ROS Observer app (opens 1 terminal window)
my $fourth_command = "roslaunch HuboApplication observe.launch";
# execute fourth command on 1 terminal
system('gnome-terminal', '-t', "observer pc (1)", '-x', 'bash', '-c', $fourth_command);

# start object recognition ROS App (opens 1 terminal window)
my $fifth_command = "rosrun HuboApplication recognise_table_objects";
# execute fifth command on 1 terminal
system('gnome-terminal', '-t', "observer pc (2)", '-x', 'bash', '-c', $fifth_command);

# Finally, start ROS Activity Coordination app on current terminal
system("rosrun HuboApplication ros_activity_coordinator");


printf("\t\tSetup Finished; and by now you must have also finished using all of the apps! \n
	\t\tThanks!\n");
