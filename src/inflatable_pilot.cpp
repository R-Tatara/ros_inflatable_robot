/*++

Name : inflatable_vicon.cpp

Abstract : Sequential control pilot for an inflatable robot

Author : Ryosuke Tatara

--*/

#include "ros/ros.h"
#include <iomanip>
#include <termios.h>
//Publishing
#include "inflatable_robot/InflatableState.h"
//Subscribing
#include "inflatable_robot/InflatableStateFlag.h"

#define INIT_IOBOARD_TIME 0.5
#define LINK_PRESSURIZATION_TIME 12.0
#define ACTUATOR_PRESSURIZATION_TIME 15.0
#define GRASPING_TIME 7.0
#define RELEASING_TIME 5.0
#define SAMPLING_FREQUENCY 100


class Time {
public:
    ros::Time now_t;         //Current time
    ros::Duration now_d;     //Current duration time
    ros::Time base;          //Time for current time
    ros::Time lapbase;       //Time when state change
    ros::Time buf;           //Time for sampling time
    ros::Duration sampling;  //Sampling time
    Time();
    ~Time();
};

Time::Time() {
}

Time::~Time() {
}

Time timer;

//Global variable
int cycle = 0;                 //The number of cycle of while loop in main()
int robot_state[2] = { 0, 1 }; //[0] : Higher order state, [1] : Lower order state
int robot_state_flag = 0;      //1 : Next state request by robotic arm

//Prototype declaration
int NextState(void);
char getch(void);

//Subscribe InflatableStateFlag.msg
void msgCallback(const inflatable_robot::InflatableStateFlag::ConstPtr& sub_msg) {
    robot_state_flag = sub_msg->inflatable_state_flag;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "inflatable_pilot");
    ros::NodeHandle nh;
    ros::Publisher ros_pub = nh.advertise<inflatable_robot::InflatableState>("inflatable_state", 10);
    ros::Subscriber ros_sub = nh.subscribe("inflatable_state_flag", 10, msgCallback);
    ros::Rate loop_rate(SAMPLING_FREQUENCY);
    inflatable_robot::InflatableState pub_msg;
    timer.base = ros::Time::now();

    while (ros::ok()) {
        //Time processing
        timer.now_t = ros::Time::now();
        timer.now_d = timer.now_t - timer.base;
        timer.sampling = timer.now_t - timer.buf;
		timer.buf = timer.now_t;
        ros::spinOnce();
        int cycle_break;

        //Keyboard processing
        int key = 0;
        key = getch();
        if (key == 110) { //n
            ROS_INFO("n key is pressed to transit to next state.");
            if ( (robot_state[0] == 0 && robot_state[1] == 1) || 
                 (robot_state[0] == 0 && robot_state[1] == 2) || 
                 (robot_state[0] == 2 && robot_state[1] == 1) || 
                 (robot_state[0] == 2 && robot_state[1] == 2) || 
                 (robot_state[0] == 6 && robot_state[1] == 1) ) {
                robot_state[1] += 1;
                robot_state_flag = 0;
                timer.lapbase = ros::Time::now();
            }
            else if ( (robot_state[0] == 1 && robot_state[1] == 1) ||
                      (robot_state[0] == 2 && robot_state[1] == 3) ||
                      (robot_state[0] == 5 && robot_state[1] == 1) ||
                      (robot_state[0] == 6 && robot_state[1] == 2) ) {
                NextState();
            }
            else if (robot_state[0] == 7) {
                robot_state[0] = 9;
                robot_state[1] = 1;
            }
            else {
                NextState();
            }
        }
        else if (key == 27) { //Esc
            ROS_INFO("Esc key is pressed.");
            robot_state[0] = 9;
            robot_state[1] = 1;
        }

        //Robot sequential control
        //Initialization
        if (robot_state[0] == 0) {
            if (robot_state[1] == 1) {
                //IO board initialization
                if (timer.now_t - timer.lapbase > ros::Duration(INIT_IOBOARD_TIME)) {
                    robot_state[1] += 1;
                    robot_state_flag = 0;
                    timer.lapbase = ros::Time::now();
                }
            }
            else if (robot_state[1] == 2) {
                //Link pressurization
                if (timer.now_t - timer.lapbase > ros::Duration(LINK_PRESSURIZATION_TIME)) {
                    robot_state[1] += 1;
                    robot_state_flag = 0;
                    timer.lapbase = ros::Time::now();
                }
            }
            else if (robot_state[1] == 3) {
                //Actuator pressurization
                if (timer.now_t - timer.lapbase > ros::Duration(ACTUATOR_PRESSURIZATION_TIME)) {
                    ROS_INFO("Push start key.");
                    while (ros::ok()) {
                        int key;
                        key = getch();

                        if (key == 32) { //Space
                            ROS_INFO("Space key is pressed to start task.");
                            NextState();
                            break;
                        }
                        else if (key == 27) { //Esc
                            ROS_INFO("Esc key is pressed.");
                            robot_state[0] = 9;
                            break;
                        }
                    }
                }
            }
        }

        //Move to table by chairbot
        else if (robot_state[0] == 1) {
            if (robot_state[1] == 1) {
                if (robot_state_flag == 1) {
                    NextState();
                }
            }
        }

        //Arm control
        else if (robot_state[0] == 2) {
            //Initialization of init_q
            if (robot_state[1] == 1) {
                if (timer.now_t - timer.lapbase > ros::Duration(1.0)) {
                    robot_state[1] += 1;
                    robot_state_flag = 0;
                    timer.lapbase = ros::Time::now();
                }
            }
            //Move above the target
            else if (robot_state[1] == 2) {
                if (robot_state_flag == 1) {
                    robot_state[1] += 1;
                    robot_state_flag = 0;
                    timer.lapbase = ros::Time::now();
                }
            }
            //Move to the target
            else if (robot_state[1] == 3) {
                if (robot_state_flag == 1) {
                    NextState();
                }
            }
        }

        //Gripper grasping
        else if (robot_state[0] == 3) {
            if (robot_state[1] == 1) {
                if (timer.now_t - timer.lapbase > ros::Duration(GRASPING_TIME)) {
                    NextState();
                }
            }
        }

        //Arm lifting
        else if (robot_state[0] == 4) {
            if (robot_state[1] == 1) {
                if (robot_state_flag == 1) {
                    NextState();
                }
            }
        }

        //Move to bed by chairbot
        else if (robot_state[0] == 5) {
            if (robot_state[1] == 1) {
                if (robot_state_flag == 1) {
                    NextState();
                }
            }
        }

        //Arm control to human
        else if (robot_state[0] == 6) {
            //Initialize init_q
            if (robot_state[1] == 1) {
                if (timer.now_t - timer.lapbase > ros::Duration(1.0)) {
                    robot_state[1] += 1;
                    robot_state_flag = 0;
                }
            }
            //Move to the target
            else if (robot_state[1] == 2) {
                if (robot_state_flag == 1) {
                    NextState();
                }
            }
        }

        //Gripper releasing
        else if (robot_state[0] == 7) {
            if (robot_state[1] == 1) {
                if (timer.now_t - timer.lapbase > ros::Duration(RELEASING_TIME)) {
                    robot_state[0] = 9;
                }
            }
        }

        //Publish InflatableState.msg
        pub_msg.inflatable_state = 100 * robot_state[0] + robot_state[1];
        if (cycle % SAMPLING_FREQUENCY == 0) {
            ROS_INFO("Robot state : %d - %d", robot_state[0],robot_state[1]);
        }
        ros_pub.publish(pub_msg);
        loop_rate.sleep();
        cycle++;

        //Termination processing
        if (robot_state[0] == 9) {
            pub_msg.inflatable_state = 100 * robot_state[0] + robot_state[1];
            if (cycle % SAMPLING_FREQUENCY == 0) {
                ROS_INFO("Robot state : %d - %d", robot_state[0],robot_state[1]);
            }
            ros_pub.publish(pub_msg);
            if (cycle_break > 500) {
                ROS_INFO("--- Termination processing ---");
                break;
            }
            cycle_break++;
        }
    }

    return 0;
}

//Change to next state
int NextState() {
    robot_state[0] += 1;
    robot_state[1] = 1;
    robot_state_flag = 0;
    timer.lapbase = ros::Time::now();
    return 0;
}

//Keyboard non blocking input
char getch() {
    fd_set set;
    struct timeval timeout;
    int rv;
    char buff = 0;
    int len = 1;
    int filedesc = 0;
    FD_ZERO(&set);
    FD_SET(filedesc, &set);

    timeout.tv_sec = 0;
    timeout.tv_usec = 1000;

    rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

    struct termios old = {0};
    if (tcgetattr(filedesc, &old) < 0) {
        ROS_ERROR("tcsetattr()");
    }
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(filedesc, TCSANOW, &old) < 0) {
        ROS_ERROR("tcsetattr ICANON");
    }
    if(rv == -1) {
        ROS_ERROR("select");
    }
    else if(rv == 0) {
        //ROS_INFO("no_key_pressed");
    }
    else {
        read(filedesc, &buff, len );
    }

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(filedesc, TCSADRAIN, &old) < 0) {
        ROS_ERROR ("tcsetattr ~ICANON");
    }
    return (buff);
}