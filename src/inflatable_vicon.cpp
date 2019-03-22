/*++

Name : inflatable_vicon.cpp

Abstract : Vicon pose data for Inflatable Robot

Author : Ryosuke Tatara

--*/

#include "ros/ros.h"
#include <math.h>
#include <vector>
#include <std_msgs/Float32.h>
#include <vicon_bridge/Markers.h>
#include <vicon_bridge/Marker.h>

//Publishing
#include "inflatable_robot/InflatablePose.h"
//Subscribing
#include "inflatable_robot/InflatableState.h"

#define DEGREE_OF_FREEDOM 4
#define SAMPLING_FREQUENCY 100
#define THE_NUMBER_OF_MARKERS 16
#define HAND_FIRST_MARKER 0
#define BOTTLE_FIRST_MARKER 4
#define BASE_FIRST_MARKER 7


class Time {
public:
    ros::Time now_t;        //Current time
    ros::Duration now_d;    //Current duration time
    ros::Time base;         //Time for current time
    ros::Time buf;          //Time for sampling time
    ros::Duration sampling; //Sampling time
    Time();
    ~Time();
};

Time::Time() {
}

Time::~Time() {
}

class Direction {
public:
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> z;
    Direction();
    ~Direction();
};

Direction::Direction() {
}

Direction::~Direction() {
}

class RobotPosition : public Direction {
public:
    Direction vicon;     //Vicon raw data[m]
    Direction vicon_pre; //Vicon raw data[m]
    Direction base;      //Base position[m]
	Direction current;   //Center of grasping[m]
	Direction current2;
	Direction target;    //Target position[m]
	Direction target2;
    Direction lift;      //Target position for lifting[m]
    Direction hand1;     //Wrist position[m]
    Direction hand2;     //Handtip position[m]
    Direction tmp;
    RobotPosition();
    ~RobotPosition();
};

RobotPosition::RobotPosition() {
    this->base.x.emplace_back(0.0F);
    this->base.y.emplace_back(0.0F);
    this->base.z.emplace_back(0.0F);
    this->hand1.x.emplace_back(0.0F);
    this->hand1.y.emplace_back(0.0F);
    this->hand1.z.emplace_back(0.0F);
    this->hand2.x.emplace_back(0.0F);
    this->hand2.y.emplace_back(0.0F);
    this->hand2.z.emplace_back(0.0F);
    for (int i = 0; i < DEGREE_OF_FREEDOM; i++) {
        this->current.x.emplace_back(0.0F);
        this->current.y.emplace_back(0.0F);
        this->current.z.emplace_back(0.0F);
        this->current2.x.emplace_back(0.0F);
        this->current2.y.emplace_back(0.0F);
        this->current2.z.emplace_back(0.0F);
        this->target.x.emplace_back(0.0F);
        this->target.y.emplace_back(0.0F);
        this->target.z.emplace_back(0.0F);
        this->target2.x.emplace_back(0.0F);
        this->target2.y.emplace_back(0.0F);
        this->target2.z.emplace_back(0.0F);
        this->lift.x.emplace_back(0.0F);
        this->lift.y.emplace_back(0.0F);
        this->lift.z.emplace_back(0.0F);
        this->tmp.x.emplace_back(0.0F);
        this->tmp.y.emplace_back(0.0F);
        this->tmp.z.emplace_back(0.0F);
    }
    for (int i = 0; i < THE_NUMBER_OF_MARKERS; i++) {
        this->vicon_pre.x.emplace_back(0.0F);
        this->vicon_pre.y.emplace_back(0.0F);
        this->vicon_pre.z.emplace_back(0.0F);
    }
}

RobotPosition::~RobotPosition() {
}

class Angle {
public:
    std::vector<float> q;
    Angle();
    ~Angle();
};

Angle::Angle() {
}

Angle::~Angle() {
}

class RobotOrientation : public Angle {
public:
    Angle init;    //Yaw angle before controlling arm
    Angle offset;  //Offset angle of pedestal
    Angle limit;   //Angle limit
    Angle hand;    //Hand angle
    Angle current; //Current angle
    Angle target;  //Target angle
    RobotOrientation();
    ~RobotOrientation();
};

RobotOrientation::RobotOrientation() {
    this->init.q.emplace_back(0.0F);
    this->offset.q.emplace_back(45.0F * M_PI / 180.0F);
    this->limit.q.emplace_back(90.0F);
    this->hand.q.emplace_back(0.0F);
    for (int i = 0; i < DEGREE_OF_FREEDOM; i++) {
        this->current.q.emplace_back(0.0F);
        this->target.q.emplace_back(0.0F);
    }
}

RobotOrientation::~RobotOrientation() {
}

class SubParam{
public:
    bool reach_flag;
    float alpha;    //Auxiliary angle for kinematics
    float beta;
    float gamma;
    float delta;
    float yaw;      //Yaw angle with respect to base coordinate
    float distance; //Distance between first joint and center of grasping
    SubParam();
    ~SubParam();
};

SubParam::SubParam():
    reach_flag(false), 
    alpha(0.0F), 
    beta(0.0F), 
    gamma(0.0F), 
    delta(0.0F), 
    yaw(0.0F), 
    distance(0.0F)
{
}

SubParam::~SubParam() {
}

class RobotParameter : public SubParam{
public:
    SubParam current;
    SubParam target;
    float link[DEGREE_OF_FREEDOM];   //Link length
    float link2[DEGREE_OF_FREEDOM];  //Squared link length
    float link_u;                    //Upper first link length
    float link_l;                    //Lower first link length
    float kvp;                       //P gain of visual feedback
    float kvi;                       //I gain of visual feedback
    float kvd;                       //D gain of visual feedback
    float ktp;                       //Gain of torque-pressure conversion
    float J[3][DEGREE_OF_FREEDOM];   //Jacobian matrix
    float J_T[DEGREE_OF_FREEDOM][3]; //Transposed Jacobian matrix
    float grab_point;                //Ratio of grasping point
    RobotParameter();
    ~RobotParameter();
};

RobotParameter::RobotParameter():
    link{ 0.35F, 0.28F, 0.28F, 0.25F }, 
    link2{ this->link[0] * this->link[0], this->link[1] * this->link[1], this->link[2] * this->link[2], this->link[3] * this->link[3] }, 
    link_u{ 0.6F * this->link[0] }, 
    link_l{ 0.4F * this->link[0] }, 
    kvp(3.0F), 
    kvi(3.0F), 
    kvd(1.4F), 
    ktp(0.0F), 
    J{ {0.0F, 0.0F, 0.0F, 0.0F}, 
       {0.0F, 0.0F, 0.0F, 0.0F}, 
       {0.0F, 0.0F, 0.0F, 0.0F} }, 
    J_T{ {0.0F, 0.0F, 0.0F}, 
         {0.0F, 0.0F, 0.0F}, 
         {0.0F, 0.0F, 0.0F}, 
         {0.0F, 0.0F, 0.0F} }, 
    grab_point(1.4F)
{
}

RobotParameter::~RobotParameter() {
}

//Instanciation
Time timer;
RobotPosition pos;
RobotOrientation ornt;
RobotParameter param;

//Global variable
int cycle = 0;
int robot_state[2];     //[0] : Higher order state, [1] : Lower order state
int subscribe_flag = 0;
bool exp_flag = true;   //true : First, false : Others

//For FB experiment
ros::Time exp_t;
ros::Time exp_b;
ros::Duration exp_d;


//Prototype declaration
int CalcCurrentPosition(void);
int CalcTargetPosition(void);
float GetDistance(float x1, float y1, float z1, float x2, float y2, float z2);
bool NeedDetailInfo(void); //Needed : True, Not needed : false

//Subscribe InflatableState.msg
void msgCallback1(const inflatable_robot::InflatableState::ConstPtr& sub_msg1) {
    int state_tmp = sub_msg1->inflatable_state;
    robot_state[1] = state_tmp % 100;
    state_tmp /= 100;
    robot_state[0] = state_tmp % 10;
}

void msgCallback2(const vicon_bridge::Markers markers_msg) {
    unsigned int count = 0;
    float jump_th = 0.3F;
    vicon_bridge::Marker Arr[THE_NUMBER_OF_MARKERS];

    if (subscribe_flag < 2) {
        subscribe_flag++;
        return;
    }

    pos.vicon.x.clear();
    pos.vicon.y.clear();
    pos.vicon.z.clear();
    pos.vicon.x.shrink_to_fit();
    pos.vicon.y.shrink_to_fit();
    pos.vicon.z.shrink_to_fit();

    for(std::vector<vicon_bridge::Marker>::const_iterator it = markers_msg.markers.begin(); it != markers_msg.markers.end(); ++it) {
        Arr[count] = *it;

        if (Arr[count].occluded == false) {
            if (Arr[count].marker_name == "Hand1") {
                pos.vicon.x.emplace_back(Arr[count].translation.x / 1000.0F);
                pos.vicon.y.emplace_back(Arr[count].translation.y / 1000.0F);
                pos.vicon.z.emplace_back(Arr[count].translation.z / 1000.0F);
            }
            else if (Arr[count].marker_name == "Hand2") {
                pos.vicon.x.emplace_back(Arr[count].translation.x / 1000.0F);
                pos.vicon.y.emplace_back(Arr[count].translation.y / 1000.0F);
                pos.vicon.z.emplace_back(Arr[count].translation.z / 1000.0F);
            }
            else if (Arr[count].marker_name == "Hand3") {
                pos.vicon.x.emplace_back(Arr[count].translation.x / 1000.0F);
                pos.vicon.y.emplace_back(Arr[count].translation.y / 1000.0F);
                pos.vicon.z.emplace_back(Arr[count].translation.z / 1000.0F);
            }
            else if (Arr[count].marker_name == "Hand4") {
                pos.vicon.x.emplace_back(Arr[count].translation.x / 1000.0F);
                pos.vicon.y.emplace_back(Arr[count].translation.y / 1000.0F);
                pos.vicon.z.emplace_back(Arr[count].translation.z / 1000.0F);
            }
            else if (Arr[count].marker_name == "Base1") {
                pos.vicon.x.emplace_back(Arr[count].translation.x / 1000.0F);
                pos.vicon.y.emplace_back(Arr[count].translation.y / 1000.0F);
                pos.vicon.z.emplace_back(Arr[count].translation.z / 1000.0F);
            }
            else if (Arr[count].marker_name == "Base2") {
                pos.vicon.x.emplace_back(Arr[count].translation.x / 1000.0F);
                pos.vicon.y.emplace_back(Arr[count].translation.y / 1000.0F);
                pos.vicon.z.emplace_back(Arr[count].translation.z / 1000.0F);
            }
            else if (Arr[count].marker_name == "Base3") {
                pos.vicon.x.emplace_back(Arr[count].translation.x / 1000.0F);
                pos.vicon.y.emplace_back(Arr[count].translation.y / 1000.0F);
                pos.vicon.z.emplace_back(Arr[count].translation.z / 1000.0F);
            }
            else if (Arr[count].marker_name == "Bottle1") {
                pos.vicon.x.emplace_back(Arr[count].translation.x / 1000.0F);
                pos.vicon.y.emplace_back(Arr[count].translation.y / 1000.0F);
                pos.vicon.z.emplace_back(Arr[count].translation.z / 1000.0F);
            }
            else if (Arr[count].marker_name == "Bottle2") {
                pos.vicon.x.emplace_back(Arr[count].translation.x / 1000.0F);
                pos.vicon.y.emplace_back(Arr[count].translation.y / 1000.0F);
                pos.vicon.z.emplace_back(Arr[count].translation.z / 1000.0F);
            }
            else if (Arr[count].marker_name == "Bottle3") {
                pos.vicon.x.emplace_back(Arr[count].translation.x / 1000.0F);
                pos.vicon.y.emplace_back(Arr[count].translation.y / 1000.0F);
                pos.vicon.z.emplace_back(Arr[count].translation.z / 1000.0F);
            }
        }
        else {
            ROS_WARN("Marker %d is occluded.", count);
        }

        count++;

        if (count > THE_NUMBER_OF_MARKERS - 1) {
            break;
        }
    }

    //Marker jump prevention
    if (subscribe_flag < 3) {
        for (int i = 0; i < THE_NUMBER_OF_MARKERS; i++) {
            pos.vicon_pre.x[i] = pos.vicon.x[i];
            pos.vicon_pre.y[i] = pos.vicon.y[i];
            pos.vicon_pre.z[i] = pos.vicon.z[i];
        }

        subscribe_flag++;
        return;
    }

    for (int i = 0; i < THE_NUMBER_OF_MARKERS; i++) {
        if (abs(pos.vicon.x[i] - pos.vicon_pre.x[i]) > jump_th ||
            abs(pos.vicon.y[i] - pos.vicon_pre.y[i]) > jump_th ||
            abs(pos.vicon.z[i] - pos.vicon_pre.z[i]) > jump_th) {
            pos.vicon.x[i] = pos.vicon_pre.x[i];
            pos.vicon.y[i] = pos.vicon_pre.y[i];
            pos.vicon.z[i] = pos.vicon_pre.z[i];
            ROS_WARN("Marker %d is jumped.", i);
            return;
        }
        pos.vicon_pre.x[i] = pos.vicon.x[i];
        pos.vicon_pre.y[i] = pos.vicon.y[i];
        pos.vicon_pre.z[i] = pos.vicon.z[i];
    }

    CalcCurrentPosition();
    CalcTargetPosition();

    return;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "inflatable_vicon");
    ros::NodeHandle nh;
    ros::Publisher ros_pub = nh.advertise<inflatable_robot::InflatablePose>("inflatable_pose", 10);
    ros::Subscriber ros_sub1 = nh.subscribe("inflatable_state", 10, msgCallback1);
    ros::Subscriber ros_sub2 = nh.subscribe("/vicon/markers", 10, msgCallback2);
    ros::Rate loop_rate(SAMPLING_FREQUENCY);
    timer.base = ros::Time::now();
    timer.buf = ros::Time::now();

    while (ros::ok()) {
        //Time processing
        timer.now_t = ros::Time::now();
        timer.now_d = timer.now_t - timer.base;
        timer.sampling = timer.now_t - timer.buf;
		timer.buf = timer.now_t;
        ros::spinOnce();
        inflatable_robot::InflatablePose pub_msg;
        geometry_msgs::Point32 tmp;

        //Publish InflatablePose.msg
        pub_msg.base.x = pos.base.x[0];
        pub_msg.base.y = pos.base.y[0];
        pub_msg.base.z = pos.base.z[0];
        
        for (int i = 0; i < DEGREE_OF_FREEDOM; i++) {
            tmp.x = pos.current.x[i];
            tmp.y = pos.current.y[i];
            tmp.z = pos.current.z[i];
            pub_msg.current.emplace_back(tmp);
            tmp.x = pos.target.x[i];
            tmp.y = pos.target.y[i];
            tmp.z = pos.target.z[i];
            pub_msg.target.emplace_back(tmp);
            pub_msg.current_q.emplace_back(ornt.current.q[i]);
            pub_msg.target_q.emplace_back(ornt.target.q[i]);
        }
        
        pub_msg.hand_q = ornt.hand.q[0];
        pub_msg.init_q = ornt.init.q[0];
        pub_msg.current_yaw = param.current.yaw;
        pub_msg.target_yaw = param.target.yaw;
        pub_msg.current_reach_flag = param.current.reach_flag;
        pub_msg.target_reach_flag = param.target.reach_flag;

        ros_pub.publish(pub_msg);
        loop_rate.sleep();
        cycle++;

        if(robot_state[0] == 9) {
            break;
        }
    }

    return 0;
}

//Calculate current position
int CalcCurrentPosition(void) {
    //Base position
    pos.base.x[0] = (pos.vicon.x[BASE_FIRST_MARKER] + pos.vicon.x[BASE_FIRST_MARKER + 1]) / 2.0F;
    pos.base.y[0] = (pos.vicon.y[BASE_FIRST_MARKER] + pos.vicon.y[BASE_FIRST_MARKER + 1]) / 2.0F;
    pos.base.z[0] = (pos.vicon.z[BASE_FIRST_MARKER] + pos.vicon.z[BASE_FIRST_MARKER + 1]) / 2.0F;

    //Hand position
    pos.hand1.x[0] = (pos.vicon.x[HAND_FIRST_MARKER] + pos.vicon.x[HAND_FIRST_MARKER + 1]) / 2.0F;
    pos.hand1.y[0] = (pos.vicon.y[HAND_FIRST_MARKER] + pos.vicon.y[HAND_FIRST_MARKER + 1]) / 2.0F;
    pos.hand1.z[0] = (pos.vicon.z[HAND_FIRST_MARKER] + pos.vicon.z[HAND_FIRST_MARKER + 1]) / 2.0F;
    pos.hand2.x[0] = (pos.vicon.x[HAND_FIRST_MARKER + 2] + pos.vicon.x[HAND_FIRST_MARKER + 3]) / 2.0F;
    pos.hand2.y[0] = (pos.vicon.y[HAND_FIRST_MARKER + 2] + pos.vicon.y[HAND_FIRST_MARKER + 3]) / 2.0F;
    pos.hand2.z[0] = (pos.vicon.z[HAND_FIRST_MARKER + 2] + pos.vicon.z[HAND_FIRST_MARKER + 3]) / 2.0F;

    pos.current.x[3] = pos.hand1.x[0] + param.grab_point * (pos.hand2.x[0] - pos.hand1.x[0]);
    pos.current.y[3] = pos.hand1.y[0] + param.grab_point * (pos.hand2.y[0] - pos.hand1.y[0]);
    pos.current.z[3] = pos.hand1.z[0] + param.grab_point * (pos.hand2.z[0] - pos.hand1.z[0]);

    //Joint position
    float tmp_d = GetDistance(pos.hand1.x[0], pos.hand1.y[0], 0.0F, pos.hand2.x[0], pos.hand2.y[0], 0.0F);
    ornt.hand.q[0] = atan2f(pos.hand2.z[0] - pos.hand1.z[0], tmp_d);
    ornt.current.q[0] = atan2f(pos.current.y[3] - pos.base.y[0], pos.current.x[3] - pos.base.x[0]);
    param.current.delta = 100.0F * M_PI / 180.0F;

    pos.current.x[0] = pos.base.x[0] + param.link_u * sinf(ornt.offset.q[0]) * cosf(ornt.current.q[0]);
    pos.current.y[0] = pos.base.y[0] + param.link_u * sinf(ornt.offset.q[0]) * sinf(ornt.current.q[0]);
    pos.current.z[0] = pos.base.z[0] + param.link_l + param.link_u * cosf(ornt.offset.q[0]);

    pos.current.x[2] = pos.current.x[3] - param.link[2] * sinf(param.current.delta) * cosf(ornt.current.q[0]);
    pos.current.y[2] = pos.current.y[3] - param.link[2] * sinf(param.current.delta) * sinf(ornt.current.q[0]);
    pos.current.z[2] = pos.current.z[3] - param.link[2] * cosf(param.current.delta);

    param.current.distance = GetDistance(
        pos.current.x[0], 
        pos.current.y[0], 
        pos.current.z[0], 
        pos.current.x[3], 
        pos.current.y[3], 
        pos.current.z[3]
    );

    //Initialize init.q
    if ( (robot_state[0] == 2 && robot_state[1] == 1) ||
         (robot_state[0] == 6 && robot_state[1] == 1) ) {
           ornt.init.q[0] = ornt.current.q[0];
    }

    if (NeedDetailInfo() == false) {
        for (int i = 0; i < DEGREE_OF_FREEDOM - 1; i++) {
            pos.current.x[i] = 9999.0F;
            pos.current.y[i] = 9999.0F;
            pos.current.z[i] = 9999.0F;
        }

        for (int i = 0; i < DEGREE_OF_FREEDOM; i++) {
            ornt.current.q[i] = 9999.0F;
        }
        param.current.yaw = 0.0F;
        param.current.reach_flag = false;

        return -1;
    }

    //Current yaw
    if (ornt.current.q[0] - ornt.init.q[0] <= -1 * M_PI) {
        param.current.yaw = ornt.current.q[0] - ornt.init.q[0] + 2 * M_PI;
    }
    else if (ornt.current.q[0] - ornt.init.q[0] > -1 * M_PI  && 
             ornt.current.q[0] - ornt.init.q[0] < M_PI) {
        param.current.yaw = ornt.current.q[0] - ornt.init.q[0];
    }
    else if (ornt.current.q[0] - ornt.init.q[0] >= M_PI) {
        param.current.yaw = ornt.current.q[0] - ornt.init.q[0] - 2 * M_PI;
    }
    else {
        ROS_WARN("Current.q[0]");
    }

    if(param.current.yaw > -1 * M_PI / 2.0F && 
       param.current.yaw < M_PI / 2.0F) {
        param.current.reach_flag = true;
    }
    else {
        param.current.reach_flag = false;
        ROS_WARN("Current.yaw is %f [deg].", param.current.yaw * 180.0F / M_PI);
        return -1;
    }

    //Inverse kinematics
    if(param.current.distance < param.link[1] + param.link[2] + param.link[3]) {
        //Parallel translation
        pos.tmp.x[2] = pos.current.x[2] - param.link_u * sinf(ornt.offset.q[0]) * cosf(ornt.current.q[0]);
        pos.tmp.y[2] = pos.current.y[2] - param.link_u * sinf(ornt.offset.q[0]) * sinf(ornt.current.q[0]);
        pos.tmp.z[2] = pos.current.z[2] - param.link_u * cosf(ornt.offset.q[0]) + param.link_u;

        pos.current2.x[2] = (pos.tmp.x[2] - pos.base.x[0]) * (pos.tmp.x[2] - pos.base.x[0]);
        pos.current2.y[2] = (pos.tmp.y[2] - pos.base.y[0]) * (pos.tmp.y[2] - pos.base.y[0]);
        pos.current2.z[2] = (pos.tmp.z[2] - pos.base.z[0] - param.link[0]) * (pos.tmp.z[2] - pos.base.z[0] - param.link[0]);

        param.current.alpha = acosf((param.link2[1] + param.link2[2] - pos.current2.x[2] - pos.current2.y[2] - pos.current2.z[2]) / (2.0F * param.link[1] * param.link[2]));
        param.current.beta = acosf((param.link2[1] - param.link2[2] + pos.current2.x[2] + pos.current2.y[2] + pos.current2.z[2]) / (2.0F * param.link[1] * sqrtf(pos.current2.x[2] + pos.current2.y[2] + pos.current2.z[2])));
        param.current.gamma = atan2f(sqrtf(pos.current2.x[2] + pos.current2.y[2]), pos.current.z[2] - pos.base.z[0] - param.link[0]);

        ornt.current.q[1] = param.current.gamma - param.current.beta - ornt.offset.q[0];

        if (ornt.current.q[1] > -1 * ornt.limit.q[0] * M_PI / 180.0F && 
            ornt.current.q[1] < ornt.limit.q[0] * M_PI / 180.0F) {
			param.current.reach_flag = true;
		}
		else {
			param.current.reach_flag = false;
            ROS_WARN("Current.q[1] is %f [deg].", ornt.current.q[1] * 180.0F / M_PI);
			return -1;
		}

		ornt.current.q[2] = M_PI - param.current.alpha;
		if (ornt.current.q[2] > -1 * ornt.limit.q[0] * M_PI / 180.0F && 
            ornt.current.q[2] < ornt.limit.q[0] * M_PI / 180.0F) {
			param.current.reach_flag = true;
		}
		else {
			param.current.reach_flag = false;
            ROS_WARN("Current.q[2] is %f [deg].", ornt.current.q[2] * 180.0F / M_PI);
			return -1;
		}

		ornt.current.q[3] = param.current.delta - ornt.current.q[1] - ornt.current.q[2] - ornt.offset.q[0];
		if (ornt.current.q[3] > -1 * ornt.limit.q[0] * M_PI / 180.0F && 
            ornt.current.q[3] < ornt.limit.q[0] * M_PI / 180.0F) {
			param.current.reach_flag = true;
		}
		else {
			param.current.reach_flag = false;
            ROS_WARN("Current.q[3] is %f [deg].", ornt.current.q[3] * 180.0F / M_PI);
			return -1;
		}

		pos.current.x[1] = pos.current.x[0] + param.link[1] * sinf(ornt.current.q[1] + ornt.offset.q[0]) * cosf(ornt.current.q[0]);
		pos.current.y[1] = pos.current.y[0] + param.link[1] * sinf(ornt.current.q[1] + ornt.offset.q[0]) * sinf(ornt.current.q[0]);
		pos.current.z[1] = pos.current.z[0] + param.link[1] * cosf(ornt.current.q[1] + ornt.offset.q[0]);
    }
	else {
		param.current.reach_flag = false;
        ROS_WARN("Current marker is out of range %f [m].", param.current.distance);
		return -1;
	}

    return 0;
}

//Calculate target position
int CalcTargetPosition(void) {
    if (robot_state[0] == 1) {
        pos.target.x[3] = pos.vicon.x[BOTTLE_FIRST_MARKER];
        pos.target.y[3] = pos.vicon.y[BOTTLE_FIRST_MARKER];
        pos.target.z[3] = pos.vicon.z[BOTTLE_FIRST_MARKER] - 0.03F;
    }

    else if (robot_state[0] == 2 && robot_state[1] == 1) {
        pos.target.x[3] = pos.vicon.x[BOTTLE_FIRST_MARKER];
        pos.target.y[3] = pos.vicon.y[BOTTLE_FIRST_MARKER];
        pos.target.z[3] = pos.vicon.z[BOTTLE_FIRST_MARKER] - 0.03F;
    }

    //Preparation target
    else if (robot_state[0] == 2 && robot_state[1] == 2) {
        pos.target.x[3] = pos.vicon.x[BOTTLE_FIRST_MARKER];
        pos.target.y[3] = pos.vicon.y[BOTTLE_FIRST_MARKER];
        pos.target.z[3] = pos.vicon.z[BOTTLE_FIRST_MARKER] - 0.03F;
        pos.lift.x[3] = pos.target.x[3];
        pos.lift.y[3] = pos.target.y[3];
        pos.lift.z[3] = pos.target.z[3];

        //For FB experiment
        //Right
        //pos.target.x[3] = 1.901856;
        //pos.target.y[3] = 2.150156;
        //pos.target.z[3] = 1.100546;
        //Down
        //pos.target.x[3] = 1.741856;
        //pos.target.y[3] = 2.220156;
        //pos.target.z[3] = 0.880546;
        //Up
        //pos.target.x[3] = 1.707118;
        //pos.target.y[3] = 2.175156;
        //pos.target.z[3] = 1.075468;
        pos.target.x[3] = 1.69132;
        pos.target.y[3] = 2.19187;
        pos.target.z[3] = 0.80157;
        //Vibration
        //float freq = 0.20F; //[Hz]
        //if (exp_flag == true) {
        //    exp_b = ros::Time::now();
        //    exp_flag = false;
        //}
        //exp_t = ros::Time::now();
        //exp_d = exp_t - exp_b;
        //float tt = (float)exp_d.sec + (float)exp_d.nsec / 1e9;
        //float targetx1 = 1.720484;
        //float targety1 = 2.150156;
        //float targetz1 = 1.100165;
        //float targetx2 = 1.741856;
        //float targety2 = 2.220156;
        //float targetz2 = 0.880546;
        //float tmpx = targetx2 - targetx1;
        //float tmpy = targety2 - targety1;
        //float tmpz = targetz2 - targetz1;
        //pos.target.x[3] = targetx1 + tmpx * (0.5F * sinf(2.0F * M_PI * freq * tt - 0.5F * M_PI) + 0.5F);
        //pos.target.y[3] = targety1 + tmpy * (0.5F * sinf(2.0F * M_PI * freq * tt - 0.5F * M_PI) + 0.5F);
        //pos.target.z[3] = targetz1 + tmpz * (0.5F * sinf(2.0F * M_PI * freq * tt - 0.5F * M_PI) + 0.5F);
    }

    //Grasping target
    else if (robot_state[0] == 2 && robot_state[1] == 3) {
        pos.target.x[3] = pos.vicon.x[BOTTLE_FIRST_MARKER];
        pos.target.y[3] = pos.vicon.y[BOTTLE_FIRST_MARKER];
        pos.target.z[3] = pos.vicon.z[BOTTLE_FIRST_MARKER] - 0.12F;
    }

    else if (robot_state[0] == 3) {
        pos.target.x[3] = pos.vicon.x[BOTTLE_FIRST_MARKER];
        pos.target.y[3] = pos.vicon.y[BOTTLE_FIRST_MARKER];
        pos.target.z[3] = pos.vicon.z[BOTTLE_FIRST_MARKER] - 0.12F;
    }
    //Lifting target
    else if (robot_state[0] == 4 && robot_state[1] == 1) {
        pos.target.x[3] = pos.lift.x[3];
        pos.target.y[3] = pos.lift.y[3];
        pos.target.z[3] = pos.lift.z[3];
    }
    //Giving to human target
    else if (robot_state[0] == 6 ||
             robot_state[0] == 7) {
        pos.target.x[3] = 3.96F;
        pos.target.y[3] = 1.20F;
        pos.target.z[3] = 1.02F;
    }
    else {
        pos.target.x[3] = 9999.0F;
        pos.target.y[3] = 9999.0F;
        pos.target.z[3] = 9999.0F;
    }

    //Joint position
    ornt.target.q[0] = atan2f(pos.target.y[3] - pos.base.y[0], pos.target.x[3] - pos.base.x[0]);
    param.target.delta = 100.0F * M_PI / 180.0F;

    pos.target.x[0] = pos.base.x[0] + param.link_u * sinf(ornt.offset.q[0]) * cosf(ornt.target.q[0]);
    pos.target.y[0] = pos.base.y[0] + param.link_u * sinf(ornt.offset.q[0]) * sinf(ornt.target.q[0]);
    pos.target.z[0] = pos.base.z[0] + param.link_l + param.link_u * cosf(ornt.offset.q[0]);

    pos.target.x[2] = pos.target.x[3] - param.link[2] * sinf(param.target.delta) * cosf(ornt.target.q[0]);
    pos.target.y[2] = pos.target.y[3] - param.link[2] * sinf(param.target.delta) * sinf(ornt.target.q[0]);
    pos.target.z[2] = pos.target.z[3] - param.link[2] * cosf(param.target.delta);

    param.target.distance = GetDistance(
        pos.target.x[0], 
        pos.target.y[0], 
        pos.target.z[0], 
        pos.target.x[3], 
        pos.target.y[3], 
        pos.target.z[3]
    );

    if (NeedDetailInfo() == false) {
        for (int i = 0; i < DEGREE_OF_FREEDOM - 1; i++) {
            pos.target.x[i] = 9999.0F;
            pos.target.y[i] = 9999.0F;
            pos.target.z[i] = 9999.0F;
        }

        for (int i = 0; i < DEGREE_OF_FREEDOM; i++) {
            ornt.target.q[i] = 9999.0F;
        }
        param.target.yaw = 0.0F;
        param.target.reach_flag = false;

        return -1;
    }

    //Target yaw
    if (ornt.target.q[0] - ornt.init.q[0] <= -1 * M_PI) {
        param.target.yaw = ornt.target.q[0] - ornt.init.q[0] + 2 * M_PI;
    }
    else if (ornt.target.q[0] - ornt.init.q[0] > -1 * M_PI  && ornt.target.q[0] - ornt.init.q[0] < M_PI) {
        param.target.yaw = ornt.target.q[0] - ornt.init.q[0];
    }
    else if (ornt.target.q[0] - ornt.init.q[0] >= M_PI) {
        param.target.yaw = ornt.target.q[0] - ornt.init.q[0] - 2 * M_PI;
    }
    else {
        ROS_WARN("Target.q[0]");
    }

    if(param.target.yaw > -1 * M_PI / 2.0F && param.target.yaw < M_PI / 2.0F) {
        param.target.reach_flag = true;
    }
    else {
        param.target.reach_flag = false;
        ROS_WARN("Target.yaw is %f [deg].", param.target.yaw * 180.0F / M_PI);
        return -1;
    }

    //Inverse kinematics
    if(param.target.distance < param.link[1] + param.link[2] + param.link[3]) {
        //Parallel translation
        pos.tmp.x[2] = pos.target.x[2] - param.link_u * sinf(ornt.offset.q[0]) * cosf(ornt.target.q[0]);
        pos.tmp.y[2] = pos.target.y[2] - param.link_u * sinf(ornt.offset.q[0]) * sinf(ornt.target.q[0]);
        pos.tmp.z[2] = pos.target.z[2] - param.link_u * cosf(ornt.offset.q[0]) + param.link_u;

        pos.target2.x[2] = (pos.tmp.x[2] - pos.base.x[0]) * (pos.tmp.x[2] - pos.base.x[0]);
        pos.target2.y[2] = (pos.tmp.y[2] - pos.base.y[0]) * (pos.tmp.y[2] - pos.base.y[0]);
        pos.target2.z[2] = (pos.tmp.z[2] - pos.base.z[0] - param.link[0]) * (pos.tmp.z[2] - pos.base.z[0] - param.link[0]);

        param.target.alpha = acosf((param.link2[1] + param.link2[2] - pos.target2.x[2] - pos.target2.y[2] - pos.target2.z[2]) / (2.0F * param.link[1] * param.link[2]));
        param.target.beta = acosf((param.link2[1] - param.link2[2] + pos.target2.x[2] + pos.target2.y[2] + pos.target2.z[2]) / (2.0F * param.link[1] * sqrtf(pos.target2.x[2] + pos.target2.y[2] + pos.target2.z[2])));
        param.target.gamma = atan2f(sqrtf(pos.target2.x[2] + pos.target2.y[2]), pos.target.z[2] - pos.base.z[0] - param.link[0]);

        ornt.target.q[1] = param.target.gamma - param.target.beta - ornt.offset.q[0];

        if (ornt.target.q[1] > -1 * ornt.limit.q[0] * M_PI / 180.0F && ornt.target.q[1] < ornt.limit.q[0] * M_PI / 180.0F) {
			param.target.reach_flag = true;
		}
		else {
			param.target.reach_flag = false;
            ROS_WARN("Target.q[1] is %f [deg].", ornt.target.q[1] * 180.0F / M_PI);
			return -1;
		}
		ornt.target.q[2] = M_PI - param.target.alpha;
		if (ornt.target.q[2] > -1 * ornt.limit.q[0] * M_PI / 180.0F && ornt.target.q[2] < ornt.limit.q[0] * M_PI / 180.0F) {
			param.target.reach_flag = true;
		}
		else {
			param.target.reach_flag = false;
            ROS_WARN("Target.q[2] is %f [deg].", ornt.target.q[2] * 180.0F / M_PI);
			return -1;
		}
		ornt.target.q[3] = param.target.delta - ornt.target.q[1] - ornt.target.q[2] - ornt.offset.q[0];
		if (ornt.target.q[3] > -1 * ornt.limit.q[0] * M_PI / 180.0F && ornt.target.q[3] < ornt.limit.q[0] * M_PI / 180.0F) {
			param.target.reach_flag = true;
		}
		else {
			param.target.reach_flag = false;
            ROS_WARN("Target.q[3] is %f [deg].", ornt.target.q[3] * 180.0F / M_PI);
			return -1;
		}

		pos.target.x[1] = pos.target.x[0] + param.link[1] * sinf(ornt.target.q[1] + ornt.offset.q[0]) * cosf(ornt.target.q[0]);
		pos.target.y[1] = pos.target.y[0] + param.link[1] * sinf(ornt.target.q[1] + ornt.offset.q[0]) * sinf(ornt.target.q[0]);
		pos.target.z[1] = pos.target.z[0] + param.link[1] * cosf(ornt.target.q[1] + ornt.offset.q[0]);
	}
	else {
		param.target.reach_flag = false;
		ROS_WARN("WARNING : Target marker is out of range %f[m].", param.target.distance);
		return -1;
	}

    return 0;
}

//Distance between two points
float GetDistance(float x1, float y1, float z1, float x2, float y2, float z2) {
	float distance;
	distance = pow((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) + (z2 - z1) * (z2 - z1), 0.5F);

	return distance;
}

//Avoid unnecessary error
bool NeedDetailInfo(void) {
    bool ret;

    if (robot_state[0] == 2 ||
        robot_state[0] == 3 ||
        robot_state[0] == 4 ||
        robot_state[0] == 6 ||
        robot_state[0] == 7) {
        ret = true;
    }
    else {
        ret = false;
    }

    return ret;
}