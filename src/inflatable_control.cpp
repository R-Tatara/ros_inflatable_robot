/*++

Name : inflatable_control.cpp

Abstract : Visual feedback control for an inflatable robot

Author : Ryosuke Tatara

--*/

#include "ros/ros.h"
#include <fstream>
#include <cmath>
//Publishing
#include "inflatable_robot/InflatableStateFlag.h"
#include "inflatable_robot/VoltageOutput.h"
//Subscribing
#include "inflatable_robot/InflatableState.h"
#include "inflatable_robot/InflatablePose.h"
#include "inflatable_robot/VoltageInput.h"

#define GRAVITY_ACC 9.80665F
#define DEGREE_OF_FREEDOM 4
#define DA_CHANNEL_NUMBER 12
#define AD_CHANNEL_NUMBER 8
#define LINK_CHANNEL_NUMBER 8 //From 8 to 10
#define HAND_CHANNEL_NUMBER 11
#define MAXIMUM_SENSOR_VOLTAGE 9.5
#define MINIMUM_SENSOR_VOLTAGE 0.5
#define BASE_PRESSURE 30.0F
#define LINK_PRESSURE 50.0F
#define HAND_PRESSURE 40.0F
#define MAXIMUM_PRESSURE 60.0F
#define MINIMUM_PRESSURE 0.0F
#define MAXIMUM_LINK_PRESSURE 120.0F
#define MAXIMUM_HAND_PRESSURE 200.0F
#define LINK_PRESSURIZATION_TIME 12.0F
#define ACTUATOR_PRESSURIZATION_TIME 10.0F
#define BOTTLE_WEIGHT 0.2F //[kg] //Table height : 88[cm]
#define SAMPLING_FREQUENCY 100


class Time {
public:
    ros::Time now_t;         //Current time
    ros::Duration now_d;     //Current duration time
    ros::Time base;          //Time for current time
    ros::Time buf;           //Time for sampling time
    ros::Duration sampling;  //Sampling time
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
    Direction base;
	Direction current;
	Direction target;
    RobotPosition();
    ~RobotPosition();
};

RobotPosition::RobotPosition() {
    this->base.x.emplace_back(0.0F);
    this->base.y.emplace_back(0.0F);
    this->base.z.emplace_back(0.0F);
    for (int i = 0; i < DEGREE_OF_FREEDOM; i++) {
        this->current.x.emplace_back(0.0F);
        this->current.y.emplace_back(0.0F);
        this->current.z.emplace_back(0.0F);
        this->target.x.emplace_back(0.0F);
        this->target.y.emplace_back(0.0F);
        this->target.z.emplace_back(0.0F);
    }
}

RobotPosition::~RobotPosition() {
}

class Diviation : public Direction {
public:
    Direction P;             //Diviation
    Direction I;             //Integral(diviation)dt
    Direction D;             //d/dt(diviation)
    Direction P_buf;         //Previous diviation
    std::vector<float> pressure_integral;
    Diviation();
    ~Diviation();
};

Diviation::Diviation() {
    this->P.x.emplace_back(9999.0F);
    this->P.y.emplace_back(9999.0F);
    this->P.z.emplace_back(9999.0F);
    this->I.x.emplace_back(0.0F);
    this->I.y.emplace_back(0.0F);
    this->I.z.emplace_back(0.0F);
    this->D.x.emplace_back(0.0F);
    this->D.y.emplace_back(0.0F);
    this->D.z.emplace_back(0.0F);
    this->P_buf.x.emplace_back(0.0F);
    this->P_buf.y.emplace_back(0.0F);
    this->P_buf.z.emplace_back(0.0F);
    for(int i = 0; i < DA_CHANNEL_NUMBER; i++) {
        this->pressure_integral.emplace_back(0.0F);
    }
}

Diviation::~Diviation() {
}

class RobotParameter : public Direction {
public:
    Direction force;                   //Force
    float torque[DEGREE_OF_FREEDOM];   //Torque
    float torque_g[DEGREE_OF_FREEDOM]; //Compensate gravity torque
    float torque_g_rate;               //Compensate gravity torque rate
    float link[DEGREE_OF_FREEDOM];     //Link length
    float link_g[DEGREE_OF_FREEDOM];   //Length between joint and center of gravity
    float link_u;                      //Upper first link length
    float link_l;                      //Lower first link length
    float m_l[DEGREE_OF_FREEDOM];      //Mass of link
    float m_a[DEGREE_OF_FREEDOM];      //Mass of actuator
    float J[3][DEGREE_OF_FREEDOM];     //Jacobian matrix
    float J_T[DEGREE_OF_FREEDOM][3];   //Transposed Jacobian matrix
    float current_yaw;                 //Yaw angle with respect to base coordinate
    float target_yaw;
    RobotParameter();
    ~RobotParameter();
};

RobotParameter::RobotParameter():
    torque_g_rate(0.5F), 
    link{ 0.35F, 0.28F, 0.28F, 0.26F }, 
    link_g{0.0F, 0.0F, 0.0F, 0.0F}, 
    link_u{ 0.6F * this->link[0] }, 
    link_l{ 0.4F * this->link[0] }, 
    m_l{0.05F, 0.05F, 0.05F, 0.05F}, 
    m_a{0.12F, 0.10F, 0.09F, 0.00F}, 
    current_yaw(0.0F), 
    target_yaw(0.0F)
{
    for (int i = 0; i < DEGREE_OF_FREEDOM; i++) {
        this->force.x.emplace_back(0.0F);
        this->force.y.emplace_back(0.0F);
        this->force.z.emplace_back(0.0F);
        this->torque[i] = 0.0F;
        this->torque_g[i] = 0.0F;
        for (int j = 0; i < 3; i++) {
           this->J[j][i] = 0.0F;
           this->J_T[i][j] = 0.0F;
        }
    }
}

RobotParameter::~RobotParameter() {
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
    Angle offset;  //Offset angle of pedestal
    Angle hand;    //Hand angle
    Angle current; //Current angle
    Angle target;  //Target angle
    RobotOrientation();
    ~RobotOrientation();
};

RobotOrientation::RobotOrientation() {
    this->offset.q.emplace_back(45.0F /180.0F * M_PI);
    this->hand.q.emplace_back(0.0F);
    for (int i = 0; i < DEGREE_OF_FREEDOM; i++) {
        this->current.q.emplace_back(0.0F);
        this->target.q.emplace_back(0.0F);
    }
}

RobotOrientation::~RobotOrientation() {
}

class Pressure {
public:
    float base[DA_CHANNEL_NUMBER];        //Base pressure
    float target[DA_CHANNEL_NUMBER];      //Target pressure
    float target_buf[DA_CHANNEL_NUMBER];  //Previous target pressure
    float current[DA_CHANNEL_NUMBER];     //Current pressure
    float current_buf[DA_CHANNEL_NUMBER]; //Previous current pressure
    float P[DA_CHANNEL_NUMBER];           //Diviation
    float I[DA_CHANNEL_NUMBER];           //int(diviation)dt
    float D[DA_CHANNEL_NUMBER];           //d/dt(diviation)
    float output[DA_CHANNEL_NUMBER];      //Output pressure
    float input[DA_CHANNEL_NUMBER];       //Input pressure
    Pressure();
    ~Pressure();
};

Pressure::Pressure():
base{ BASE_PRESSURE + 3.0F,
        BASE_PRESSURE - 3.0F,
        BASE_PRESSURE - 12.0F,
        BASE_PRESSURE + 12.0F,
        BASE_PRESSURE + 5.0F,
        BASE_PRESSURE - 10.0F,
        BASE_PRESSURE,
        BASE_PRESSURE,
        LINK_PRESSURE,
        LINK_PRESSURE,
        LINK_PRESSURE,
        0.0F }
{
    for (int i = 0; i < DA_CHANNEL_NUMBER; i++) {
        this->target[i] = 0.0F;
        this->target_buf[i] = 0.0F;
        this->current[i] = 0.0F;
        this->current_buf[i] = this->base[i];
        this->P[i] = 0.0F;
        this->I[i] = 0.0F;
        this->D[i] = 0.0F;
        this->output[i] = 0.0F;
        this->input[i] = 0.0F;
    }
}

Pressure::~Pressure() {
}

class Voltage {
public:
    std::vector<float> voltage;
    std::vector<float> reliability; //0 : Reliable, -1 : Not reliable
    Voltage();
    ~Voltage();
};

Voltage::Voltage() {
}

Voltage::~Voltage() {
}

class IO : public Voltage {
public:
    Voltage input;
    Voltage output;
    IO();
    ~IO();
};

IO::IO() {
    for (int i = 0; i < AD_CHANNEL_NUMBER; i++) {
        this->input.voltage.emplace_back(0.0F);
        this->input.reliability.emplace_back(0.0F);
    }
    for (int i = 0; i < DA_CHANNEL_NUMBER; i++) {
        this->output.voltage.emplace_back(0.0F);
        this->output.reliability.emplace_back(0.0F);
    }
}

IO::~IO() {
}

class Coefficient {
public:
    float torque[DA_CHANNEL_NUMBER];
    const float visual_P;                      //Visual feedback P gain
    const float visual_I;                      //Visual feedback I gain
    const float visual_D;                      //Visual feedback D gain
    const float pressure_P[AD_CHANNEL_NUMBER]; //Pressure feedback P gain
    const float pressure_I[AD_CHANNEL_NUMBER]; //Pressure feedback I gain
    const float pressure_D[AD_CHANNEL_NUMBER]; //Pressure feedback D gain
    const float AD_a[AD_CHANNEL_NUMBER];       //Voltage - pressure conversion coefficient
    const float AD_b[AD_CHANNEL_NUMBER];
    const float AD_c[AD_CHANNEL_NUMBER];
    const float DA_a[DA_CHANNEL_NUMBER];       //Pressure - voltage conversion coefficient
    const float DA_b[DA_CHANNEL_NUMBER];
    const float DA_c[DA_CHANNEL_NUMBER];
    Coefficient();
    ~Coefficient();
};

Coefficient::Coefficient():
    torque{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F }, 
    visual_P(3.0F), 
    visual_I(3.5F), 
    visual_D(0.0F), //Lowpass filter is required
    pressure_P{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F}, 
    pressure_I{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F}, 
    pressure_D{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F}, 
    AD_a{ -1.8E-02F, -5.6E-02F, -4.3E-02F,      0.0F, -7.4E-02F,      0.0F, -6.8E-01F,  3.1E-02F },
	AD_b{ -1.4E+01F, -1.4E+01F, -1.4E+01F, -1.5E+01F, -1.4E+01F, -1.5E+01F, -4.7E+00F, -1.5E+01F },
	AD_c{  1.3E+02F,  1.4E+02F,  1.3E+02F,  1.4E+02F,  1.3E+02F,  1.3E+02F,  9.9E+01F,  1.4E+02F },
    DA_a{  7.9E-06F, -1.3E-04F, -7.7E-06F, -3.2E-05F,  1.0E-04F,      0.0F,  6.4E-06F, -4.2E-06F,       0.0F,       0.0F,       0.0F,       0.0F },
	DA_b{  9.9E-02F,  1.1E-01F,  1.0E-01F,  7.0E-03F,  9.3E-02F,  1.0E-01F,  5.2E-03F,  5.8E-03F,  1.05E-02F,  1.04E-02F,  1.05E-02F,  1.05E-02F },
	DA_c{ -9.4E-03F, -7.4E-02F,  1.1E-01F, -3.7E-02F, -5.7E-04F,  7.5E-03F, -9.5E-03F,  5.3E-03F,  1.99E-02F,  1.08E-02F, -1.65E-03F,  2.92E-02F }
{
}

Coefficient::~Coefficient() {
}

class Flag {
public:
    std::vector<bool> overpressure; //true : safe, false : unsafe
    bool current_reach;             //true : safe, false : unsafe
    bool target_reach;              //true : safe, false : unsafe
    Flag();
    ~Flag();
};

Flag::Flag():
    current_reach(false),
    target_reach(false)
{
    for (int i = 0; i < AD_CHANNEL_NUMBER; i++) {
        this->overpressure.emplace_back(false);
    }
}

Flag::~Flag() {
}

//Instanciation
Time timer;
RobotPosition pos;
Diviation diviation;
RobotParameter param;
RobotOrientation ornt;
Pressure pressure;
IO io;
Coefficient k;
Flag flag;

//Global variable
int cycle = 0;
int visual_FB_cycle = 0;
int pressure_FB_cycle = 0;
int robot_state[2];                      //[0] : Higher order state, [1] : Lower order state
bool safety_flag = false;                //true : safe, false : unsafe
bool base_pressure_link_flag = true;     //true : First, false : Others
bool base_pressure_actuator_flag = true; //true : First, false : Others
ros::Time base_time;
ros::Time now_time;

//Prototype declaration
int BasePressureLink(void);
int BasePressureActuator(void);
int PressurizeHand(void);
int FeedbackControl(void);
int CompensateGravity(void);
int CheckRobotState(void);
float GetDistance(float x1, float y1, float z1, float x2, float y2, float z2);
int CheckOverpressure(void);
int DAconversion(void);
int ADconversion(void);
int Decompression(void);

//Subscribe InflatableState.msg
void msgCallback1(const inflatable_robot::InflatableState::ConstPtr& sub_msg1) {
    int state_tmp = sub_msg1->inflatable_state;
    robot_state[1] = state_tmp % 100;
    state_tmp /= 100;
    robot_state[0] = state_tmp % 10;
}

//Subscribe InflatablePose.msg
void msgCallback2(const inflatable_robot::InflatablePose::ConstPtr& sub_msg2) {
    pos.base.x[0] = sub_msg2->base.x;
    pos.base.y[0] = sub_msg2->base.y;
    pos.base.z[0] = sub_msg2->base.z;
    for (int i = 0; i < DEGREE_OF_FREEDOM; i++) {
        pos.current.x[i] = sub_msg2->current[i].x;
        pos.current.y[i] = sub_msg2->current[i].y;
        pos.current.z[i] = sub_msg2->current[i].z;
        pos.target.x[i] = sub_msg2->target[i].x;
        pos.target.y[i] = sub_msg2->target[i].y;
        pos.target.z[i] = sub_msg2->target[i].z;
        ornt.current.q[i] = sub_msg2->current_q[i];
        ornt.target.q[i] = sub_msg2->target_q[i];
    }

    ornt.hand.q[0] = sub_msg2->hand_q;
    //Yaw angle
    param.current_yaw = sub_msg2->current_yaw;
    param.target_yaw = sub_msg2->target_yaw;

    //Reach_flag
    flag.current_reach = sub_msg2->current_reach_flag;
    flag.target_reach = sub_msg2->target_reach_flag;
}

//Subscribe VoltageInput.msg
void msgCallback3(const inflatable_robot::VoltageInput::ConstPtr& sub_msg3) {
    for(int i = 0; i < AD_CHANNEL_NUMBER; i++) {
        io.input.voltage[i] = sub_msg3->voltage_input[i];
    }

    //Conversion from voltage to preassure
    ADconversion();

    for (int i = 0; i < AD_CHANNEL_NUMBER; i++) {
        io.input.reliability[i] = io.input.voltage[i] > MINIMUM_SENSOR_VOLTAGE && io.input.voltage[i] < MAXIMUM_SENSOR_VOLTAGE ? 0 : -1;
    }

    //Pressure feedback control
    for (int i = 0; i < AD_CHANNEL_NUMBER; i++) {
        if (pressure_FB_cycle > 2) {
            if (robot_state[0] != 0) {
                diviation.pressure_integral[i] += ((pressure.target_buf[i] - pressure.current_buf[i])
                                       + (pressure.target[i] - pressure.current[i]))
                                       / (2.0F * SAMPLING_FREQUENCY);
            }
        }

        if (io.input.reliability[i] == 0) {
            if (pressure_FB_cycle > 1) {
                //P component
                pressure.P[i] = k.pressure_P[i] * (pressure.target[i] - pressure.current[i]);
                pressure.output[i] += pressure.P[i];

                //I component
                pressure.I[i] = k.pressure_I[i] * diviation.pressure_integral[i];
                pressure.output[i] += pressure.I[i];

                //D component
                pressure.D[i] = -1.0F * k.pressure_D[i] * (pressure.current[i] - pressure.current_buf[i]);
                pressure.output[i] += pressure.D[i];

                //Update previous value
                pressure.target_buf[i] = pressure.target[i];
                pressure.current_buf[i] = pressure.current[i];
            }
        }
    }

    //Overpressure prevention
    CheckOverpressure();
    DAconversion();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "inflatable_control");
    ros::NodeHandle nh;
    ros::Publisher ros_pub1 = nh.advertise<inflatable_robot::InflatableStateFlag>("inflatable_state_flag", 10);
    ros::Publisher ros_pub2 = nh.advertise<inflatable_robot::VoltageOutput>("voltage_output", 10);
    ros::Subscriber ros_sub1 = nh.subscribe("inflatable_state", 10, msgCallback1);
    ros::Subscriber ros_sub2 = nh.subscribe("inflatable_pose", 10, msgCallback2);
    ros::Subscriber ros_sub3 = nh.subscribe("voltage_input", 10, msgCallback3);
    ros::Rate loop_rate(SAMPLING_FREQUENCY);
    inflatable_robot::InflatableStateFlag pub_msg1;
    inflatable_robot::VoltageOutput pub_msg2;
    timer.base = ros::Time::now();
    timer.buf = ros::Time::now();

    //File output preparation
    char filename[64];
    time_t date_info = time(NULL);
    struct tm *pnow = localtime(&date_info);
    sprintf(filename, "/home/tatara/catkin_ws/src/inflatable_robot/result/%04d%02d%02d%02d%02d.csv", 
        pnow->tm_year + 1900, 
        pnow->tm_mon + 1, 
        pnow->tm_mday, 
        pnow->tm_hour, 
        pnow->tm_min);
    std::ofstream ofs(filename);
    ofs.clear();
    ofs.seekp(0);
    ofs << "cycle" << ',' << "Time[s]" << ','
        << "robot state" << ',' << "Current reach flag" << ',' << "Target reach flag" << ','
        << "xb[m]" << ',' << "yb[m]" << ',' << "zb[m]" << ','
        << "xc[m]" << ',' << "yc[m]" << ',' << "zc[m]" << ','
        << "xt[m]" << ',' << "yt[m]" << ',' << "zt[m]" << ','
        << "qc0[deg]" << ',' << "qc1[deg]" << ',' << "qc2[deg]" << ',' << "qc3[deg]" << ',' << "qca[deg]" << ','
        << "qt0[deg]" << ',' << "qt1[deg]" << ',' << "qt2[deg]" << ',' << "qt3[deg]" << ',' << "qta[deg]" << ','
        << "Fx[N]" << ',' << "Fy[N]" << ',' << "Fz[N]" << ','
        << "torque0[Nm]" << ',' << "torque1[Nm]" << ',' << "torque2[Nm]" << ',' << "torque3[Nm]" << ',';
    for (int i = 0; i < AD_CHANNEL_NUMBER; i++) { ofs << "Target pressure" << i << "[kPa]" << ','; }
    ofs << std::endl;

    while (ros::ok()) {
        //Time processing
        timer.now_t = ros::Time::now();
        timer.now_d = timer.now_t - timer.base;
        timer.sampling = timer.now_t - timer.buf;
		timer.buf = timer.now_t;
        ros::spinOnce();

        //Hand state
        if (robot_state[0] == 4 || 
            robot_state[0] == 5 || 
            robot_state[0] == 6) {
            pressure.target[HAND_CHANNEL_NUMBER] = HAND_PRESSURE;
        }
        else {
            pressure.target[HAND_CHANNEL_NUMBER] = MINIMUM_PRESSURE + 1.0F;
        }
        pressure.output[HAND_CHANNEL_NUMBER] = pressure.target[HAND_CHANNEL_NUMBER];

        //Robot state
        if (robot_state[0] == 0 &&
            robot_state[1] == 2) {
            BasePressureLink();
        }
        else if (robot_state[0] == 0 &&
                 robot_state[1] == 3) {
            BasePressureActuator();
        }
        else if (robot_state[0] == 2 &&
                 robot_state[1] == 2) {
            FeedbackControl();
        }
        else if (robot_state[0] == 2 &&
                 robot_state[1] == 3) {
            FeedbackControl();
        }
        else if (robot_state[0] == 4 &&
                 robot_state[1] == 1) {
            FeedbackControl();
        }
        else if (robot_state[0] == 6 &&
                 robot_state[1] == 2) {
            FeedbackControl();
        }
        else if (robot_state[0] == 3 ||
                 robot_state[0] == 7) {
            PressurizeHand();
        }
        else if(robot_state[0] == 9) {
            Decompression();
            break;
        }

        //Publish InflatableStateFlag.msg
        pub_msg1.inflatable_state_flag = CheckRobotState();
        ros_pub1.publish(pub_msg1);

        //Publish VoltageOutput.msg
        pub_msg2.voltage_output.clear();
        pub_msg2.voltage_output.shrink_to_fit();
        for (int i = 0; i < DA_CHANNEL_NUMBER; i++) {
            pub_msg2.voltage_output.emplace_back(io.output.voltage[i]);
        }
        ros_pub2.publish(pub_msg2);
        loop_rate.sleep();

        //File output
        if (robot_state[0] != 0) {
            ofs << cycle << ','
		        << (float)timer.now_d.sec + (float)timer.now_d.nsec / 1e9 << ','
                << 100 * robot_state[0] + robot_state[1] << ','
                << flag.current_reach << ','
                << flag.target_reach << ','
                << pos.base.x[0] << ',' << pos.base.y[0] << ',' << pos.base.z[0] << ','
                << pos.current.x[3] << ',' << pos.current.y[3] << ',' << pos.current.z[3] << ','
                << pos.target.x[3] << ',' << pos.target.y[3] << ',' << pos.target.z[3] << ','
                << ornt.current.q[0] / M_PI * 180.0F << ',' << ornt.current.q[1] / M_PI * 180.0F << ',' << ornt.current.q[2] / M_PI * 180.0F << ',' << ornt.current.q[3] / M_PI * 180.0F << ',' << 90.0F - ornt.hand.q[0] / M_PI * 180.0F << ','
                << ornt.target.q[0] / M_PI * 180.0F << ',' << ornt.target.q[1] / M_PI * 180.0F << ',' << ornt.target.q[2] / M_PI * 180.0F << ',' << ornt.target.q[3] / M_PI * 180.0F << ',' << (ornt.offset.q[0] + ornt.current.q[1] + ornt.current.q[2] + ornt.current.q[3]) / M_PI * 180.0F << ','
                << param.force.x[0] << ',' << param.force.y[0] << ',' << param.force.z[0] << ','
                << param.torque[0] << ',' << param.torque[1] << ',' << param.torque[2] << ',' << param.torque[3] << ",";
            for (int i = 0; i < AD_CHANNEL_NUMBER; i++) { ofs << pressure.output[i] << ','; }
            ofs << std::endl;
        }

        cycle++;
    }

    return 0;
}

//Link pressurization
int BasePressureLink(void) {
    //Initialize base time
    if(base_pressure_link_flag == true) {
        base_time = ros::Time::now();
        base_pressure_link_flag = false;
    }

    now_time = ros::Time::now();

    //Link pressurization
    for (int i = 0; i < 3; i++) {
        pressure.target[LINK_CHANNEL_NUMBER + i] = pressure.base[LINK_CHANNEL_NUMBER + i];
        pressure.output[LINK_CHANNEL_NUMBER + i] = pressure.target[LINK_CHANNEL_NUMBER + i];
    }

    //Overpressure prevention
    CheckOverpressure();

    //Conversion from pressure to voltage
    DAconversion();

    return 0;
}

//Actuator pressurization
int BasePressureActuator(void) {
    //Initialize base time
    if(base_pressure_actuator_flag == true) {
        base_time = ros::Time::now();
        base_pressure_actuator_flag = false;
    }

    now_time = ros::Time::now();
    ros::Duration time_tmp = now_time - base_time;
    double pressurization_time = time_tmp.toSec();

    //Actuator pressurization
    for (int i = 0; i < DA_CHANNEL_NUMBER - 4; i++) {
        pressure.target[i] = pressure.base[i];
        pressure.output[i] = pressure.target[i];
        if ( (float)pressurization_time / ACTUATOR_PRESSURIZATION_TIME < 1.0F ) {
            pressure.output[i] = pressure.output[i] * ((float)pressurization_time / ACTUATOR_PRESSURIZATION_TIME);
        }
    }

    //Link pressurization
    for (int i = 0; i < 3; i++) {
        pressure.target[LINK_CHANNEL_NUMBER + i] = pressure.base[LINK_CHANNEL_NUMBER + i];
        pressure.output[LINK_CHANNEL_NUMBER + i] = pressure.target[LINK_CHANNEL_NUMBER + i];
    }

    //Overpressure prevention
    CheckOverpressure();

    //Conversion from pressure to voltage
    DAconversion();

    return 0;
}

//Hand pressurization
int PressurizeHand(void) {
    if (robot_state[0] == 3) {
        pressure.target[HAND_CHANNEL_NUMBER] = HAND_PRESSURE;
        pressure.output[HAND_CHANNEL_NUMBER] = pressure.target[HAND_CHANNEL_NUMBER];
    }
    else if (robot_state[0] == 7) {
        pressure.target[HAND_CHANNEL_NUMBER] = MINIMUM_PRESSURE + 1.0F;
        pressure.output[HAND_CHANNEL_NUMBER] = pressure.target[HAND_CHANNEL_NUMBER];
    }

    //Overpressure prevention
    CheckOverpressure();

    //Conversion from pressure to voltage
    DAconversion();

    return 0;
}

//Visual feedback control
int FeedbackControl(void) {
    int fine_pressure;
    
    //Check safety
    if (flag.current_reach == true && flag.target_reach == true) {
        safety_flag = true;
    }
    else {
        safety_flag = false;
    }
    
    //Feedback control
    //Diviation
    diviation.P.x[0] = pos.target.x[3] - pos.current.x[3];
    diviation.P.y[0] = pos.target.y[3] - pos.current.y[3];
    diviation.P.z[0] = pos.target.z[3] - pos.current.z[3];

    if (safety_flag == true) {
        if (visual_FB_cycle > 2) {
            for (fine_pressure = 0; fine_pressure < DA_CHANNEL_NUMBER; fine_pressure++) {
                if (flag.overpressure[fine_pressure] == false) {
                    break;
                }
            }

            if (fine_pressure == DA_CHANNEL_NUMBER) {
                diviation.I.x[0] += (diviation.P_buf.x[0] + diviation.P.x[0]) / (2.0F * SAMPLING_FREQUENCY);
                diviation.I.y[0] += (diviation.P_buf.y[0] + diviation.P.y[0]) / (2.0F * SAMPLING_FREQUENCY);
                diviation.I.z[0] += (diviation.P_buf.z[0] + diviation.P.z[0]) / (2.0F * SAMPLING_FREQUENCY);
            }
        }

        if (visual_FB_cycle != 0) {
            diviation.D.x[0] = diviation.P.x[0] - diviation.P_buf.x[0];
            diviation.D.y[0] = diviation.P.y[0] - diviation.P_buf.y[0];
            diviation.D.z[0] = diviation.P.z[0] - diviation.P_buf.z[0];
        }

        diviation.P_buf.x[0] = diviation.P.x[0];
        diviation.P_buf.y[0] = diviation.P.y[0];
        diviation.P_buf.z[0] = diviation.P.z[0];

        //Target force
        if (robot_state[0] == 4) {
            param.force.x[0] = k.visual_P / 2.0F * diviation.P.x[0] + k.visual_I / 2.0F * diviation.I.x[0] + k.visual_D * diviation.D.x[0];
            param.force.y[0] = k.visual_P / 2.0F * diviation.P.y[0] + k.visual_I / 2.0F * diviation.I.y[0] + k.visual_D * diviation.D.y[0];
            param.force.z[0] = k.visual_P / 2.0F * diviation.P.z[0] + k.visual_I / 2.0F * diviation.I.z[0] + k.visual_D * diviation.D.z[0];
        }
        else {
            param.force.x[0] = k.visual_P * diviation.P.x[0] + k.visual_I * diviation.I.x[0] + k.visual_D * diviation.D.x[0];
            param.force.y[0] = k.visual_P * diviation.P.y[0] + k.visual_I * diviation.I.y[0] + k.visual_D * diviation.D.y[0];
            param.force.z[0] = k.visual_P * diviation.P.z[0] + k.visual_I * diviation.I.z[0] + k.visual_D * diviation.D.z[0];
        }

        //For FB experiment
        param.force.z[0] = k.visual_P / 2.0F * diviation.P.z[0] + k.visual_I / 2.0F * diviation.I.z[0] + k.visual_D * diviation.D.z[0];

        //Jacobi matrix
        param.J[0][0] =
            -1 * param.link_u * sinf(ornt.offset.q[0]) * sinf(ornt.current.q[0]) 
            - param.link[1] * sinf(ornt.current.q[0]) * sinf(ornt.current.q[1] + ornt.offset.q[0]) 
            - param.link[2] * sinf(ornt.current.q[0]) * sinf(ornt.current.q[1] + ornt.current.q[2] + ornt.offset.q[0]) 
            - param.link[3] * sinf(ornt.current.q[0]) * sinf(ornt.current.q[1] + ornt.current.q[2] + ornt.current.q[3] + ornt.offset.q[0]);
        param.J[0][1] =
              param.link[1] * cosf(ornt.current.q[0]) * cosf(ornt.current.q[1] + ornt.offset.q[0]) 
            + param.link[2] * cosf(ornt.current.q[0]) * cosf(ornt.current.q[1] + ornt.current.q[2] + ornt.offset.q[0]) 
            + param.link[3] * cosf(ornt.current.q[0]) * cosf(ornt.current.q[1] + ornt.current.q[2] + ornt.current.q[3] + ornt.offset.q[0]);
        param.J[0][2] =
              param.link[2] * cosf(ornt.current.q[0]) * cosf(ornt.current.q[1] + ornt.current.q[2] + ornt.offset.q[0]) 
            + param.link[3] * cosf(ornt.current.q[0]) * cosf(ornt.current.q[1] + ornt.current.q[2] + ornt.current.q[3] + ornt.offset.q[0]);
        param.J[0][3] =
            param.link[3] * cosf(ornt.current.q[0]) * cosf(ornt.current.q[1] + ornt.current.q[2] + ornt.current.q[3] + ornt.offset.q[0]);

        param.J[1][0] =
              param.link_u * sinf(ornt.offset.q[0]) * cosf(ornt.current.q[0]) 
            + param.link[1] * cosf(ornt.current.q[0]) * sinf(ornt.current.q[1] + ornt.offset.q[0]) 
            + param.link[2] * cosf(ornt.current.q[0]) * sinf(ornt.current.q[1] + ornt.current.q[2] + ornt.offset.q[0]) 
            + param.link[3] * cosf(ornt.current.q[0]) * sinf(ornt.current.q[1] + ornt.current.q[2] + ornt.current.q[3] + ornt.offset.q[0]);
        param.J[1][1] =
              param.link[1] * sinf(ornt.current.q[0]) * cosf(ornt.current.q[1] + ornt.offset.q[0]) 
            + param.link[2] * sinf(ornt.current.q[0]) * cosf(ornt.current.q[1] + ornt.current.q[2] + ornt.offset.q[0]) 
            + param.link[3] * sinf(ornt.current.q[0]) * cosf(ornt.current.q[1] + ornt.current.q[2] + ornt.current.q[3] + ornt.offset.q[0]);
        param.J[1][2] =
              param.link[2] * sinf(ornt.current.q[0]) * cosf(ornt.current.q[1] + ornt.current.q[2] + ornt.offset.q[0]) 
            + param.link[3] * sinf(ornt.current.q[0]) * cosf(ornt.current.q[1] + ornt.current.q[2] + ornt.current.q[3] + ornt.offset.q[0]);
        param.J[1][3] =
            param.link[3] * sinf(ornt.current.q[0]) * cosf(ornt.current.q[1] + ornt.current.q[2] + ornt.current.q[3] + ornt.offset.q[0]);

        param.J[2][0] = 0;
        param.J[2][1] =
            -1 * param.link[1] * sinf(ornt.current.q[1] + ornt.offset.q[0]) 
            - param.link[2] * sinf(ornt.current.q[1] + ornt.current.q[2] + ornt.offset.q[0]) 
            - param.link[3] * sinf(ornt.current.q[1] + ornt.current.q[2] + ornt.current.q[3] + ornt.offset.q[0]);
        param.J[2][2] =
            -1 * param.link[2] * sinf(ornt.current.q[1] + ornt.current.q[2] + ornt.offset.q[0]) 
            - param.link[3] * sinf(ornt.current.q[1] + ornt.current.q[2] + ornt.current.q[3] + ornt.offset.q[0]);
        param.J[2][3] =
            -1 * param.link[3] * sinf(ornt.current.q[1] + ornt.current.q[2] + ornt.current.q[3] + ornt.offset.q[0]);

        //Inverse Jacobian matrix
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < DEGREE_OF_FREEDOM; j++) {
                param.J_T[j][i] = param.J[i][j];
            }
        }
        //Force - torque conversion
        for (int i = 0; i < DEGREE_OF_FREEDOM; i++) {
            param.torque[i] = param.J_T[i][0] * param.force.x[0] 
                            + param.J_T[i][1] * param.force.y[0] 
                            + param.J_T[i][2] * param.force.z[0];
        }

        //Gravity compensation
        CompensateGravity();

        for (int i = 0; i < DEGREE_OF_FREEDOM; i++) {
            param.torque[i] += param.torque_g_rate * param.torque_g[i];
        }

        //Coefficient for torque - pressure conversion
        k.torque[0] = 30.0F * (M_PI / 2.0F + param.current_yaw) - 14.0F; //M_PI / 2.0F is default angle of actuator
        k.torque[1] = 30.0F * (M_PI / 2.0F - param.current_yaw) - 14.0F;
        k.torque[2] = 30.0F * (M_PI + ornt.current.q[1]) - 14.0F;        //M_PI is default angle of actuator
        k.torque[3] = 30.0F * (M_PI - ornt.current.q[1]) - 14.0F;
        k.torque[4] = 30.0F * (M_PI + ornt.current.q[2]) - 14.0F;
        k.torque[5] = 30.0F * (M_PI - ornt.current.q[2]) - 14.0F;
        k.torque[6] = 30.0F * (M_PI + ornt.current.q[3]) - 14.0F;
        k.torque[7] = 30.0F * (M_PI - ornt.current.q[3]) - 14.0F;

        //Torque - pressure conversion
        pressure.target[0] = pressure.base[0] - k.torque[0] * param.torque[0] / 2.0F * 1.25F; //1.25F : Gain supplement
        pressure.target[1] = pressure.base[1] + k.torque[1] * param.torque[0] / 2.0F * 1.25F; //1.25F : Gain supplement
        pressure.target[2] = pressure.base[2] + k.torque[2] * param.torque[1] / 2.0F;
        pressure.target[3] = pressure.base[3] - k.torque[3] * param.torque[1] / 2.0F;
        pressure.target[4] = pressure.base[4] + k.torque[4] * param.torque[2] / 2.0F;
        pressure.target[5] = pressure.base[5] - k.torque[5] * param.torque[2] / 2.0F;
        pressure.target[6] = pressure.base[6] + k.torque[6] * param.torque[3] / 2.0F;
        pressure.target[7] = pressure.base[7] - k.torque[7] * param.torque[3] / 2.0F;
        pressure.target[LINK_CHANNEL_NUMBER] = pressure.base[LINK_CHANNEL_NUMBER];
        pressure.target[LINK_CHANNEL_NUMBER + 1] = pressure.base[LINK_CHANNEL_NUMBER + 1];
        pressure.target[LINK_CHANNEL_NUMBER + 2] = pressure.base[LINK_CHANNEL_NUMBER + 2];

        for (int i = 0; i < AD_CHANNEL_NUMBER; i++) {
            pressure.output[i] = pressure.target[i];
        }

        //Overpressure prevention
        CheckOverpressure();

        //Conversion from pressure to voltage
        DAconversion();

        safety_flag = 0;
        visual_FB_cycle++;
    }
    
    return 0;
}

//Calculate compensate gravity torque
int CompensateGravity() {
    float m[DEGREE_OF_FREEDOM];   //Mass summation of link and actuator
	float x_l[DEGREE_OF_FREEDOM]; //Link center of gravity
	float y_l[DEGREE_OF_FREEDOM];
	float z_l[DEGREE_OF_FREEDOM];
	float x_g[DEGREE_OF_FREEDOM]; //Link and actuator center of gravity
	float y_g[DEGREE_OF_FREEDOM];
	float z_g[DEGREE_OF_FREEDOM];

    if (robot_state[0] == 4 ||
        robot_state[0] == 5 ||
        robot_state[0] == 6) {
        param.m_a[3] = BOTTLE_WEIGHT;
    }
    else {
        param.m_a[3] = 0.0F;
    }

    //For FB experiment
    param.m_a[3] = BOTTLE_WEIGHT;


    x_l[1] = pos.target.x[0] + (1.0F / 2.0F) * param.link[1] * sinf(ornt.current.q[1] + ornt.offset.q[0]) * cosf(ornt.current.q[0]);
	y_l[1] = pos.target.y[0] + (1.0F / 2.0F) * param.link[1] * sinf(ornt.current.q[1] + ornt.offset.q[0]) * sinf(ornt.current.q[0]);
	z_l[1] = pos.target.z[0] + (1.0F / 2.0F) * param.link[1] * cosf(ornt.current.q[1] + ornt.offset.q[0]);
	x_l[2] = pos.target.x[1] + (1.0F / 2.0F) * param.link[2] * sinf(ornt.current.q[1] + ornt.current.q[2] + ornt.offset.q[0]) * cosf(ornt.current.q[0]);
	y_l[2] = pos.target.y[1] + (1.0F / 2.0F) * param.link[2] * sinf(ornt.current.q[1] + ornt.current.q[2] + ornt.offset.q[0]) * sinf(ornt.current.q[0]);
	z_l[2] = pos.target.z[1] + (1.0F / 2.0F) * param.link[2] * cosf(ornt.current.q[1] + ornt.current.q[2] + ornt.offset.q[0]);
	x_l[3] = pos.target.x[2] + (1.0F / 2.0F) * param.link[3] * sinf(ornt.current.q[1] + ornt.current.q[2] + ornt.current.q[3] + ornt.offset.q[0]) * cosf(ornt.current.q[0]);
	y_l[3] = pos.target.y[2] + (1.0F / 2.0F) * param.link[3] * sinf(ornt.current.q[1] + ornt.current.q[2] + ornt.current.q[3] + ornt.offset.q[0]) * sinf(ornt.current.q[0]);
	z_l[3] = pos.target.z[2] + (1.0F / 2.0F) * param.link[3] * cosf(ornt.current.q[1] + ornt.current.q[2] + ornt.current.q[3] + ornt.offset.q[0]);

	for (int i = 0; i < DEGREE_OF_FREEDOM; i++) {
		x_g[i] = (param.m_l[i] * x_l[i] + param.m_a[i] * pos.current.x[i]) / (param.m_l[i] + param.m_a[i]);
		y_g[i] = (param.m_l[i] * y_l[i] + param.m_a[i] * pos.current.y[i]) / (param.m_l[i] + param.m_a[i]);
		z_g[i] = (param.m_l[i] * z_l[i] + param.m_a[i] * pos.current.z[i]) / (param.m_l[i] + param.m_a[i]);
		m[i] = param.m_l[i] + param.m_a[i];
		param.link_g[i] = GetDistance(x_g[i], y_g[i], z_g[i], pos.current.x[i], pos.current.y[i], pos.current.z[i]);
	}

	param.torque_g[0] = 0.0F;
	param.torque_g[1] = (-1 * m[1] * GRAVITY_ACC * param.link_g[1] - m[2] * GRAVITY_ACC * param.link[1] - m[3] * GRAVITY_ACC * param.link[1]) * sinf(ornt.current.q[1] + ornt.offset.q[0])
		+ (-1 * m[2] * GRAVITY_ACC * param.link_g[2] - m[3] * GRAVITY_ACC * param.link[2]) * sinf(ornt.current.q[1] + ornt.current.q[2] + ornt.offset.q[0])
		+ (-1 * m[3] * GRAVITY_ACC * param.link_g[3]) * sinf(ornt.current.q[1] + ornt.current.q[2] + ornt.current.q[3] + ornt.offset.q[0]);
	param.torque_g[2] = (-1 * m[2] * GRAVITY_ACC * param.link_g[2] - m[3] * GRAVITY_ACC * param.link[2]) * sinf(ornt.current.q[1] + ornt.current.q[2] + ornt.offset.q[0])
		+ (-1 * m[3] * GRAVITY_ACC * param.link_g[3]) * sinf(ornt.current.q[1] + ornt.current.q[2] + ornt.current.q[3] + ornt.offset.q[0]);
	param.torque_g[3] = (-1 * m[3] * GRAVITY_ACC * param.link_g[3]) * sinf(ornt.current.q[1] + ornt.current.q[2] + ornt.current.q[3] + ornt.offset.q[0]);

    return 0;
}

//Check robot state
int CheckRobotState(void) {
    int convergence_flag = 0;
    float target_diviation = 0.012F; //[m]

    if ((robot_state[0] == 2 && robot_state[1] == 2) ||
        (robot_state[0] == 2 && robot_state[1] == 3) ||
        (robot_state[0] == 4 && robot_state[1] == 1) ||
        (robot_state[0] == 6 && robot_state[1] == 2)) {
        if (std::abs(diviation.P.x[0]) < target_diviation && 
            std::abs(diviation.P.y[0]) < target_diviation && 
            std::abs(diviation.P.z[0]) < target_diviation) {
            convergence_flag = 1;
            ros::Duration(0.1).sleep();
        }
        else {
            convergence_flag = 0;
        }
    }

    //For FB experiment
    convergence_flag = 0;
    return convergence_flag;
}

//Distance between two points
float GetDistance(float x1, float y1, float z1, float x2, float y2, float z2) {
	float distance;
	distance = pow((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) + (z2 - z1) * (z2 - z1), 0.5F);

	return distance;
}

//Check overpressure
int CheckOverpressure(void) {
	for (int i = 0; i < DA_CHANNEL_NUMBER - 4; i++) {
		if (pressure.output[i] > MAXIMUM_PRESSURE) {
			pressure.output[i] = MAXIMUM_PRESSURE;
			flag.overpressure[i] = false;
			ROS_WARN("Over maximum pressure %d\n", i);
		}
		else {
			flag.overpressure[i] = true;
		}
	}

    for (int i = LINK_CHANNEL_NUMBER; i < LINK_CHANNEL_NUMBER + 2; i++) {
        if (pressure.output[i] > MAXIMUM_LINK_PRESSURE) {
            pressure.output[i] = MAXIMUM_LINK_PRESSURE;
            flag.overpressure[i] = false;
            ROS_WARN("Over maximum pressure %d\n", LINK_CHANNEL_NUMBER);
        }
        else {
            flag.overpressure[i] = true;
        }
    }

	if (pressure.output[HAND_CHANNEL_NUMBER] > MAXIMUM_HAND_PRESSURE) {
		pressure.output[HAND_CHANNEL_NUMBER] = MAXIMUM_HAND_PRESSURE;
		flag.overpressure[HAND_CHANNEL_NUMBER] = false;
		ROS_WARN("Over maximum pressure %d\n", HAND_CHANNEL_NUMBER);
	}
	else {
		flag.overpressure[HAND_CHANNEL_NUMBER] = true;
	}

	for (int i = 0; i < DA_CHANNEL_NUMBER; i++) {
		if (pressure.output[i] < MINIMUM_PRESSURE) {
			pressure.output[i] = MINIMUM_PRESSURE;
			flag.overpressure[i] = false;
			ROS_WARN("Under minimum pressure %d\n", i);
		}
		else {
			flag.overpressure[i] = true;
		}
	}

	return 0;
}

//Pressure - voltage conversion
int DAconversion() {
	for (int i = 0; i < DA_CHANNEL_NUMBER; i++) {
		io.output.voltage[i] = k.DA_a[i] * pressure.output[i] * pressure.output[i]
                			 + k.DA_b[i] * pressure.output[i]
                			 + k.DA_c[i];
	}
	return 0;
}

//Voltage - pressure conversion
int ADconversion() {
	for (int i = 0; i < AD_CHANNEL_NUMBER; i++) {
		pressure.input[i] = k.AD_a[i] * io.input.voltage[i] * io.input.voltage[i]
		                  + k.AD_b[i] * io.input.voltage[i]
			              + k.AD_c[i];
	}
	return 0;
}

//Decompression
int Decompression(void) {
    for (int i = 0; i < DA_CHANNEL_NUMBER; i++) {
		io.output.voltage[i] = 0.0F;
	}

    //Overpressure prevention
    CheckOverpressure();

    //Conversion from pressure to voltage
    DAconversion();

    return 0;
}
