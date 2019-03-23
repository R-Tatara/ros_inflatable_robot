/*++

Name : io_board.cpp

Abstract : IO board driver for an inflatable robot

Author : Ryosuke Tatara

--*/

#include "ros/ros.h"
#include <vector>
#include <string>
//Publishing
#include "inflatable_robot/VoltageInput.h"
//Subscribing
#include "inflatable_robot/InflatableState.h"
#include "inflatable_robot/VoltageOutput.h"

#include "caio.h"

#define DA_CHANNEL_NUMBER 12
#define AD_CHANNEL_NUMBER 8
#define CUTOFF_FREQUENCY 10.0F
#define SAMPLING_FREQUENCY 200


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

class SingleAio {
public:
	long Ret;
    char DeviceName[256];
	char ErrorString[256];
	short Id;
    short Range;
    std::vector<float> Data;
	SingleAio();
	~SingleAio();
};

SingleAio::SingleAio()
	: Range(0)
{
}

SingleAio::~SingleAio() {
    AioExit(10); //Ao.Id = 10
    AioExit(11); //Ai.Id = 11
}

class IIR {
public:
	const float f0; //Cutoff frequency
	const float Q;  //Quality factor
	float fs;       //Sampling frequency
	float ts;       //Sampling time
	float buf1[AD_CHANNEL_NUMBER];
	float buf2[AD_CHANNEL_NUMBER];
	float w0;
	float a;
	float b0;
	float b1;
	float b2;
	float a1;
	float a2;
	void setIIRparam(void);
	IIR();
	~IIR();
};

void IIR::setIIRparam() {
	this->w0 = tanf(M_PI * this->f0 / this->fs);
	this->a = sinf(this->w0) / this->Q;
	this->b0 = (1.0F - cosf(this->w0)) / (2.0F * (1.0F + this->a));
	this->b1 = (1.0F - cosf(this->w0)) / (1.0F + this->a);
	this->b2 = (1.0F - cosf(this->w0)) / (2.0F * (1.0F + this->a));
	this->a1 = (2.0F * cosf(this->w0)) / (1.0F + this->a);
	this->a2 = (this->a - 1.0F) / (this->a + 1.0F);
}

IIR::IIR()
	:f0(CUTOFF_FREQUENCY), //must be less than fs/2
	Q(sqrtf(2.0F) / 2.0F),
    fs(SAMPLING_FREQUENCY),
	ts(1.0F / SAMPLING_FREQUENCY),
	buf1{ 256.0F, 256.0F, 256.0F, 256.0F, 256.0F, 256.0F, 256.0F, 256.0F },
	buf2{ 256.0F, 256.0F, 256.0F, 256.0F, 256.0F, 256.0F, 256.0F, 256.0F },
    w0(0.0F), a(0.0F), b0(0.0F), b1(0.0F), b2(0.0F), a1(0.0F), a2(0.0F)
{
}

IIR::~IIR() {
}

//Instanciation
Time timer;
SingleAio Ao;
SingleAio Ai;
IIR iir;

//Global variable
int cycle = 0;
int robot_state[2]; //[0] : Higher order state, [1] : Lower order state

//Prototype declaration
int InitAi(void);
int InitAo(void);
int AnalogOutput(void);
int AnalogInput(void);
int AioTermination(void);
float LowpassFilter(float vin, int i);

//Subscribe InflatableState.msg
void msgCallback1(const inflatable_robot::InflatableState::ConstPtr& sub_msg1) {
    int state_tmp = sub_msg1->inflatable_state;
    robot_state[1] = state_tmp % 100;
    state_tmp /= 100;
    robot_state[0] = state_tmp % 10;
}

//Subscribe VoltageOuput.msg
void msgCallback2(const inflatable_robot::VoltageOutput::ConstPtr& sub_msg2) {
	for (int i = 0; i < DA_CHANNEL_NUMBER; i++) {
		Ao.Data[i] = sub_msg2->voltage_output[i];
	}
    AnalogOutput();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "io_board");
    ros::NodeHandle nh;
    ros::Publisher ros_pub = nh.advertise<inflatable_robot::VoltageInput>("voltage_input", 10);
    ros::Subscriber ros_sub1 = nh.subscribe("inflatable_state", 10, msgCallback1);
    ros::Subscriber ros_sub2 = nh.subscribe("voltage_output", 10, msgCallback2);
    ros::Rate loop_rate(SAMPLING_FREQUENCY);
    inflatable_robot::VoltageInput pub_msg;
    timer.base = ros::Time::now();
    timer.buf = ros::Time::now();
    InitAo();
    InitAi();
	iir.setIIRparam();

    while (ros::ok()) {
        //Time processing
        timer.now_t = ros::Time::now();
        timer.now_d = timer.now_t - timer.base;
        timer.sampling = timer.now_t - timer.buf;
		timer.buf = timer.now_t;
		iir.ts = (float)timer.sampling.sec + (float)timer.sampling.nsec / 1e9;
		ros::spinOnce();

		AnalogInput();

		//Lowpass filter
		for (int i = 0; i < AD_CHANNEL_NUMBER; i++) {
			Ai.Data[i] = LowpassFilter(Ai.Data[i], i);
		}

		pub_msg.voltage_input.clear();
        pub_msg.voltage_input.shrink_to_fit();
        for (int i = 0; i < AD_CHANNEL_NUMBER; i++) {
            pub_msg.voltage_input.emplace_back(Ai.Data[i]);
        }
        ros_pub.publish(pub_msg);
        loop_rate.sleep();
        cycle++;

        if(robot_state[0] == 9) {
			for (int i = 0; i < DA_CHANNEL_NUMBER; i++) {
				Ao.Data[i] = 0.0F;
			}
			AnalogOutput();
			AioTermination();
			break;
        }
    }

    return 0;
}

//Initialize Ao board AO-1616L-LPE
int InitAo(void) {
	for (int i = 0; i < DA_CHANNEL_NUMBER; i++) {
		Ao.Data.emplace_back(0.0F);
	}
    strcpy(Ao.DeviceName, "FFRBW09000263"); // /proc/contec_aio.config
	Ao.Ret = AioInit(Ao.DeviceName, &Ao.Id);
	if(Ao.Ret != 0){
		AioGetErrorString(Ao.Ret, Ao.ErrorString);
		ROS_ERROR("AoInit = %ld : %s", Ao.Ret, Ao.ErrorString);
		exit(0);
	}

    Ao.Ret = AioSetAoRangeAll(Ao.Id, Ao.Range);
	if(Ao.Ret != 0){
		AioGetErrorString(Ao.Ret, Ao.ErrorString);
		ROS_ERROR("AioSetAoRangeAll = %ld : %s", Ao.Ret, Ao.ErrorString);
		Ao.Ret = AioExit(Ao.Id);
		exit(0);
	}

    return 0;
}


//Initialize Ai board AI-1616LI-PE
int InitAi(void) {
	for (int i = 0; i < AD_CHANNEL_NUMBER; i++) {
		Ai.Data.emplace_back(0.0F);
	}
    strcpy(Ai.DeviceName, "FFRBW06000318"); // /proc/contec_aio.config
    Ai.Ret = AioInit(Ai.DeviceName, &Ai.Id);
	if(Ai.Ret != 0){
		AioGetErrorString(Ai.Ret, Ai.ErrorString);
		ROS_ERROR("AiInit = %ld : %s", Ai.Ret, Ai.ErrorString);
		exit(0);
	}

    Ai.Ret = AioSetAiRangeAll(Ai.Id, Ai.Range);
	if(Ai.Ret != 0){
		AioGetErrorString(Ai.Ret, Ai.ErrorString);
		ROS_ERROR("AioSetAiRangeAll = %ld : %s", Ai.Ret, Ai.ErrorString);
		Ai.Ret = AioExit(Ai.Id);
		exit(0);
	}

    return 0;
}

//Analog output
int AnalogOutput(void) {
	for (int i = 0; i < DA_CHANNEL_NUMBER; i++) {
		Ao.Ret = AioSingleAoEx(Ao.Id, i, Ao.Data[i]);
		if (Ao.Ret != 0) {
			AioGetErrorString(Ao.Ret, Ao.ErrorString);
            ROS_ERROR("AioSingleAoEx = %ld : %s", Ao.Ret, Ao.ErrorString);
            Ao.Ret = AioExit(Ao.Id);
            exit(0);
		}
	}

	return 0;
}

//Analog input
int AnalogInput(void) {
	for (int i = 0; i < AD_CHANNEL_NUMBER; i++) {
		Ai.Ret = AioSingleAiEx(Ai.Id, i, &Ai.Data[i]);
		if (Ai.Ret != 0) {
			AioGetErrorString(Ai.Ret, Ai.ErrorString);
            ROS_ERROR("AioSingleAiEx = %ld : %s", Ai.Ret, Ai.ErrorString);
            Ai.Ret = AioExit(Ai.Id);
            exit(0);
		}
	}

	return 0;
}

//AioTemination
int AioTermination(void) {
    Ao.Ret = AioExit(Ao.Id);
	if(Ao.Ret != 0){
		AioGetErrorString(Ao.Ret, Ao.ErrorString);
		ROS_ERROR("AoExit = %ld : %s", Ao.Ret, Ao.ErrorString);
	}

    Ai.Ret = AioExit(Ai.Id);
	if(Ai.Ret != 0){
		AioGetErrorString(Ai.Ret, Ai.ErrorString);
		ROS_ERROR("AiExit = %ld : %s", Ai.Ret, Ai.ErrorString);
	}

	return 0;
}

float LowpassFilter(float vin, int i) {
	float reg[AD_CHANNEL_NUMBER];
	float vout;

	reg[i] = vin + iir.a1 * iir.buf1[i] + iir.a2 * iir.buf2[i];
	vout = iir.b0 * reg[i] + iir.b1 * iir.buf1[i] + iir.b2 * iir.buf2[i];
	iir.buf2[i] = iir.buf1[i];
	iir.buf1[i] = reg[i];
	return vout;
}
