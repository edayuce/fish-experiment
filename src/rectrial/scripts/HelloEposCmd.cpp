//============================================================================
// Name        : HelloEposCmd.cpp
// Author      : Dawid Sienkiewicz
// Version     :
// Copyright   : maxon motor ag 2014-2021
// Description : Hello Epos in C++
//============================================================================
#include <ros/ros.h>
#include <rectrial/pub_data.h>
#include "std_msgs/String.h"
#include <iostream>
#include "Definitions.h"
#include <chrono>
#include <ctime>
#include <string.h>
#include <sstream>
#include <unistd.h>
#include <getopt.h>
#include <stdlib.h>
#include <stdio.h>
#include <list>
#include <math.h>
#include <sys/types.h>
#include <stdio.h>
#include <sys/times.h>
#include <sys/time.h>
#include <chrono>
#include <cmath>
#include <thread>
#include <signal.h>
#include <fstream>
#include <algorithm>

#include <vector>
#include <fstream>

typedef void* HANDLE;
typedef int BOOL;
std::string video_path_name;
std::ofstream myfile;
enum EAppMode
{
	AM_UNKNOWN,
	AM_DEMO,
	AM_INTERFACE_LIST,
	AM_PROTOCOL_LIST,
	AM_VERSION_INFO
};

using namespace std;

void* g_pKeyHandle = 0;
unsigned short g_usNodeId = 1;
string g_deviceName;
string g_protocolStackName;
string g_interfaceName;
string g_portName;
int g_baudrate = 0;
EAppMode g_eAppMode = AM_DEMO;

const string g_programName = "HelloEposCmd";

// CSV Variables
volatile int csvIndex;
std::vector<float> positions;
string folder_name = "electro";
float fish_direction = 1.0;
// CSV Variables END
bool OpenLoop = false;
bool ClosedLoop = false;
// Gain Variables
double reafferentGain=0;
bool GainMode = false;
double limit = 10.0;
double pos_x = 0.0;
// Gain Variables END
float low = -0.5;
int high = 0.5;
//Sine Movement Variables
bool manualControl = false;
bool sumofsines=true;
double frequency = 2.05;
double period = 1/frequency;
double AmpMM = 10.0;
double MMperREV = 10.0; //Hatve
double encoderCount = 512;
// double encoderCount = 500;
double gearRatio = (624.0/35.0)*4.0;
// double gearRatio = (26.0)*4;
double countPerREV = encoderCount*gearRatio;
double countPerMM = countPerREV/MMperREV;
double A_pos = countPerMM * AmpMM; 
double REVperSEC = AmpMM/MMperREV;
double RPMGear = REVperSEC*60.0;
double RPMMotor = RPMGear * (624.0/35.0);

double freqs[13] = {0.1,0.15,0.25,0.35,0.55,0.65,0.85,0.95,1.15,1.45,1.55,1.85,2.05};
double posPH[13]={1.7413,0.9611,2.2505,0.8712,0.9909,0.2849,1.7916,0.5468,0.9560,1.1420,0.4083,0.9600,2.0160};
double velPH[13]={0.2162,2.5980,0.0991,0.2986,2.9446,2.5794,2.8213,0.0693,1.4556,0.3746,1.0430,2.9469,2.6706};
double phase[13] = {1.7304, 2.2247, 0.9139, 1.6048, 2.8053, 2.8158, 0.3945, 0.6511, 0.1617, 1.3848, 0.0939, 1.4352, 2.0393};

double time_ms = 0;
long targetposition = 0;
long oldtargetposition=0;
long targetVelocity = 0;
long targetAcc = 0;
double passTime_ms = 42;

std::chrono::steady_clock::time_point beginSelf;
std::chrono::steady_clock::time_point beginLoop;
//Sine Movement Variables END
int myCounter =0;
int totalFrameNumber =0;

#ifndef MMC_SUCCESS
	#define MMC_SUCCESS 0
#endif

#ifndef MMC_FAILED
	#define MMC_FAILED 1
#endif

#ifndef MMC_MAX_LOG_MSG_SIZE
	#define MMC_MAX_LOG_MSG_SIZE 512
#endif


void  LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode);
void  LogInfo(string message);
void  PrintSettings();
int   OpenDevice(unsigned int* p_pErrorCode);
int   CloseDevice(unsigned int* p_pErrorCode);
void  SetDefaultParameters();
int   PrepareDemo(unsigned int* p_pErrorCode);

ofstream positionFile;
ros::Subscriber sub;


void LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode)
{
	cerr << g_programName << ": " << functionName << " failed (result=" << p_lResult << ", errorCode=0x" << std::hex << p_ulErrorCode << ")"<< endl;
}

void LogInfo(string message)
{
	cout << message << endl;
}

void SeparatorLine()
{
	const int lineLength = 65;
	for(int i=0; i<lineLength; i++)
	{
		cout << "-";
	}
	cout << endl;
}

void PrintSettings()
{
	stringstream msg;

	msg << "default settings:" << endl;
	msg << "node id             = " << g_usNodeId << endl;
	msg << "device name         = '" << g_deviceName << "'" << endl;
	msg << "protocol stack name = '" << g_protocolStackName << "'" << endl;
	msg << "interface name      = '" << g_interfaceName << "'" << endl;
	msg << "port name           = '" << g_portName << "'"<< endl;
	msg << "baudrate            = " << g_baudrate;

	LogInfo(msg.str());

	SeparatorLine();
}

void SetDefaultParameters()
{
	//USB
	g_usNodeId = 1;
	g_deviceName = "EPOS4"; 
	g_protocolStackName = "MAXON SERIAL V2"; 
	g_interfaceName = "USB"; 
	g_portName = "USB0"; 
	g_baudrate = 1000000; 
}

// void SetDefaultParameters()
// {
// 	//USB
// 	g_usNodeId = 1;
// 	g_deviceName = "EPOS4"; 
// 	g_protocolStackName = "CANopen"; 
// 	g_interfaceName = "CAN_mttcan 0"; 
// 	g_portName = "CAN0"; 
// 	g_baudrate = 500000; 
// }

int OpenDevice(unsigned int* p_pErrorCode)
{
		
	int lResult = MMC_FAILED;

	char* pDeviceName = new char[255];
	char* pProtocolStackName = new char[255];
	char* pInterfaceName = new char[255];
	char* pPortName = new char[255];

	strcpy(pDeviceName, g_deviceName.c_str());
	strcpy(pProtocolStackName, g_protocolStackName.c_str());
	strcpy(pInterfaceName, g_interfaceName.c_str());
	strcpy(pPortName, g_portName.c_str());

	LogInfo("Open device...");

	g_pKeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, p_pErrorCode);
	//bool res = VCS_SetGatewaySettings(g_pKeyHandle, 115200, p_pErrorCode);
	if(g_pKeyHandle!=0 && *p_pErrorCode == 0)
	{
		unsigned int lBaudrate = 0;
		unsigned int lTimeout = 0;

		if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
		{
			if(VCS_SetProtocolStackSettings(g_pKeyHandle, g_baudrate, lTimeout, p_pErrorCode)!=0)
			{
				if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
				{
					if(g_baudrate==(int)lBaudrate)
					{
						lResult = MMC_SUCCESS;
					}
				}
			}
		}
	}
	else
	{
		g_pKeyHandle = 0;
	}

	delete []pDeviceName;
	delete []pProtocolStackName;
	delete []pInterfaceName;
	delete []pPortName;

	return lResult;
}

int CloseDevice(unsigned int* p_pErrorCode)
{
	int lResult = MMC_FAILED;

	*p_pErrorCode = 0;

	LogInfo("Close device");

	if(VCS_CloseDevice(g_pKeyHandle, p_pErrorCode)!=0 && *p_pErrorCode == 0)
	{
		lResult = MMC_SUCCESS;
	}

	return lResult;
}


volatile sig_atomic_t stop;
void inthand(int signum) {
    stop = 1;
}

int PrepareDemo(unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	BOOL oIsFault = 0;

	if(VCS_GetFaultState(g_pKeyHandle, g_usNodeId, &oIsFault, p_pErrorCode ) == 0)
	{
		LogError("VCS_GetFaultState", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}

	if(lResult==0)
	{
		if(oIsFault)
		{
			stringstream msg;
			msg << "clear fault, node = '" << g_usNodeId << "'";
			LogInfo(msg.str());

			if(VCS_ClearFault(g_pKeyHandle, g_usNodeId, p_pErrorCode) == 0)
			{
				LogError("VCS_ClearFault", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}
		}

		if(lResult==0)
		{
			BOOL oIsEnabled = 0;

			if(VCS_GetEnableState(g_pKeyHandle, g_usNodeId, &oIsEnabled, p_pErrorCode) == 0)
			{
				LogError("VCS_GetEnableState", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}

			if(lResult==0)
			{
				if(!oIsEnabled)
				{
					if(VCS_SetEnableState(g_pKeyHandle, g_usNodeId, p_pErrorCode) == 0)
					{
						LogError("VCS_SetEnableState", lResult, *p_pErrorCode);
						lResult = MMC_FAILED;
					}
				}
			}
		}
	}
	return lResult;
}

std::vector<float> read_record(string path)
{

	// File pointer
	std::fstream fin;

	// Open an existing file
	fin.open(path, std::ios::in);

	// Read the Data from the file
	// as String Vector
	std::vector<float> positions;
	std::string temp;

	while (fin >> temp) {

        
        positions.push_back(stof(temp));
        
	}
	// for(int i=0;i<positions.size();i++){
    //     cout << positions[i] << " \n";
    // }

    // cout << positions.size()<< endl;
    return positions;
}

bool initializeMotor(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode){
	
	int lResult = MMC_SUCCESS;
	stringstream msg;
	unsigned short motorType, pNominalCurrent, pMaxOutputCurrent, pThermalTimeConstant;
	unsigned int errorCode;
	bool motorTypeRead;
	bool parameters;
	unsigned int pEncoderResolution;
	int pInvertedPolarity;
	

	motorTypeRead = VCS_GetMotorType(p_DeviceHandle, p_usNodeId, &motorType, &errorCode);
	if(motorTypeRead!= 0){
		cout << "Motor Type: " << motorType << endl;

	}else{
		cout << "Error when reading motor type! :" << errorCode << endl;
	}
	string freqStr = to_string(frequency);
	replace(freqStr.begin(), freqStr.end(), '.', '_');
	if(sumofsines) freqStr="sum";

	cout << "Gear Ratio: " << gearRatio << endl;
	cout << "Frequency: " << frequency << endl;

	cout << "Encoder Count per MM of Output Gear: " << countPerMM << endl;
	cout << "A POS: " << A_pos << endl;
	cout << "RPM Motor: " << A_pos << endl;


	msg << "set profile position mode, node = " << p_usNodeId;
	LogInfo(msg.str());

	if(VCS_ActivateProfilePositionMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
	{
		LogError("VCS_ActivateProfilePositionMode", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}
	else
	{

		parameters = VCS_GetDcMotorParameter(p_DeviceHandle, p_usNodeId, &pNominalCurrent, &pMaxOutputCurrent, &pThermalTimeConstant, &errorCode);

		if(parameters!= 0){
			cout << "NominalCurrent: " << pNominalCurrent << endl;
			cout << "MaxOutputCurrent: " << pMaxOutputCurrent << endl;
			cout << "ThermalTimeConstant: " << pThermalTimeConstant << endl;

		}else{
			cout << "Error when reading parameters! :" << errorCode << endl;
		}
		
		parameters = VCS_GetIncEncoderParameter(p_DeviceHandle, p_usNodeId, &pEncoderResolution, &pInvertedPolarity, &errorCode);

		if(parameters!= 0){
			cout << "pEncoderResolution: " << pEncoderResolution << endl;
			cout << "pInvertedPolarity: " << pInvertedPolarity << endl;
		

		}else{
			cout << "Error when reading parameters! :" << errorCode << endl;
		}
		
		//VCS_GetPositionProfile(p_DeviceHandle, p_usNodeId, &pNominalCurrent, &pMaxOutputCurrent, &pThermalTimeConstant, &errorCode)
		bool check;

		if(VCS_SetPositionProfile(p_DeviceHandle, p_usNodeId, 5000, 90000, 90000, &p_rlErrorCode) == 0)
		{
			LogError("VCS_MoveToPosition", lResult, p_rlErrorCode);
			lResult = MMC_FAILED;
			return lResult;
		}
		
		if(VCS_MoveToPosition(p_DeviceHandle, p_usNodeId, 0, 1, 1, &p_rlErrorCode) == 0)
		{
			LogError("VCS_MoveToPosition", lResult, p_rlErrorCode);
			lResult = MMC_FAILED;
			
		}
	}

	usleep(1);

	return lResult;
}




void imageCallback(const rectrial::pub_data::ConstPtr& msg){

	double targetpositionFloat =0;
	double targetVelocityFloat =0;
	double targetAccFloat =0;

	int lResult = MMC_SUCCESS;

	auto p_DeviceHandle = g_pKeyHandle;
	unsigned short p_usNodeId = g_usNodeId;
	unsigned int lErrorCode = 0;
	unsigned int & p_rlErrorCode = lErrorCode;

	totalFrameNumber++;

	ROS_INFO("Callback function in HelloEposCmd");

	if(msg->finish_c=="start"){
		beginSelf = std::chrono::steady_clock::now();
		beginLoop = std::chrono::steady_clock::now();
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		positions.clear();
		positions = read_record("/home/neurolab/Desktop/Experiment_Materials/openloop_refuge_csv/refuge_f01_n0.csv");
	
		csvIndex =0;
	}
	if(msg->finish_c=="end"){
		if(lResult == MMC_SUCCESS)
		{
			LogInfo("halt velocity movement");

			if(VCS_HaltVelocityMovement(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
			{

				targetVelocity = 2700;

				targetAcc = 5000;

				if(VCS_SetPositionProfile(p_DeviceHandle, p_usNodeId, targetVelocity, targetAcc, targetAcc, &p_rlErrorCode) == 0)
				{
					LogError("VCS_MoveToPosition", lResult, p_rlErrorCode);
					lResult = MMC_FAILED;
				}
				lResult = MMC_FAILED;
				LogError("VCS_HaltVelocityMovement", lResult, p_rlErrorCode);
			}
		}

		if(VCS_MoveToPosition(p_DeviceHandle, p_usNodeId, 0, 1, 1, &p_rlErrorCode) == 0)
		{
			LogError("VCS_MoveToPosition", lResult, p_rlErrorCode);
			lResult = MMC_FAILED;
			ROS_INFO("Can't Home!!!ERROR");
		}
		ROS_INFO("HOMING: Please Wait!");
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		ROS_INFO("Overshoot Count: %d total of %d frames",myCounter,totalFrameNumber);
		totalFrameNumber=0;
		myCounter=0;
		//ros::shutdown();
		return;
		

	}

	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	//double passTime_s = std::chrono::duration_cast<std::chrono::seconds>(end - begin).count();
	passTime_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - beginSelf).count();
	time_ms += passTime_ms;
	if(passTime_ms > 41){
		//cout << "Time overshoot.";
		myCounter ++;
	}
	//cout<<passTime_ms<<endl;
	
	time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end-beginLoop).count();

	if (!sumofsines && !GainMode && OpenLoop && !ClosedLoop){

		auto seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();

		

		srand(static_cast<unsigned int>(seed));
		float random = std::rand();
		float noise = (low + (static_cast<double>((random)) / (RAND_MAX / (high-low))))*2 + 0.5;

		targetpositionFloat =0.0;
		for(int i=0;i<13;i++){
			targetpositionFloat +=  ((A_pos)/(2*M_PI*freqs[i]))*sin(2.0*M_PI*freqs[i]*time_ms/1000+velPH[i] +(M_PI/2.0));
		}
		targetpositionFloat += A_pos*(noise*2);
		ROS_INFO(" noise is %f  seed is %li position is %f \n", noise, seed, targetpositionFloat);
		
	}

    else if (sumofsines && !GainMode && !OpenLoop && !ClosedLoop){

        targetpositionFloat =0.0;
		for(int i=0;i<13;i++){
			targetpositionFloat +=  (A_pos/(2*M_PI*freqs[i]))*sin(2.0*M_PI*freqs[i]*time_ms/1000+velPH[i] +(M_PI/2.0));
            
        }
		
		// if(msg->finish_c=="start"){
        //       video_path_name = "sos";
        //       video_path_name.pop_back();
        //       myfile.open ("/home/neurolab/Desktop/Experiment_Materials/"+video_path_name + ".csv");     
        //       }
      
      

        //     myfile << std::to_string(targetpositionFloat) +"\n";

        //     if(msg->finish_c=="end"){ 
        //     myfile.close(); 
       
		// }
	}
	

	else if (!sumofsines && !GainMode && !OpenLoop && !ClosedLoop)
	{ 
		 // targetpositionFloat = (A_pos/(2*M_PI*frequency))*sin(2.0*M_PI*frequency*time_ms/1000); 

        //*******ADD NOİSE ********
		// auto seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();

		// srand(static_cast<unsigned int>(seed));
		// float random = std::rand();
		// float noise = (low + (static_cast<double>((random)) / (RAND_MAX / (high-low))))*2 + 0.5;
		// targetpositionFloat += A_pos*(noise/5);
        // *************************

		// From Interface
		targetpositionFloat = A_pos*positions[csvIndex]*fish_direction/29.0; // From CSV
	}
	//if(//TODO){ targetposition = A_pos*std::atof(msg->target_pos.data); }

	if(GainMode && !OpenLoop && !sumofsines && !ClosedLoop){
		  pos_x = -reafferentGain*msg->target_pos_x/29.55;
		if(pos_x>=limit){
			pos_x=limit;

		}else if(pos_x<=-limit){
			pos_x=-limit;
		}
		targetpositionFloat =0.0;
		targetpositionFloat = (A_pos*pos_x);
		//for(int i=0;i<13;i++){
		//	targetpositionFloat +=  (A_pos/(2*M_PI*freqs[i]))*sin(2.0*M_PI*freqs[i]*time_ms/1000+velPH[i] +(M_PI/2.0));
            
        // }
		targetpositionFloat = (A_pos/(2*M_PI*frequency))*sin(2.0*M_PI*frequency*time_ms/1000) + targetpositionFloat;
	}


	if(GainMode && OpenLoop && !sumofsines && !ClosedLoop)
	{
		  pos_x = -reafferentGain*msg->target_pos_x/29.55;
		if(pos_x>=limit){
			pos_x=limit;

		}else if(pos_x<=-limit){
			pos_x=-limit;
		}
        targetpositionFloat = (A_pos*pos_x);
		auto seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();

		srand(static_cast<unsigned int>(seed));
		float random = std::rand();
		float noise = (low + (static_cast<double>((random)) / (RAND_MAX / (high-low))))*2 + 0.5;

		
		for(int i=0;i<13;i++){
			targetpositionFloat +=  ((A_pos)/(2*M_PI*freqs[i]))*sin(2.0*M_PI*freqs[i]*time_ms/1000+velPH[i] +(M_PI/2.0));
		}

		
		targetpositionFloat += A_pos*(noise*2);
	}

	if(ClosedLoop ){
		  pos_x = -reafferentGain*msg->target_pos_x/29.55;
		ROS_INFO("2222 %f", pos_x);
		if(pos_x>=limit){
			pos_x=limit;

		}else if(pos_x<=-limit){
			pos_x=-limit;
		}
		// targetpositionFloat = 20000;
		ROS_INFO("1111111111");
		// targetpositionFloat = (A_pos*pos_x);
		// for(int i=0;i<13;i++){
		// 	targetpositionFloat +=  (A_pos/(2*M_PI*freqs[i]))*sin(2.0*M_PI*freqs[i]*time_ms/1000+velPH[i] +(M_PI/2.0));
            
        // }
		// // targetpositionFloat = (A_pos/(2*M_PI*frequency))*sin(2.0*M_PI*frequency*time_ms/1000) + targetpositionFloat;
	}
	// targetpositionFloat = 0;

	// std::cout << pos_x<< std::endl;
	// ROS_INFO("asdasd %d", GainMode);
	// ROS_INFO("asdasd %d", OpenLoop);
	// ROS_INFO("asdasd %d", ClosedLoop);
	ROS_INFO("1111");
	ROS_INFO("2222 %f", targetpositionFloat);
	
	beginSelf = std::chrono::steady_clock::now();

	targetposition = (long)targetpositionFloat;
	if(VCS_MoveToPosition(p_DeviceHandle, p_usNodeId, targetposition, 1, 1, &p_rlErrorCode) == 0)
	{
		LogError("VCS_MoveToPosition", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
		
	}
	csvIndex++;
	usleep(1);

}
void setReafferentGain (const rectrial::pub_data::ConstPtr& msg)
{
	std::string my_msg = msg->data_e.c_str();
	reafferentGain = stod(my_msg);
	OpenLoop = false;
	GainMode = true;
	sumofsines = false;
	cout<< "Using Gain of " <<reafferentGain;

}
void motorSetFreq(const std_msgs::String::ConstPtr& msg)
{
  std::string my_msg = msg->data.c_str();
  if(msg->data == "sum"){
	  OpenLoop = false;
	  sumofsines=true;
	  GainMode = false;
	  cout<<"Using sum of sines signal.";
  }
  else if(my_msg.substr(0,4)=="gain"){
	std::string my_msg = msg->data.c_str();
	int pos = my_msg.find(":");
    string sub = my_msg.substr(pos + 1);
	reafferentGain = stod(sub);
	OpenLoop = false;
	GainMode = true;
	sumofsines = false;
	cout<< "Using Gain of " <<reafferentGain;

  }

   else if(my_msg.substr(3,4)=="gain"){
	std::string my_msg = msg->data.c_str();
	int pos = my_msg.find(":");
    string sub = my_msg.substr(pos + 1);
	reafferentGain = stod(sub);
	OpenLoop = true;
	GainMode = true;
	sumofsines = false;
	cout<< "Using Gain of " <<reafferentGain;

  }


  else if(msg->data == "openloop"){
    OpenLoop = true;
    sumofsines = false;
	GainMode = false;
	cout<< " Using the open loop. ";

  }
  else if(msg->data == "closedloop"){
	ClosedLoop = true;
    OpenLoop = false;
    sumofsines = false;
	GainMode = false;
	cout<< " Using the closed loop. ";

  }
  else{
	  sumofsines=false;
	  GainMode =false;
	  frequency= stod(msg->data);
	  cout<<"Using frequency of "<<frequency;
  }
  cout<<endl;
}
void setState(const std_msgs::String::ConstPtr& msg){
	if(msg->data =="shutdown"){
		ROS_INFO("Closing Motor Node!");
		ros::shutdown();
	}
}
int main(int argc, char** argv)
{
	int lResult = MMC_FAILED;
	unsigned int ulErrorCode = 0;

	ros::init(argc, argv, "maxon");
	ros::NodeHandle nh;
	
	SetDefaultParameters();

	PrintSettings();
		
	if((lResult = OpenDevice(&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("OpenDevice", lResult, ulErrorCode);
		return lResult;
	}
	cout<<"Device is opened."<<endl;
	if((lResult = PrepareDemo(&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("PrepareDemo", lResult, ulErrorCode);
		return lResult;
	}
	cout<<"Motor is prepared."<<endl;
	ros::Subscriber motorFreqSub = nh.subscribe("set_motor_freq", 100, motorSetFreq);
	//ros::Subscriber FilteredDataSub = nh.subscribe("filtered_data", 100, setReafferentGain);
	sub = nh.subscribe("filtered_data", 100, &imageCallback);
	ros::Subscriber stateSub = nh.subscribe("set_state", 100, setState);



    //ros::Subscriber coordinatesSub = nh.subscribe("Coordinates", 100, &coordinatesCallback);

	// if((lResult = Demo(&ulErrorCode))!=MMC_SUCCESS)
	// {
	// 	LogError("Demo", lResult, ulErrorCode);
	// 	return lResult;
	// }
	
	if(initializeMotor(g_pKeyHandle, g_usNodeId, ulErrorCode) != MMC_SUCCESS)
	{
		LogError("MotorInitialization", lResult, ulErrorCode);
		return lResult;
	}
	cout<<"Motor is initialized"<<endl;
	ros::spin();
	cout<<"Overshoot Count: "<<myCounter<<" total of "<<totalFrameNumber<<" frames"<<endl;
	

	if((lResult = CloseDevice(&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("CloseDevice", lResult, ulErrorCode);
		return lResult;
	}
		
		
	

	return lResult;
}