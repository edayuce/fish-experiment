#include "epos_lib.h"

using namespace std;

HANDLE g_deviceHandle = 0;
unsigned short g_usNodeId = 1;
string g_deviceName;
string g_protocolStackName;
string g_interfaceName;
string g_portName;
int g_baudrate = 0;

EAppMode g_eAppMode = AM_DEMO;
ControllerMode g_controllerMode = UNSET;

unsigned int g_errorCode = 0;


const string g_programName = "HelloEposCmd";



int lResult = MMC_SUCCESS;


bool printInfo = true;

void LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode)
{
	cerr << g_programName << ": " << functionName << " failed (result=" << p_lResult << ", errorCode=0x" << std::hex << p_ulErrorCode << ")"<< endl;
}

void LogInfo(string message)
{
    if(printInfo){
	    cout << message << endl;
    }
}

bool InitializeController(string deviceName, string protocolStackName, string interfaceName, string portName, int baudrate){

    g_usNodeId = 1;
    g_deviceName = deviceName; 
    g_protocolStackName = protocolStackName; 
    g_interfaceName = interfaceName; 
    g_portName = portName; 
    g_baudrate = baudrate; 

    return true;

}

bool InitializeEPOS4(string interface = "USB")
{
    if(interface == "USB"){
        return InitializeController("EPOS4", "MAXON SERIAL V2", "USB", "USB0",1000000);

    }else if(interface == "RS232"){
        // RS232 (NOT TESTED)
        return InitializeController("EPOS4", "MAXON_RS232", "RS232", "COM1",115200);

    }else{
        LogInfo("Wrong interface name!");
        return false;
    }
}

bool InitializeEPOS2(string interface = "USB")
{
    if(interface == "USB"){
        return InitializeController("EPOS2", "MAXON SERIAL V2", "USB", "USB0",1000000);

    }else if(interface == "RS232"){
        // RS232 (NOT TESTED)
        return InitializeController("EPOS2", "MAXON_RS232", "RS232", "COM1",115200);

    }else{
        LogInfo("Wrong interface name!");
        return false;
    }

}


bool OpenDevice()
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

	g_deviceHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName,&g_errorCode);

	if(g_deviceHandle!=0 && g_errorCode == 0)
	{
		unsigned int lBaudrate = 0;
		unsigned int lTimeout = 0;

		if(VCS_GetProtocolStackSettings(g_deviceHandle, &lBaudrate, &lTimeout, &g_errorCode)!=0)
		{
			if(VCS_SetProtocolStackSettings(g_deviceHandle, g_baudrate, lTimeout, &g_errorCode)!=0)
			{
				if(VCS_GetProtocolStackSettings(g_deviceHandle, &lBaudrate, &lTimeout, &g_errorCode)!=0)
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
		g_deviceHandle = 0;
	}

	delete []pDeviceName;
	delete []pProtocolStackName;
	delete []pInterfaceName;
	delete []pPortName;

    if((lResult !=MMC_SUCCESS))
    {
        LogError("OpenDevice", lResult, g_errorCode);
    }else{
        LogInfo("Device is opened.");
    }

	return lResult == MMC_SUCCESS;
}





bool InitializePositionMode(){
    if(VCS_ActivateProfilePositionMode(g_deviceHandle, g_usNodeId, &g_errorCode) == 0)
	{
		LogError("VCS_ActivateProfilePositionMode", lResult, g_errorCode);
		lResult = MMC_FAILED;
	}else{
        LogInfo("Position mode is ready.");
        g_controllerMode = POSITION;
    }

    return lResult == MMC_SUCCESS;
}

bool InitializeVelocityMode(){
    if(VCS_ActivateProfileVelocityMode(g_deviceHandle, g_usNodeId, &g_errorCode) == 0)
	{
		LogError("VCS_ActivateProfileVelocityMode", lResult, g_errorCode);
		lResult = MMC_FAILED;
	}else{
        LogInfo("Velocity mode is ready.");
        g_controllerMode = VELOCITY;
    }

    return lResult == MMC_SUCCESS;
}


bool goToPosition(long targetPosition){
	stringstream msg;

    int lResult = MMC_SUCCESS;

    if(g_controllerMode != POSITION){
        LogInfo("Go to position is supported in POSITION Mode only! Please set POSITION Mode first!");
    }

    BOOL moveImmediate = 1; /* MOVE IMMEDIATE (1) OR WAIT PREVIOUS MOVEMENT TO FINISH(0) */
    BOOL relativeMovement = 0; /* MOVE TO AN ABSOLUTE POS = 0 , MOVE TO A RELATIVE POS = 1*/


    msg << "move to position = " << targetPosition << ", node = " << g_usNodeId;
			LogInfo(msg.str());

    if(VCS_MoveToPosition(g_deviceHandle, g_usNodeId, targetPosition, relativeMovement, moveImmediate, &g_errorCode) == 0)
    {
        LogError("VCS_MoveToPosition", lResult, g_errorCode);
        lResult = MMC_FAILED;
    }else{
        LogInfo("Position is executed.");
    }

    return lResult == MMC_SUCCESS;

}


bool CloseDevice(unsigned int* p_pErrorCode)
{
	int lResult = MMC_FAILED;

	*p_pErrorCode = 0;

	LogInfo("Close device");

	if(VCS_CloseDevice(g_deviceHandle, &g_errorCode)!=0 && g_errorCode == 0)
	{
		lResult = MMC_SUCCESS;
        cout << "Device is successfully closed." << endl;
	}else{
		LogError("CloseDevice", lResult, g_errorCode);
    }

	return lResult == MMC_SUCCESS;
}


