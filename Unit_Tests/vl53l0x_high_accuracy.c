//High Accuracy Test vl53l0x
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include <malloc.h>
#include "I2C.h"
#include "delay.h"
#include "UART.h"
#include "clock.h"

#define VERSION_REQUIRED_MAJOR 1
#define VERSION_REQUIRED_MINOR 0
#define VERSION_REQUIRED_BUILD 4

void print_pal_error(VL53L0X_Error Status){
    char buf[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetPalErrorString(Status, buf);
    char msg[100];
    snprintf(msg, 100, "API Status: %i : %s\n", Status, buf);
    USART3_SendString(msg);
}

void print_range_status(VL53L0X_RangingMeasurementData_t* pRangingMeasurementData){
    char buf[VL53L0X_MAX_STRING_LENGTH];
    uint8_t RangeStatus;

    /*
     * New Range Status: data is valid when pRangingMeasurementData->RangeStatus = 0
     */

    RangeStatus = pRangingMeasurementData->RangeStatus;

    VL53L0X_GetRangeStatusString(RangeStatus, buf);
    char msg[100];
    snprintf(msg, 100, "Range Status: %i : %s\n", RangeStatus, buf);
    USART3_SendString(msg);
}


VL53L0X_Error rangingTest(VL53L0X_Dev_t *pMyDevice)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_RangingMeasurementData_t    RangingMeasurementData;
    int i;
    FixPoint1616_t LimitCheckCurrent;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    if(Status == VL53L0X_ERROR_NONE)
    {
        USART3_SendString ("Call of VL53L0X_StaticInit\n");
        Status = VL53L0X_StaticInit(pMyDevice); // Device Initialization
        print_pal_error(Status);
    }
    
    if(Status == VL53L0X_ERROR_NONE)
    {
        USART3_SendString ("Call of VL53L0X_PerformRefCalibration\n");
        Status = VL53L0X_PerformRefCalibration(pMyDevice,
        		&VhvSettings, &PhaseCal); // Device Initialization
        print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE) // needed if a coverglass is used and no calibration has been performed
    {
        char msg[100];
        USART3_SendString ("Call of VL53L0X_PerformRefSpadManagement\n");
        Status = VL53L0X_PerformRefSpadManagement(pMyDevice,
        		&refSpadCount, &isApertureSpads); // Device Initialization
        snprintf (msg, 100, "refSpadCount = %ld, isApertureSpads = %d\n", refSpadCount, isApertureSpads);
        USART3_SendString(msg);
        print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {

        // no need to do this when we use VL53L0X_PerformSingleRangingMeasurement
        USART3_SendString ("Call of VL53L0X_SetDeviceMode\n");
        Status = VL53L0X_SetDeviceMode(pMyDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
        print_pal_error(Status);
    }

    // Enable/Disable Sigma and Signal check
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
        		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    }
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    }
				
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(pMyDevice,
        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
        		(FixPoint1616_t)(0.25*65536));
	}			
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(pMyDevice,
        		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
        		(FixPoint1616_t)(18*65536));			
    }
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pMyDevice,
        		200000);
    }
    /*
     *  Step  4 : Test ranging mode
     */

    if(Status == VL53L0X_ERROR_NONE)
    {
        for(i=0;i<10;i++){
            USART3_SendString ("Call of VL53L0X_PerformSingleRangingMeasurement\n");
            Status = VL53L0X_PerformSingleRangingMeasurement(pMyDevice,
            		&RangingMeasurementData);

            print_pal_error(Status);
            print_range_status(&RangingMeasurementData);

            VL53L0X_GetLimitCheckCurrent(pMyDevice,
            		VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, &LimitCheckCurrent);
            char msg[100];
            snprintf(msg, 100, "RANGE IGNORE THRESHOLD: %f\n\n", (float)LimitCheckCurrent/65536.0);
            USART3_SendString(msg);

            if (Status != VL53L0X_ERROR_NONE) break;

            snprintf(msg, 100, "Measured distance: %i\n\n", RangingMeasurementData.RangeMilliMeter);
            USART3_SendString(msg);

        }
    }
    return Status;
}

int main(int argc, char **argv)
{
    Clock_Config();
    Timer_Config();
    UART3_Config();
    I2C_Config();

    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_Dev_t MyDevice;
    VL53L0X_Dev_t *pMyDevice = &MyDevice;
    VL53L0X_Version_t                   Version;
    VL53L0X_Version_t                  *pVersion   = &Version;
    VL53L0X_DeviceInfo_t                DeviceInfo;

    int32_t status_int;

    USART3_SendString ("VL53L0X API Simple Ranging example\n\n");

    // Initialize Comms
    pMyDevice->I2cDevAddr      = 0x29;
    pMyDevice->comms_type      =  1;
    pMyDevice->comms_speed_khz =  400;

    /*
     *  Get the version of the VL53L0X API running in the firmware
     */

    if(Status == VL53L0X_ERROR_NONE)
    {
        status_int = VL53L0X_GetVersion(pVersion);
        if (status_int != 0)
            Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    /*
     *  Verify the version of the VL53L0X API running in the firmware
     */

    if(Status == VL53L0X_ERROR_NONE)
    {
        if( pVersion->major != VERSION_REQUIRED_MAJOR ||
            pVersion->minor != VERSION_REQUIRED_MINOR ||
            pVersion->build != VERSION_REQUIRED_BUILD )
        {
            char msg[250];
            snprintf(msg, 250, "VL53L0X API Version Error: Your firmware has %d.%d.%d (revision %ld). This example requires %d.%d.%d.\n",
                pVersion->major, pVersion->minor, pVersion->build, pVersion->revision,
                VERSION_REQUIRED_MAJOR, VERSION_REQUIRED_MINOR, VERSION_REQUIRED_BUILD);
            USART3_SendString(msg);
        }
    }


    if(Status == VL53L0X_ERROR_NONE)
    {
        USART3_SendString ("Call of VL53L0X_DataInit\n");
        Status = VL53L0X_DataInit(&MyDevice); // Data initialization
        print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_GetDeviceInfo(&MyDevice, &DeviceInfo);
        if(Status == VL53L0X_ERROR_NONE)
        {
            char msg[150];
            USART3_SendString("VL53L0X_GetDeviceInfo:\n");
            snprintf(msg, 150, "Device Name : %s\n", DeviceInfo.Name);
            USART3_SendString(msg);
            snprintf(msg, 150, "Device Type : %s\n", DeviceInfo.Type);
            USART3_SendString(msg);
            snprintf(msg, 150, "Device ID : %s\n", DeviceInfo.ProductId);
            USART3_SendString(msg);
            snprintf(msg, 150, "ProductRevisionMajor : %d\n", DeviceInfo.ProductRevisionMajor);
            USART3_SendString(msg);
            snprintf(msg, 150, "ProductRevisionMinor : %d\n", DeviceInfo.ProductRevisionMinor);
            USART3_SendString(msg);

        if ((DeviceInfo.ProductRevisionMinor != 1) && (DeviceInfo.ProductRevisionMinor != 1)) {
        	snprintf(msg, 150, "Error expected cut 1.1 but found cut %d.%d\n",
                       DeviceInfo.ProductRevisionMajor, DeviceInfo.ProductRevisionMinor);
                Status = VL53L0X_ERROR_NOT_SUPPORTED;
            USART3_SendString(msg);
            }
            
        }
        print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
        Status = rangingTest(pMyDevice);
    }

    print_pal_error(Status);
    return (0);
}

