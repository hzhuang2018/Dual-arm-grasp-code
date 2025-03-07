  /*
  ******************************************************************************
  * @file     : CANTest.cpp
  * @Copyright: ViewTool 
  * @Revision : ver 1.0.1
  * @Date     : 2020/11/21 9:27
  * @brief    : CANTest demo
  ******************************************************************************
  * @attention
  *
  * Copyright 2009-2020, ViewTool
  * http://www.viewtool.com/
  * All Rights Reserved
  * 
  ******************************************************************************
  */

#ifndef OS_UNIX
#include <Windows.h>
#else
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#endif
#include "..\include\touch_sensor\ControlCAN.h"
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <process.h>

#define VERSION    				"1.0.1"
#define VT_LOG    				1

#define	CAN_MODE_LOOP_BACK		0
#define	CAN_SEND_DATA			0
#define CAN_GET_BOARD_INFO		1
#define CAN_READ_DATA			1
#define CAN_CALLBACK_READ_DATA	0
#define	CAN_INIT_EX				1
#define CAN_GET_STATUS			1

#define CAN_SEND_FRAME_COUNT 	2 // 10
#define CAN_DATA_SEND_FRAME_COUNT 1
// CAN_CHANNEL_START 0    CAN_CHANNEL_END 0
#define CAN_CHANNEL_START 		0  	// mini : MUST BE   1 
									// Ginkgo3: MUST BE 0

#define CAN_CHANNEL_END 		0  	// mini : MUST BE   1 
									// Ginkgo3: MUST BE 0
									// Ginkgo2: [0, 1] 
				
int devType2=VCI_USBCAN2; 	
int devIndex2=1; 
int canIndex2=CAN_CHANNEL_START;

/*
Used for Hardware (CLK=36MHz):
1. ViewTool Ginkgo USB-CAN Bus Adapter: VTG202A
2. ViewTool Ginkgo USB-CAN Interface: VTG203B
*/


int read_data(int localdevindex);
/* BAUD_RATE SJW BS1 BS2 PreScale; */
s_CAN_BaudRate  Ginkgo2CAN_BaudRateInitTab[]= {
	{1000,1,2,1,9},     // 1M (1000K)
	{900,1,5,2,5},      // 900K
	{800,1,10,4,3},     // 800K
	{666,1,7,1,6},      // 666K
	{600,1,3,1,12},     // 600K
	{500,1,4,1,12},     // 500K
	{400,1,7,1,10},     // 400K
	{300,1,6,1,15},     // 300K
	{250,1,6,1,18},     // 250K
	{225,1,6,1,20},     // 225K
	{200,1,15,2,10},    // 200K
	{160,1,12,2,15},    // 160K
	{150,1,6,1,30},     // 150K
	{144,1,3,1,50},     // 144K
	{125,1,6,1,36},     // 125K
	{120,1,12,2,20},    // 120K
	{100,1,6,1,45},     // 100K
	{90,1,6,1,50},      // 90K
	{80,1,4,1,75},      // 80K
	{75,1,6,1,60},      // 75K
	{60,1,6,1,75},      // 60K
	{50,1,6,1,90},      // 50K
	{40,1,7,1,100},     // 40K
	{30,1,6,1,150},     // 30K
	{20,1,6,1,225},     // 20K
};
	
s_CAN_BaudRate  Ginkgo3CAN_BaudRateInitTab[]= {     
	{1000,1,2,4,6},       // 1M (1000K)
	{900,1,16,6,2},       // 900K
	{800,1,5,7,4},       // 800K
	{666,1,12,8,3},       // 666K
	{600,1,5,4,7},       // 600K
	{500,1,7,6,6},       // 500K
	{400,1,7,7,7},       // 400K
	{300,1,8,5,10},      // 300K
	{250,1,8,5,12},      // 250K
	{225,1,13,3,11},      // 225K
	{200,1,5,8,15},      // 200K
	{160,1,9,1,24},      // 160K
	{150,1,6,3,28},      // 150K
	{144,1,7,2,29},      // 144K
	{125,1,4,3,42},     // 125K
	{120,1,3,3,50},     // 120K
	{100,1,6,5,35},      // 100K
	{90,1,4,7,39},     // 90K
	{80,1,16,4,25},     // 80K
	{75,1,8,1,56},      // 75K
	{60,1,5,1,100},      // 60K
	{50,1,6,1,105},      // 50K
	{40,1,7,2,105},      // 40K
	{30,1,11,2,100},     // 30K
	{20,1,9,2,175},     // 20K
};

eGinkgoCanDev vt_usb_can_check_dev2(PVCI_BOARD_INFO_EX pInfo){
	eGinkgoCanDev ret;
    char str_product_name[32];

    // memcpy(str_product_name, (u8 *)dev->adapter->BoardInfo->ProductName, 32);
	strncpy_s(str_product_name, (const char*)pInfo->ProductName + 0, sizeof(STR_HEADER_GINKG3)-1);

    // cmp = strstr(str_product_name, STR_HEADER_GINKG3);
	if(strstr(str_product_name, STR_HEADER_GINKG3) != NULL){
	    	ret = DEV_GINKGO3;	
	}	
 	else{
		ret = DEV_GINKGO2;
	}    

    return ret;
}

/*
SetBaudRate:  Set CAN protocol communication baudrate
Parameter:  
  baudrate: unit: Khz, range: 20-1000 -> 20Khz-1000Khz
Return: 
  1: success
  0: failed (no matched)
*/

int SetBaudRate2(int baudrate, VCI_INIT_CONFIG_EX *config_ex, PVCI_BOARD_INFO_EX pInfo)
{
	int i, ret=0; 

	int tbl_size = 0;
	eGinkgoCanDev ginkgoDev;
	s_CAN_BaudRate br_tbl[30]={0};
	
	ginkgoDev = vt_usb_can_check_dev2(pInfo);
	SPRINTF(("ginkgoDev: %d\n", ginkgoDev));
    if(ginkgoDev == DEV_GINKGO3){
		memcpy((void *)br_tbl, Ginkgo3CAN_BaudRateInitTab, sizeof(Ginkgo3CAN_BaudRateInitTab));
    }
	else // if(ginkgoDev == DEV_GINKGO2)
    {
        memcpy((void *)br_tbl, Ginkgo2CAN_BaudRateInitTab, sizeof(Ginkgo2CAN_BaudRateInitTab));        
    }
	
	tbl_size = sizeof(Ginkgo2CAN_BaudRateInitTab);	
		
	for(i = 0; i < tbl_size; i++)
	{
		if(br_tbl[i].BAUD_RATE == baudrate)
		{
			config_ex->CAN_SJW = br_tbl[i].SJW;
			config_ex->CAN_BS1 = br_tbl[i].BS1;
			config_ex->CAN_BS2 = br_tbl[i].BS2;
			config_ex->CAN_BRP = br_tbl[i].PreScale;
			
	        SPRINTF(("tbl i : %d %d %d %d %d \n", i, config_ex->CAN_SJW, config_ex->CAN_BS1, config_ex->CAN_BS2, config_ex->CAN_BRP));
		    SPRINTF(("Baud Rate : %d Khz\n", baudrate));

			ret = true;
			break;
		}
	}   
	
	if(!ret){ // default 1000K.		
		config_ex->CAN_SJW = br_tbl[0].SJW;
		config_ex->CAN_BS1 = br_tbl[0].BS1;
		config_ex->CAN_BS2 = br_tbl[0].BS2;
		config_ex->CAN_BRP = br_tbl[0].PreScale;			

		SPRINTF(("Set default Baud Rate = 1000 Khz\n"));
	}
		
	return ret; 
}

#if CAN_CALLBACK_READ_DATA
void WINAPI GetDataCallback2(uint32_t DevIndex,uint32_t CANIndex,uint32_t Len)
{
	int		x_force_raw_data[24];//原始数据
	float	x_force_data[24];//实际表示的力
	int		y_force_raw_data[24];//原始数据
	float	y_force_data[24];//实际表示的力
	int		z_force_raw_data[24];//原始数据
	float	z_force_data[24];//实际表示的力
	std::ofstream x_record_pos1("x_record_pos1.txt", std::ios::app);//trunc
	std::ofstream y_record_pos1("y_record_pos1.txt", std::ios::app);//trunc
	std::ofstream z_record_pos1("z_record_pos1.txt", std::ios::app);//trunc
	std::ofstream x_record_pos7("x_record_pos7.txt", std::ios::app);//trunc
	std::ofstream y_record_pos7("y_record_pos7.txt", std::ios::app);//trunc
	std::ofstream z_record_pos7("z_record_pos7.txt", std::ios::app);//trunc
	int col, row, sensorID;

	canIndex2 =1;
	int ReadDataNum;
	int DataNum = VCI_GetReceiveNum(devType2, devIndex2, canIndex2);
	VCI_CAN_OBJ* pCAN_ReceiveData = (VCI_CAN_OBJ*)malloc(DataNum * sizeof(VCI_CAN_OBJ));
	if ((DataNum > 0) && (pCAN_ReceiveData != NULL)) {
		ReadDataNum = VCI_Receive(devType2, devIndex2, canIndex2, pCAN_ReceiveData, DataNum);
		for (int i = 0; i < ReadDataNum; i++)
		{
			SPRINTF(("\n"));
			SPRINTF(("CAN_CALLBACK_READ_DATA:\n"));
			SPRINTF(("--CAN_ReceiveData.RemoteFlag = %d\n", pCAN_ReceiveData[i].RemoteFlag));
			SPRINTF(("--CAN_ReceiveData.ExternFlag = %d\n", pCAN_ReceiveData[i].ExternFlag));
			SPRINTF(("--CAN_ReceiveData.ID = 0x%X\n", pCAN_ReceiveData[i].ID));
			row = pCAN_ReceiveData[i].ID / 256;
			col = pCAN_ReceiveData[i].ID % 256 / 16;
			sensorID = pCAN_ReceiveData[i].ID % 256 % 16;

			SPRINTF(("--CAN_ReceiveData.DataLen = %d\n", pCAN_ReceiveData[i].DataLen));
			SPRINTF(("--CAN_ReceiveData.Data:"));
			for (int j = 0; j < pCAN_ReceiveData[i].DataLen; j++) {
				SPRINTF(("%02X ", pCAN_ReceiveData[i].Data[j]));
			}
			//将收到的数据调整到测量范围内，并存储数据
			x_force_raw_data[col * 4 + row] = pCAN_ReceiveData[i].Data[1] * 256 + pCAN_ReceiveData[i].Data[2];
			y_force_raw_data[col * 4 + row] = pCAN_ReceiveData[i].Data[3] * 256 + pCAN_ReceiveData[i].Data[4];
			z_force_raw_data[col * 4 + row] = pCAN_ReceiveData[i].Data[5] * 256 + pCAN_ReceiveData[i].Data[6];
			if (1 == sensorID)
			{
				x_record_pos1 << (float)900.0 * (x_force_raw_data[col * 4 + row] - 32768) / 32768 << " ";
				y_record_pos1 << (float)900.0 * (y_force_raw_data[col * 4 + row] - 32768) / 32768 << " ";
				z_record_pos1 << (float)5000.0 * (z_force_raw_data[col * 4 + row] - 35000) / 32768 << " ";
			}
			else
			{
				x_record_pos7 << (float)900.0 * (x_force_raw_data[col * 4 + row] - 32768) / 32768 << " ";
				y_record_pos7 << (float)900.0 * (y_force_raw_data[col * 4 + row] - 32768) / 32768 << " ";
				z_record_pos7 << (float)5000.0 * (z_force_raw_data[col * 4 + row] - 35000) / 32768 << " ";
			}

			SPRINTF(("\n"));
			SPRINTF(("--CAN_ReceiveData.TimeStamp = %d\n\n", pCAN_ReceiveData[i].TimeStamp));
			if ((row == 3) && (col == 5) && (1 == sensorID))
			{
				x_record_pos1 << std::endl;
				y_record_pos1 << std::endl;
				z_record_pos1 << std::endl;
			}
			if ((row == 3) && (col == 5) && (7 == sensorID))
			{
				x_record_pos7 << std::endl;
				y_record_pos7 << std::endl;
				z_record_pos7 << std::endl;
			}
		}
	}
	x_record_pos1.close();
	y_record_pos1.close();
	z_record_pos1.close();
	x_record_pos7.close();
	y_record_pos7.close();
	z_record_pos7.close();
	free(pCAN_ReceiveData);
	//canIndex = (canIndex == 0) ? 1 : 0;
}
#endif

//int main(void)
//{
//
//	read_data(0);
//	//read_data(1);
//}

int touch_sensor_read_data2()
{
	int		x_force_raw_data[24];//原始数据
	float	x_force_data[24];//实际表示的力
	int		y_force_raw_data[24];//原始数据
	float	y_force_data[24];//实际表示的力
	int		z_force_raw_data[24];//原始数据
	float	z_force_data[24];//实际表示的力
	//std::ofstream x_record_pos1("x_record_pos1.txt", std::ios::trunc);//trunc
	//std::ofstream y_record_pos1("y_record_pos1.txt", std::ios::trunc);//trunc
	//std::ofstream z_record_pos1("z_record_pos1.txt", std::ios::trunc);//trunc
	std::ofstream x_record_pos7("x_record_pos7.txt", std::ios::trunc);//trunc
	std::ofstream y_record_pos7("y_record_pos7.txt", std::ios::trunc);//trunc
	std::ofstream z_record_pos7("z_record_pos7.txt", std::ios::trunc);//trunc
	int col, row, sensorID;




	//devIndex = localdevindex;
    int DevNum,Status;
	int i;  

    SPRINTF(("CAN Test version : %s\n", VERSION));

	//Scan device
    DevNum = VCI_ScanDevice(1);
    if(DevNum > 0){
        SPRINTF(("Have %d device connected!\n",DevNum));
    }else{
        SPRINTF(("No device connected!\n"));
        return 0;
    }
	
	//Open device
    Status = VCI_OpenDevice(devType2, devIndex2, 0);
    if(Status==STATUS_ERR){
        SPRINTF(("Open device failed!\n"));
        return 0;
    }else{
        SPRINTF(("Open device success!\n"));
    }
	
    //Get board info
#if CAN_GET_BOARD_INFO
	VCI_BOARD_INFO_EX CAN_BoardInfo;
    Status = VCI_ReadBoardInfoEx(devIndex2, &CAN_BoardInfo);//It will open device
    if(Status==STATUS_ERR){
        SPRINTF(("Get board info failed!\n"));
        return 0;
    }else{
        SPRINTF(("--CAN_BoardInfo.ProductName = %s\n",CAN_BoardInfo.ProductName));
        SPRINTF(("--CAN_BoardInfo.FirmwareVersion = V%d.%d.%d\n",CAN_BoardInfo.FirmwareVersion[1],CAN_BoardInfo.FirmwareVersion[2],CAN_BoardInfo.FirmwareVersion[3]));
        SPRINTF(("--CAN_BoardInfo.HardwareVersion = V%d.%d.%d\n",CAN_BoardInfo.HardwareVersion[1],CAN_BoardInfo.HardwareVersion[2],CAN_BoardInfo.HardwareVersion[3]));
        SPRINTF(("--CAN_BoardInfo.SerialNumber = %08X%08X%08X\n",(uint32_t)(*(uint32_t*)(&CAN_BoardInfo.SerialNumber[0])),(uint32_t)(*(uint32_t*)(&CAN_BoardInfo.SerialNumber[4])),(uint32_t)(*(uint32_t*)(&CAN_BoardInfo.SerialNumber[8]))));
    }
#endif
#if CAN_INIT_EX
	VCI_INIT_CONFIG_EX	CAN_InitEx;
    //Config device
    CAN_InitEx.CAN_ABOM = 0;
#if CAN_MODE_LOOP_BACK
    CAN_InitEx.CAN_Mode = 1;
#else
    CAN_InitEx.CAN_Mode = 0;
#endif

    CAN_InitEx.CAN_NART = 0;
    CAN_InitEx.CAN_RFLM = 0;
    CAN_InitEx.CAN_TXFP = 1;
	CAN_InitEx.CAN_RELAY = 0;
	
    SetBaudRate2(1000, &CAN_InitEx, &CAN_BoardInfo); 

	for(int canCh=CAN_CHANNEL_START; canCh<=CAN_CHANNEL_END; canCh++)
	{
		canIndex2=canCh;
		Status = VCI_InitCANEx(devType2, devIndex2, canIndex2, &CAN_InitEx);
#if VT_LOG
        SPRINTF(("\n****************************\n"));
        SPRINTF(("VCI_InitCANEx() status (1:ok): %d\n",Status));
        SPRINTF(("VCI_USBCAN2 = %d:\n", VCI_USBCAN2));
        SPRINTF(("devIndex = %d: \n",devIndex2));
        SPRINTF(("canIndex = %d: \n",canIndex2));
        SPRINTF(("CANConfig.CAN_ABOM %d:\n", CAN_InitEx.CAN_ABOM));
        SPRINTF(("CANConfig.CAN_BRP: %d \n",CAN_InitEx.CAN_BRP));
        SPRINTF(("CANConfig.CAN_BS1:%d\n",CAN_InitEx.CAN_BS1));
        SPRINTF(("CANConfig.CAN_BS2: %d\n ",CAN_InitEx.CAN_BS2));
        SPRINTF(("CANConfig.CAN_Mode: %d\n",CAN_InitEx.CAN_Mode));
        SPRINTF(("CANConfig.CAN_NART: %d\n ",CAN_InitEx.CAN_NART));
        SPRINTF(("CANConfig.CAN_RELAY: %d\n ",CAN_InitEx.CAN_RELAY));
        SPRINTF(("CANConfig.CAN_RFLM: %d\n ",CAN_InitEx.CAN_RFLM));
        SPRINTF(("CANConfig.CAN_SJW: %d\n ",CAN_InitEx.CAN_SJW));
        SPRINTF(("CANConfig.CAN_TXFP: %d\n ",CAN_InitEx.CAN_TXFP));
#endif

		if(Status==STATUS_ERR){
			SPRINTF(("Init device failed!\n"));
			return 0;
		}else{
			SPRINTF(("Init device success!\n"));
		}
	}		
	
    //Set filter
	VCI_FILTER_CONFIG CAN_FilterConfig;
    CAN_FilterConfig.FilterIndex = 0;
    CAN_FilterConfig.Enable = 1;		
    CAN_FilterConfig.ExtFrame = 0;
    CAN_FilterConfig.FilterMode = 0;
    CAN_FilterConfig.ID_IDE = 0;
    CAN_FilterConfig.ID_RTR = 0;
    CAN_FilterConfig.ID_Std_Ext = 0;
    CAN_FilterConfig.MASK_IDE = 0;
    CAN_FilterConfig.MASK_RTR = 0;
    CAN_FilterConfig.MASK_Std_Ext = 0;

	for(int canIdx=CAN_CHANNEL_START; canIdx<=CAN_CHANNEL_END; canIdx++){
		canIndex2=canIdx;
		Status = VCI_SetFilter(devType2, devIndex2, canIndex2, &CAN_FilterConfig);
		for(i=1; i<=13; i++)
		{
			CAN_FilterConfig.FilterIndex = i;
			Status = VCI_SetFilter(devType2, devIndex2, canIndex2,&CAN_FilterConfig);
		}
		if(Status==STATUS_ERR){
			SPRINTF(("Set filter failed!\n"));
			return 0;
		}else{
			SPRINTF(("can index:%d Set filter success!\n", canIndex2));
		}

		//Register callback function
	#if CAN_CALLBACK_READ_DATA
		VCI_RegisterReceiveCallback(devIndex2, GetDataCallback2);
		//VCI_RegisterReceiveCallback(1, GetDataCallback2);
	#endif

		//Start CAN
		Status = VCI_StartCAN(devType2, devIndex2, canIndex2);
		if(Status==STATUS_ERR){
			SPRINTF(("Start CAN:%d failed!\n", canIndex2));
			return 0;
		}else{
			SPRINTF(("Start CAN%d success!\n", canIndex2));
		}
	}
#else
	VCI_INIT_CONFIG	CAN_Init;
    //Config device
	CAN_Init.AccCode = 0x00000000;
	CAN_Init.AccMask = 0xFFFFFFFF;
	CAN_Init.Filter = 1;
	CAN_Init.Mode = 0;
	CAN_Init.Timing0 = 0x00;
	CAN_Init.Timing1 = 0x14;
    Status = VCI_InitCAN(devType, devIndex, canIndex,&CAN_Init);
    if(Status==STATUS_ERR){
        SPRINTF(("Init device failed!\n"));
        return 0;
    }else{
        SPRINTF(("Init device success!\n"));
    }
#endif

	canIndex2=CAN_CHANNEL_START;

    //Send data
#if CAN_SEND_DATA
	VCI_CAN_OBJ	CAN_SendData[CAN_DATA_SEND_FRAME_COUNT];//
	for(int j=0;j<CAN_DATA_SEND_FRAME_COUNT;j++){
		//if(j%2)
		CAN_SendData[j].DataLen = 8;
		//else
		//	CAN_SendData[j].DataLen = 4;

		for(int i=0;i<CAN_SendData[j].DataLen;i++){
			CAN_SendData[j].Data[i] = i+j*10;
		}
		CAN_SendData[j].ExternFlag = 0;
		CAN_SendData[j].RemoteFlag = 0;
		CAN_SendData[j].ID = 0x155+j;
#if CAN_MODE_LOOP_BACK
		CAN_SendData[j].SendType = 2;
#else
		CAN_SendData[j].SendType = 0;
#endif//CAN_MODE_LOOP_BACK
		CAN_SendData[j].TimeFlag = 0x0;
		CAN_SendData[j].TimeStamp = 0xaa;
		for(int i=0;i<3;i++){
			CAN_SendData[j].Reserved[i] = 0;
		}

	}
	Status = VCI_Transmit(devType, devIndex, canIndex, CAN_SendData,CAN_DATA_SEND_FRAME_COUNT );
	
//   Status = VCI_Transmit(VCI_USBCAN2,0,0,CAN_SendData,0);
	while(0){
		int j=1;
		//for(j=0; j<CAN_DATA_SEND_FRAME_COUNT; j++)
		for(int canIdx=CAN_CHANNEL_START; canIdx<=CAN_CHANNEL_END; canIdx++)
		{
			canIndex = canIdx;
			CAN_SendData[0].Data[0] = j<<1; 
			Status = VCI_Transmit(devType, devIndex, canIndex, CAN_SendData,CAN_DATA_SEND_FRAME_COUNT );
#ifndef OS_UNIX
				Sleep(1000);
#else
				sleep(1);
#endif
			if(Status == STATUS_ERR){
				SPRINTF(("Send CAN %d data failed!\n", canIndex));
				VCI_ResetCAN(devType, devIndex, canIndex);
			}else{
				SPRINTF(("Send CAN %d data success!\n", canIndex));
			}
		}
	}
#endif//CAN_SEND_DATA

#ifndef OS_UNIX
    Sleep(10);
#else
	sleep(0.01);
#endif

#if CAN_GET_STATUS
            VCI_CAN_STATUS CAN_Status;
            Status = VCI_ReadCANStatus(devType2, devIndex2, canIndex2, &CAN_Status);
            if (Status == STATUS_ERR)
            {
                SPRINTF(("Get CAN status failed!\n"));
                return 1;
            }
            else
            {
                SPRINTF(("Buffer Size : %d\n",CAN_Status.BufferSize));
                SPRINTF(("ESR : 0x%08X\n" ,CAN_Status.regESR));
                SPRINTF(("------Error warning flag : %d\n" ,((CAN_Status.regESR>>0)&0x01)));
                SPRINTF(("------Error passive flag : %d\n" , ((CAN_Status.regESR >> 1) & 0x01)));
                SPRINTF(("------Bus-off flag : %d\n" , ((CAN_Status.regESR >> 2) & 0x01)));
                SPRINTF(("------Last error code(%d) : ",(CAN_Status.regESR>>4)&0x07));
                switch ((CAN_Status.regESR>>4)&0x07)
                {
                    case 0:
                        SPRINTF(("No Error\n"));
                        break;
                    case 1:
                        SPRINTF(("Stuff Error\n"));
                        break;
                    case 2:
                        SPRINTF(("Form Error\n"));
                        break;
                    case 3:
                        SPRINTF(("Acknowledgment Error\n"));
                        break;
                    case 4:
                        SPRINTF(("Bit recessive Error\n"));
                        break;
                    case 5:
                        SPRINTF(("Bit dominant Error\n"));
                        break;
                    case 6:
                        SPRINTF(("CRC Error\n"));
                        break;
                    case 7:
                        SPRINTF(("Set by software\n"));
                        break;
                    default:
                        break;
                }
                SPRINTF(("------Transmit error counter : %d\n" , ((CAN_Status.regESR >> 16) & 0xFF)));
                SPRINTF(("------Receive error counter : %d\n" , ((CAN_Status.regESR >> 24) & 0xFF)));
                SPRINTF(("TSR : 0x%08X\n" , CAN_Status.regTSR));
            }
#endif

#if CAN_READ_DATA
	while(1){
#if ! CAN_CALLBACK_READ_DATA
		canIndex2=0;
		int ReadDataNum;
		int DataNum = VCI_GetReceiveNum(devType2, devIndex2, canIndex2);
		VCI_CAN_OBJ	*pCAN_ReceiveData = (VCI_CAN_OBJ *)malloc(DataNum*sizeof(VCI_CAN_OBJ));
		if((pCAN_ReceiveData != NULL))
		{
			//if(DataNum == 0)
			//{
			//	canIndex2 = (canIndex2==0)?1:0;
			//    DataNum = VCI_GetReceiveNum(devType2, devIndex2, canIndex);	
			//}

			if(DataNum > 0){
			    ReadDataNum = VCI_Receive(devType2, devIndex2, canIndex2, pCAN_ReceiveData, DataNum);
				//数据为四个一行，共6行，对应传感器右下角从下往上的点开始
				for (int i = 0; i < ReadDataNum; i++)
				{
					SPRINTF(("\n"));
					//std::cout << "ReadDataNum=" << ReadDataNum << std::endl;
					SPRINTF(("Can read data:\n"));
					SPRINTF(("--CAN_ReceiveData.RemoteFlag = %d\n",pCAN_ReceiveData[i].RemoteFlag));
					SPRINTF(("--CAN_ReceiveData.ExternFlag = %d\n",pCAN_ReceiveData[i].ExternFlag));
					SPRINTF(("--CAN_ReceiveData.ID = 0x%X\n",pCAN_ReceiveData[i].ID));//0x%X
					row = pCAN_ReceiveData[i].ID / 255;
					col = pCAN_ReceiveData[i].ID % 255 / 16;
					sensorID = pCAN_ReceiveData[i].ID % 256 % 16;
					//对应行号std::cout << "--CAN_ReceiveData.ID =" << pCAN_ReceiveData[i].ID/255<<std::endl;
					//对应列号std::cout << "--CAN_ReceiveData.ID2 =" << pCAN_ReceiveData[i].ID % 255/16 << std::endl;
					SPRINTF(("--CAN_ReceiveData.DataLen = %d\n",pCAN_ReceiveData[i].DataLen));
					SPRINTF(("--CAN_ReceiveData.Data:"));
					for(int j=0;j<pCAN_ReceiveData[i].DataLen;j++){
						SPRINTF(("%02X ",pCAN_ReceiveData[i].Data[j]));
					}
					//将收到的数据调整到测量范围内，并存储数据
					x_force_raw_data[col * 4 + row] = pCAN_ReceiveData[i].Data[1] * 256 + pCAN_ReceiveData[i].Data[2];
					y_force_raw_data[col * 4 + row] = pCAN_ReceiveData[i].Data[3] * 256 + pCAN_ReceiveData[i].Data[4];
					z_force_raw_data[col * 4 + row] = pCAN_ReceiveData[i].Data[5] * 256 + pCAN_ReceiveData[i].Data[6];
					if (1 == sensorID)
					{
						//x_record_pos1 << (float)900.0 * (x_force_raw_data[col * 4 + row] - 32768) / 32768 << " ";
						//y_record_pos1 << (float)900.0 * (y_force_raw_data[col * 4 + row] - 32768) / 32768 << " ";
						//z_record_pos1 << (float)5000.0 * (z_force_raw_data[col * 4 + row] - 35000) / 32768 << " ";
					}
					else
					{
						x_record_pos7 << (float)900.0 * (x_force_raw_data[col * 4 + row] - 32768) / 32768 << " ";
						y_record_pos7 << (float)900.0 * (y_force_raw_data[col * 4 + row] - 32768) / 32768 << " ";
						z_record_pos7 << (float)5000.0 * (z_force_raw_data[col * 4 + row] - 35000) / 32768 << " ";
					}

					SPRINTF(("\n"));
					SPRINTF(("--CAN_ReceiveData.TimeStamp = %d\n\n", pCAN_ReceiveData[i].TimeStamp));
					if ((row == 3) && (col == 5) && (1 == sensorID))
					{
						//x_record_pos1 << std::endl;
						//y_record_pos1 << std::endl;
						//z_record_pos1 << std::endl;
					}
					if ((row == 3) && (col == 5) && (7 == sensorID))
					{
						x_record_pos7 << std::endl;
						y_record_pos7 << std::endl;
						z_record_pos7 << std::endl;
					}
					
				}
			}
		}

		free(pCAN_ReceiveData);
#endif 
	}
#endif

    while(getchar()!='\n');
	
#if CAN_CALLBACK_READ_DATA
	VCI_LogoutReceiveCallback(0);
	SPRINTF(("VCI_LogoutReceiveCallback\n"));
#endif
#ifndef OS_UNIX
	Sleep(10);
#else
	usleep(10*1000);
#endif

	//Stop receive can data
	Status = VCI_ResetCAN(devType2, devIndex2, canIndex2);
	SPRINTF(("VCI_ResetCAN %d\n",Status));

	VCI_CloseDevice(devType2, devIndex2);
	SPRINTF(("VCI_CloseDevice\n"));
	//x_record_pos1.close();
//y_record_pos1.close();
//z_record_pos1.close();
	x_record_pos7.close();
	y_record_pos7.close();
	z_record_pos7.close();
    return 0;
}
