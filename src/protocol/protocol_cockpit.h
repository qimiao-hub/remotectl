/**
************************************************************
* @file         PROTOCOL_COCKPIT.h
* @brief        协议打包头文件
* @author       sany-Yanhao
* @date         2021-08-20
* @version      V0.0.1
* @copyright    sany
***********************************************************/
#ifndef PROTOCOL_COCKPIT_H
#define PROTOCOL_COCKPIT_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "terminal_client.h"
#include <sys/time.h>
#include "FailureReporter.h"
#include "daemon_sdk.h"
#include "read_ini.h"
#include "./common/yaml.h"

#include "../include/ddsType/ChassisCtrl.h"
#include "../include/ddsType/ChassisCtrlPubSubTypes.h"
#include "../include/ddsType/gps_imu_msg.h"
#include "../include/ddsType/gps_imu_msgPubSubTypes.h"
#include "../include/ddsType/FailureReporterInterface.h"
#include "../include/ddsType/FailureReporterInterfacePubSubTypes.h"
#include "../include/ddsType/imple_msg.h"
#include "../include/ddsType/imple_msgPubSubTypes.h"
#include "../include/ddsType/Cockpit_address.h"
#include "../include/ddsType/Cockpit_addressPubSubTypes.h"
#include <zkbs/ddsBase/ddsPubBase.h>
#include <zkbs/ddsBase/ddsSubBase.h>
#include "zkbs/zkbs.hpp"
/*********************************************************************
 * MACROS
 */
#ifndef RET_OK
#define RET_OK 1
#define RET_ERROR 0
#endif

#define SEND_RTDATA_UPLOAD_TICK (100)
#define SEND_STATUS_DATA_UPLOAD_TICK (10)

#define PROTOCOL_COCKPIT_SEND_BUF 40
#define PROTOCOL_COCKPIT_RECV_BUF 40

#define PROTO_PACK_MAX_LEN (1024)
#define PROTO_DATE_MAX_LEN (1024 - 24)

#define Message_Max_Len 65535

#define PROTOCOL_COCKPIT_MAX_RECV_FUN_NUM EV_CLAPP_MAX * 2

#define SEND_PERIOD_US 1000 
#define SEND_LOGIN_TICK (1000 * 1000 / SEND_PERIOD_US)

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

extern FailureReporter *m_pFailureSDK;

typedef struct
{
	SANY_U32 u32SendLen;
	SANY_U8 szDataBuf[PROTO_PACK_MAX_LEN];
} Protocol_Cockpit_Buffer_st;

typedef struct
{
	int iWheelAngle;	  // 车轮转角
	int iThrottleOpening; // 油门开度
	int iBrakeOpening;	  // 制动开度
	int iGearOpening;	  // 档位操作
	int iCombinedGearL;	  // 组合开关左
	int iCombinedGearR;	  // 组合开关右
	int iKeyOperation1;	  // 按键操作1
	int iKeyOperation2;	  // 按键操作2
} RemoteDrivingWirectlInInfo;

typedef struct
{
	float iVehicle_speed;	   // 车速，单位：km/h
	float iHeadingAngle;	   // 航向角
	float iWheelAngleL;		   // 左轮转角
	int iThrottleOpening;	   // 油门开度
	int iBrakeOpening;		   // 制动开度
	float iEngineSpeed;		   // 发动机转速
	float iAccumulatedMileage; // 累计里程
	float iSubtotalMileage;	   // 小计里程
	float iPitchAngle;			//俯仰角
	int iLiftUpStatus;			//举升到位状态
	int iLiftDownStatus;		//下降到位状态
} RemoteDrivingWirectlOutInfo;

/*********************************************************************
 * CLASS
 */
class Protocol_Cockpit
{
public:
	std::thread m_thread;

	SANY_U8 g_bNetReady = 0;
	SANY_U8 g_old_current_state = 0;
	SANY_U8 g_current_state = 0;

	char cockpitServerName[22] = {0};
	const char HostclientName[22] = "192.168.8.249:8080";
//	ini::iniReader config;

	/*遥控控制命令*/
	RemoteDrivingWirectlInInfo struWireCtrlIn;
	/*遥控反馈信息*/
	RemoteDrivingWirectlOutInfo struWireCtrlOut;
	/*进程通信定义分界线----------------------------start*/
	/*底盘反馈信息*/
	WirectlFeedbackInfo WireInfo;
	WirectlFeedbackInfo OldWireInfo; // 用于状态变更时与WireInfo比较
	GpsImuInfo struGpsInfo;

	/*发布参数相关*/
	TopicDataType *pDataTypeWire = new WirectlFeedbackInfoPubSubType();
	TopicDataType *pDataTypeGps = new GpsImuInfoPubSubType();

	ddsSubscriber<WirectlFeedbackInfo> *pSubWire = NULL;
	ddsSubscriber<GpsImuInfo> *pSubGps =  NULL;

	/*进程通信定义分界线----------------------------end*/

	typedef struct
	{
		SANY_U8 Cmd;
		SANY_U8 Ack;
		SANY_U8(Protocol_Cockpit::*pFun)
		(Protocol_Cockpit_Buffer_st *pDateBuffer);
	} Protocol_Cockpit_Recv_Func_st;

public:
	Protocol_Cockpit();

	void start();
	void stop();
	void ProtoCockpit_setServerName(const char *pServerName);

	/*遥控控制信息输出*/
	SANY_U8 ProtoCockpit_getCtrlDate(ImpleResultInfo *pCtrlInfo);
	/*遥控接管使能位和最大车速*/
//	SANY_U8 ProtoCockpit_getMaxSpeed(Remote2Control *pRemoteCtrl);

	/*读取配置文件*/
//	void Read_INI(ini::iniReader config);
	void Read_Config();

private:
	/*heartbeat with 297*/
	SANY_U16 g_count = 0;

	/*登录状态 0未登录 1已登录，未登录状态下，不处理命令协议*/ /*延时程序写死成1*/
	SANY_U8 g_u8LoginFlag = 0;

	/*登录流水号*/
	SANY_U8 g_u8LoginSeqNum = 0;

	/* 发送标识，是否开始实时上报（高频） */
	SANY_U32 g_u32RTDataFlg = 0; // 实时数据上报标识

	/* 发送标识，是否开始实时上报（状态） */
	SANY_U32 g_u32NLDataFlg = 0; // normal data flag

	/* 急停标识 */
	SANY_U8 g_u8EmergencyStopFlg = 0; // normal data flag

	/*任务结束标识，驾舱登出后释放线程资源，置位后四个线程退出*/
	volatile SANY_U8 g_u8TaskStopFlag = 0;

	/*读取配置文件*/
	std::string iccid;
	std::string vin;
	std::string type;
	std::string HardVer = "0101";
	std::string SoftVer = "0213";
	SANY_U32 brake_delay = 0;
	SANY_U32 read_brakeing = 0;
	SANY_U32 v_type = 0;

	pthread_t t_recv_id;
	pthread_t t_upload_data_id;
	pthread_t t_send_id;
	pthread_t t_download_data_id;

	terminal_client m_Terminal;

	/*发送缓冲区*/
	pthread_mutex_t SendBufferMutex; /*做互斥锁，锁住整个发送缓冲区*/
	SANY_U8 g_u8SendBufferWriteIndex = 0;
	SANY_U8 g_u8SendBufferReadIndex = 0;
	SANY_U8 g_u8SendBufferDateLen = 0;
	Protocol_Cockpit_Buffer_st *arrSendBuffer;

	/*接收缓冲区*/
	pthread_mutex_t RecvBufferMutex; /*做互斥锁，锁住整个接收缓冲区*/
	SANY_U8 g_u8RecvBufferWriteIndex = 0;
	SANY_U8 g_u8RecvBufferReadIndex = 0;
	SANY_U8 g_u8RecvBufferDateLen = 0;
	Protocol_Cockpit_Buffer_st *arrRecvBuffer;

	sa_comm_socket_client *m_Socket_CockpitClient;

	/*接收处理函数集合*/
	Protocol_Cockpit_Recv_Func_st arrRecvFunc[PROTOCOL_COCKPIT_MAX_RECV_FUN_NUM];

private:
	/*数据接收线程*/
	void funcRecvData();
	/*数据发送线程*/
	void funcSendData();
	/*数据上行业务处理线程*/
	void funcAppUploadData();
	/*数据下行业务处理线程*/
	void funcAppDownloadData();

	/*数据接收线程静态成员*/
	static void *threadRecvData(void *args);
	/*数据发送线程静态成员*/
	static void *threadSendData(void *args);
	/*数据上行业务处理线程静态成员*/
	static void *threadAppUploadData(void *args);
	/*数据下行业务处理线程静态成员*/
	static void *threadAppDownloadData(void *args);

	/* 通用接口 */
	void SmalltoBig(SANY_U8 *ucSrc, SANY_U8 *ucDest, SANY_S32 iType);
	SANY_U8 ProtoCockpit_PackHeader(SANY_U8 u8Cmd, SANY_U8 u8AckFlg, CP_Header_st *pst_DataBuf);
	SANY_U8 ProtoCockpit_CalcXOR(SANY_U8 *pu8DataBuf, SANY_S32 s32auhXorSize);
	SANY_U8 ProtoCockpit_GetLocalTime(SANY_U8 *pucTime);
	SANY_U8 ProtoCockpit_GetSequenceNum(SANY_U8 *pu8SeqNum);
	SANY_U8 ProtoCockpit_GetICCID(SANY_U8 *pu8Iccid);

	/*命令协议处理接口数组初始化*/
	void InitFuncArray_Cockpit();
	/*函数查找接口*/
	SANY_U8(Protocol_Cockpit::*GetRecvProc_Cockpit(SANY_U8 u8Cmd, SANY_U8 u8Ack))
	(Protocol_Cockpit_Buffer_st *);

	/* 命令协议发送接口，内部实现组包、发送；后续考虑将组包和发送分开*/
	SANY_U8 ProtoCockpit_SendCmdLogin();
	SANY_U8 ProtoCockpit_SendCmdRealtimeRep();
	SANY_U8 ProtoCockpit_SendCmdRealtimeStatusRep();
	/*发送通用ACK*/
	SANY_U8 ProtoCockpit_SendAckCom(SANY_U8 ucCmd, SANY_U8 ucAckflg);

	/*协议解析分发处理接口*/
	SANY_U8 ProtoCockpit_ParsePacket(SANY_U8 *pu8DataBuf, SANY_U32 u32DataLen);

	/*协议处理接口，内部实现协议解析、协议组包、协议发送*/
	SANY_U8 ProtoCockpit_RecvCmdRepCtrl(Protocol_Cockpit_Buffer_st *pDateBuffer);
	SANY_U8 ProtoCockpit_RecvCmdRealtimeCtrl(Protocol_Cockpit_Buffer_st *pDateBuffer);
	SANY_U8 ProtoCockpit_RecvCmdEmergencyStop(Protocol_Cockpit_Buffer_st *pDateBuffer);
	SANY_U8 ProtoCockpit_RecvCmdLoginOut(Protocol_Cockpit_Buffer_st *pDateBuffer);
	SANY_U8 ProtoCockpit_RecvAckLoginPacket(Protocol_Cockpit_Buffer_st *pDateBuffer);

	/*协议组包接口*/
	SANY_U8 ProtoCockpit_PackAckRealtimeCtrl(SANY_U8 *pDataBuf, SANY_U32 *pOutDataLen, SANY_U8 ucSequenceNum);
	SANY_U8 ProtoCockpit_PackAckEmergencyStop(SANY_U8 *pDataBuf, SANY_U32 *pOutDataLen, SANY_U8 ucSequenceNum);
	SANY_U8 ProtoCockpit_PackAckLoginOut(SANY_U8 *pDataBuf, SANY_U32 *pOutDataLen);
	SANY_U8 ProtoCockpit_PackAckRepCtrl(SANY_U8 *pDataBuf, SANY_U32 *pOutDataLen, SANY_U8 ucSequenceNum);

	/*网络相关接口*/
	void InitSocket_Cockpit();

	SANY_U8 ProtoCockpit_WriteSendBuffer(SANY_U8 *pu8DataBuf, SANY_U32 u32DataLen);
	SANY_U8 ProtoCockpit_ReadSendBuffer(SANY_U8 *pu8DataBuf, SANY_U32 *u32DataLen);
	void ProtoCockpit_SendBufferInit(void);

	SANY_U8 ProtoCockpit_WriteRecvBuffer(SANY_U8 *pu8DataBuf, SANY_U32 u32DataLen);
	SANY_U8 ProtoCockpit_ReadRecvBuffer(SANY_U8 *pu8DataBuf, SANY_U32 *u32DataLen);
	void ProtoCockpit_RecvBufferInit(void);

	/*遥控反馈信息整合到 struWireCtrlOut*/
	SANY_U8 ProtoCockpit_getFeedbackData();
	/*遥控控制信息初始化*/
	SANY_U8 ProtoCockpit_initCtrlData();
};

#endif
