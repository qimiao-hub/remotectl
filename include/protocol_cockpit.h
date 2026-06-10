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
#include <algorithm>
#include "terminal_client.h"
#include <sys/time.h>
#include <sys/timeb.h>
#include "FailureReporter.h"
#include "daemon_sdk.h"
#include "./common/read_ini.h"
#include "./common/yaml.h"


#include "./ddsType/Cockpit_address.h"
#include "./ddsType/Cockpit_addressPubSubTypes.h"

// dvpn-pnc-common规控common库
#include "../dvpn-pnc-common/ddsType/include/gps_imu_msg.h"
#include "../dvpn-pnc-common/ddsType/include/gps_imu_msgPubSubTypes.h"
#include "../dvpn-pnc-common/ddsType/include/ChassisCtrl.h"
#include "../dvpn-pnc-common/ddsType/include/ChassisCtrlPubSubTypes.h"
// #include "../include/ddsType/FailureReporterInterface.h"
// #include "../include/ddsType/FailureReporterInterfacePubSubTypes.h"
#include "../dvpn-pnc-common/ddsType/include/imple_msg.h"
#include "../dvpn-pnc-common/ddsType/include/imple_msgPubSubTypes.h"
#include "../dvpn-pnc-common/ddsType/include/safe_ctrl_msg.h"
#include "../dvpn-pnc-common/ddsType/include/safe_ctrl_msgPubSubTypes.h"
#include "../dvpn-pnc-common/ddsType/include/obst_msg.h"
#include "../dvpn-pnc-common/ddsType/include/obst_msgPubSubTypes.h"
#include "../include/polygon_boundary.h"

#include "remote_config.h"

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

#define SEND_RTDATA_UPLOAD_TICK (200)
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
extern Remote_Config *m_Config;
extern SANY_U8 g_u8IpFlag;
extern SANY_U8 g_u8TaskStartFlag;
extern SANY_U8 g_u8NetReady;

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
	int iSafeRelease;			// 安全措施-清除故障
	int iSafeProtect;		// 安全措施-进入安全模式
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
	int iGearOpening;	  		// 档位操作
	int iRemainingOil;			//剩余油量/电量
	int iWaterTemp;				//水温
	float iBrakeAirPre1;			//气压1
	float iBrakeAirPre2;			//气压2
	int	iCombinedGearL;			//组合开关左
    int	iCombinedGearR;			//组合开关右
    int	iKeyOperation1;			//按键操作1
    int	iKeyOperation2;			//按键操作2 
	int	iWorkStatus;			//车辆驾驶状态
	int iSafeInfo1;				// 安全模块-故障1
	int iSafeInfo2;				// 安全模块-故障2
} RemoteDrivingWirectlOutInfo;

/*********************************************************************
 * CLASS
 */
class Protocol_Cockpit
{
public:
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
	~Protocol_Cockpit();
	
	void InitProtocol();
	void Task_start();
	void Task_stop();
	void Task_Reset();
	SANY_U8 ProtoCockpit_DownshiftDecision();
	// void ProtoCockpit_setServerName(const char *pServerName);

	/*遥控控制信息输出*/
	SANY_U8 ProtoCockpit_getCtrlDate(ImpleResultInfo *pCtrlInfo);
	/*遥控接管使能位和最大车速*/
//	SANY_U8 ProtoCockpit_getMaxSpeed(Remote2Control *pRemoteCtrl);

	/*读取配置文件*/
//	void Read_INI(ini::iniReader config);
	// void Read_Config();
	/*溜车判断-计算车速航向*/
	float ProtoCockpit_SpeedHeading(const float gps_vn, const float gps_ve, const float heading);
	SANY_U8 ProtoCockpit_Rolling();
	/* 20250925 坡起策略*/
	float ProtoCockpit_SlopeStart();
	/*心跳、控车数据监测*/
	SANY_U8 ProtoCockpit_HeartCheck(const SANY_U32 Losttimes);
	/*断连/心跳丢失停车*/
	SANY_U8 ProtoCockpit_DoStop();
	/*遥控控制信息初始化*/
	SANY_U8 ProtoCockpit_initCtrlData();
	/*发送通用ACK*/
	SANY_U8 ProtoCockpit_SendAckCom(SANY_U8 ucCmd, SANY_U8 ucAckflg);
	/*遥控刹车启动前初始化 刹车 档位 油门 主车*/
	SANY_U8 ProtoCockpit_CheckBrakeLimit();
	/*202605 排土场倒车策略*/
	SANY_U8 ProtoCockpit_DumpBackSafeCheck();
	/*排土场倒车限扭*/
	bool ProtoCockpit_dumpBackThrottleLimit();
	// 排土场边界判断
	bool ProtoCockpit_GeneralBackCurbCheck(double temp_gpsLng, double temp_gpsLat, std::string dump_id);
	/*排土场倒车俯仰角异常*/
	bool ProtoCockpit_dumpBackPitchCheck();
	float GetNormalizedPitch();
	void CalibrateBasePitch();
	void ReEvaluatePitchStable();
	void pitch_Attitude_montoring();

public:
/*配置文件*/
	std::string vehicle_iccid;
	std::string vehicle_vin;
	std::string vehicle_type;
	std::string vehicle_HardVer = "0101";
	std::string vehicle_SoftVer = "0222";//02软件类型 20软件版本号 20不调整，21调整气压速度显示小数点后一位，增加刹车提醒；22 调整130 车型、水温显示

	/*登录状态 0未登录 1已登录，未登录状态下，不处理命令协议*/ /*延时程序写死成1*/
	SANY_U8 g_u8LoginFlag = 0;
	/*响应驾驶模式状态 0 默认，1 人工，2 遥控， 3 无人*/
	SANY_U8 g_u8ResponseCtrlState = 0;

private:
	/*排土场倒车策略*/
	bool g_boundary_load = false;
	bool g_task_main_dump = false;
	bool g_task_subState_back = false;
	// 到挡墙的距离
	float g_dumpBack_minD1 = 0.0f;
	float g_dumpBack_dist_to_end = 0.0f;

	// 边界计算
	Polygon_Boundary *m_polygonBodary;
	bool g_dumpBack_curb_force_stop = false; // 边界停车标志
	
	//////// * 油门限纽 */
	float g_dumpBack_throttle_compensation = 1.0f;
	bool g_dumpBack_throttle_limit = false;
	

	////////////* 俯仰角异常 */
    float stop_park_counter = 0.0f;
	float stop_stable_counter = 0.0f;
	float vehilce_dump_base_pitch = 0.0f;		// 标定后的基准俯仰角
	float vehicle_pitch_pair_value = 0.0f;
	int peak_high = 0;
    int trough_low = 0;
    int cycle_count = 0;
    bool g_dumpBack_pitch_abnormal = false;		// 俯仰角异常标志
	bool g_dumpBack_pitch_force_stop = false;	// 俯仰角异常停车标志

    
	std::deque<float> local_pich_record;		// 检查俯仰角异常记录
	std::deque<float> base_pitch_buffer_;     // 基础俯仰角标定缓冲区
	std::deque<float> stable_pitch_buffer_;  // 停车稳定后采集俯仰角缓冲区       	
private:
	/*heartbeat with 297*/
	SANY_U16 g_297count = 0;

	/*登录流水号*/
	SANY_U8 g_u8LoginSeqNum = 0;

	/* 急停标识 */
	SANY_U8 g_u8EmergencyStopFlg = 0; // normal data flag

	/*溜车标识*/
	SANY_U8 g_u8RollNeedStopFlg = 0;		// normal data flag
	
	/*请求驾驶模式状态， 0 默认，1 遥控，2 无人，3 人工*/
	SANY_U8 g_u8CtrlState = 0;
	


	/* 刹车60启动标志位*/
	SANY_U32 g_BrakeLimitCount = 0;

	/*缓刹*/
	// SANY_U32 brake_delay = 40;
	SANY_U32 read_brakeing = 50;
	SANY_U32 brake_count = 0;
	float BrakeValue;
	SANY_U32 BRAKE_DELAY_THRESHOLD = 40;	// 刹车延迟时间阈值
	float mass_coefficient_thr = 1.2f; // 重载系数
	float DANGER_PITCH_THRESHOLD = 2.0f;	// 触发坡起的俯仰角度数
	SANY_U32 RECOVERY_TIME_THRESHOLD = 200;	// 1秒，坡起波动取消的时间阈值
	SANY_U32 recovery_count = -1;

	/*线程id*/
	pthread_t t_upload_data_id;
	pthread_t t_download_data_id;
	pthread_t t_task_id;

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
	SANY_U8 g_u8SendBufferFull = 0;		//发送缓冲区写满标志
	Protocol_Cockpit_Buffer_st *arrRecvBuffer;

	/*遥控控制命令*/
	RemoteDrivingWirectlInInfo struWireCtrlIn;
	/*遥控反馈信息*/
	RemoteDrivingWirectlOutInfo struWireCtrlOut;

	/*接收处理函数集合*/
	Protocol_Cockpit_Recv_Func_st arrRecvFunc[PROTOCOL_COCKPIT_MAX_RECV_FUN_NUM];

private:
	/*数据上行业务处理线程*/
	void funcAppUploadData();
	/*数据下行业务处理线程*/
	void funcAppDownloadData();
	/*任务线程*/
	void funcTask();

	/*数据上行业务处理线程静态成员*/
	static void *threadAppUploadData(void *args);
	/*数据下行业务处理线程静态成员*/
	static void *threadAppDownloadData(void *args);
	/*任务线程静态成员*/
	static void *threadTask(void *args);

private:
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
	
	/*排土场倒车策略上报*/
	SANY_U8 ProtoCockpit_SendCmdDumpBackSafeRep();

	/*协议解析分发处理接口*/
	SANY_U8 ProtoCockpit_ParsePacket(SANY_U8 *pu8DataBuf, SANY_U32 u32DataLen);

	/*协议处理接口，内部实现协议解析、协议组包、协议发送*/
	SANY_U8 ProtoCockpit_RecvCmdRepCtrl(Protocol_Cockpit_Buffer_st *pDateBuffer);
	SANY_U8 ProtoCockpit_RecvCmdRealtimeCtrl(Protocol_Cockpit_Buffer_st *pDateBuffer);
	SANY_U8 ProtoCockpit_RecvCmdEmergencyStop(Protocol_Cockpit_Buffer_st *pDateBuffer);
	SANY_U8 ProtoCockpit_RecvCmdHeartBeat(Protocol_Cockpit_Buffer_st *pDateBuffer);
	SANY_U8 ProtoCockpit_RecvCmdLoginOut(Protocol_Cockpit_Buffer_st *pDateBuffer);
	SANY_U8 ProtoCockpit_RecvAckLoginPacket(Protocol_Cockpit_Buffer_st *pDateBuffer);
	
	/*协议组包接口*/
	SANY_U8 ProtoCockpit_PackAckEmergencyStop(SANY_U8 *pDataBuf, SANY_U32 *pOutDataLen, SANY_U8 ucSequenceNum);
	SANY_U8 ProtoCockpit_PackAckHeartBeat(SANY_U8 *pDataBuf, SANY_U32 *pOutDataLen, SANY_U16 ucSequenceNum);
	SANY_U8 ProtoCockpit_PackAckLoginOut(SANY_U8 *pDataBuf, SANY_U32 *pOutDataLen);
	SANY_U8 ProtoCockpit_PackAckRepCtrl(SANY_U8 *pDataBuf, SANY_U32 *pOutDataLen, SANY_U8 ucSequenceNum);
	/*打包安全模块上报的故障信息*/
	SANY_U8 ProtoCockpit_PackSafeFeedbackInfo();
	/*297反馈信息整合到 struWireCtrlOut*/
	SANY_U8 ProtoCockpit_getFeedbackData();
	

/*dds相关*/
private:
	/*底盘数据反馈*/
	TopicDataType *pDataTypeWire = NULL;
	TopicDataType *pDataTypeGps = NULL;
	ddsSubscriber<WirectlFeedbackInfo> *pSubWire = NULL;
	ddsSubscriber<GpsImuInfo> *pSubGps =  NULL;

	/*安全模块反馈*/
	TopicDataType *pDataTypeSafe = NULL;
	ddsSubscriber<SafeFeedbackInfo> *pSubSafe = NULL;
	/*后向挡墙*/
	DDS_FusionCurbMessage m_fusionCurbMsg;
    TopicDataType *pDataFusionCurbMsg = NULL;
    ddsSubscriber<DDS_FusionCurbMessage> *pSubFusionCurbMsg = NULL;
	/*挡墙边界回调*/
    SANY_VOID fusionCurbMsgCallback(DDS_FusionCurbMessage &data);
	
	/*发布遥控驾驶控制数据*/
	ImpleResultInfo imple_result_info;
	TopicDataType *pDataTypeImple = NULL;
	ddsPublisher<ImpleResultInfo> *pPubImple = NULL;

public:
	/*网络相关接口*/
	// void InitSocket_Cockpit();

	SANY_U8 ProtoCockpit_WriteSendBuffer(SANY_U8 *pu8DataBuf, SANY_U32 u32DataLen);
	SANY_U8 ProtoCockpit_ReadSendBuffer(SANY_U8 *pu8DataBuf, SANY_U32 *u32DataLen);
	void ProtoCockpit_SendBufferInit(void);

	SANY_U8 ProtoCockpit_WriteRecvBuffer(SANY_U8 *pu8DataBuf, SANY_U32 u32DataLen);
	SANY_U8 ProtoCockpit_ReadRecvBuffer(SANY_U8 *pu8DataBuf, SANY_U32 *u32DataLen);
	void ProtoCockpit_RecvBufferInit(void);
};

#endif
