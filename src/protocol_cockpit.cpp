/*********************************************************************************
**  Copyright(C),2020-2021, Sany
**  FileName:       protocol_cockpit.cpp
**  Author:         yanhao
**  Version:        0-0-1
**  Date:           2021/08/20
**  Description:    车-舱通信协议实现
**  Function List:
	  1.
**  History:
	  <Author>      <Date>      <Version>      <Description>
	   linzhipeng   2021/08/20      0-0-1        创建源文件说明
**  Others:
***********************************************************************************/
/*********************************************************************
 * INCLUDES
 */
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>

#include "sany_utils.h"
#include "../include/protoDef.h"
#include "../include/protocol_cockpit.h"


		
/*底盘反馈信息*/
static WirectlFeedbackInfo WireInfo;
static GpsImuInfo struGpsInfo;
/*safe模块-故障信息*/
static SafeFeedbackInfo struSafeFeedbackInfo;


void wireFeedbackCallback(WirectlFeedbackInfo *info)
{
	if(1 == g_u8TaskStartFlag){
		// memcpy(&pCockpit->OldWireInfo, &pCockpit->WireInfo, sizeof(WirectlFeedbackInfo));
		memcpy(&WireInfo, info, sizeof(WirectlFeedbackInfo));
	}
	return;
}

void gpsFeedbackCallback(GpsImuInfo *info)
{
	if(1 == g_u8TaskStartFlag){
		memcpy(&struGpsInfo, info, sizeof(GpsImuInfo));
	}
	return;
}

void safeFeedbackCallback(SafeFeedbackInfo *info)
{
	if(1 == g_u8TaskStartFlag){
		memcpy(&struSafeFeedbackInfo, info, sizeof(SafeFeedbackInfo));
	}
	return;
}



/********************************************************************
 * 函数名称:      Protocol_Cockpit构造函数
 * 描述:        将该协议中相关全局变量进行初始化
 *
 * 参数:        无
 * 返回值:       无
 *
 *********************************************************************/
Protocol_Cockpit::Protocol_Cockpit()
{
	
}

Protocol_Cockpit::~Protocol_Cockpit(){
	/*等待线程退出*/
	pthread_join(t_upload_data_id, NULL);
	pthread_join(t_download_data_id, NULL);
	
	/*释放动态申请的内存空间*/
	free(arrSendBuffer);
	free(arrRecvBuffer);
	MINE_LOG(SANY_LOG_ERROR, "STOP,  free up send/recv buffer!!!\n");

	/*销毁互斥锁*/
	pthread_mutex_destroy(&SendBufferMutex);
	pthread_mutex_destroy(&RecvBufferMutex);
	MINE_LOG(SANY_LOG_ERROR, "STOP, destroy pthread mutex!!!\n");
}

void Protocol_Cockpit::InitProtocol(){
	/* 初始化变量 */
	g_u8NetReady = 0;
	g_u8LoginFlag = 0;
	g_u8LoginSeqNum = 0;
	g_u8CtrlState = 0;
	g_u8EmergencyStopFlg = 0;
	g_u8RollNeedStopFlg = 0;
	g_BrakeLimitCount = 0;

	/*底盘数据相关订阅*/
	pDataTypeWire = new WirectlFeedbackInfoPubSubType();
	pSubWire = new ddsSubscriber<WirectlFeedbackInfo>(pDataTypeWire, &wireFeedbackCallback);
	MINE_LOG(SANY_LOG_INFO, "CREATE WIRE SUBSCRIBER!!\n");
	pDataTypeGps = new GpsImuInfoPubSubType();
	pSubGps =  new ddsSubscriber<GpsImuInfo>(pDataTypeGps, &gpsFeedbackCallback);
	MINE_LOG(SANY_LOG_INFO, "CREATE GPS SUBSCRIBER!!\n");

	/*安全模块订阅*/
	pDataTypeSafe = new SafeFeedbackInfoPubSubType();
	pSubSafe = new ddsSubscriber<SafeFeedbackInfo>(pDataTypeSafe, &safeFeedbackCallback);
	MINE_LOG(SANY_LOG_INFO, "CREATE SAFE SUBSCRIBER!!\n");

	/*遥控驾驶控制数据*/
	pDataTypeImple = new ImpleResultInfoPubSubType();
	pPubImple = new ddsPublisher<ImpleResultInfo>(pDataTypeImple);
	MINE_LOG(SANY_LOG_INFO, "CREATE CONTROL_DATA PUBLISHER!!\n");

	// 初始化数据结构体
	bzero(&struWireCtrlIn, sizeof(struWireCtrlIn));
	bzero(&struWireCtrlOut, sizeof(struWireCtrlOut));
	bzero(&WireInfo, sizeof(WireInfo));
	bzero(&struGpsInfo, sizeof(struGpsInfo));
	bzero(&imple_result_info, sizeof(imple_result_info));
	bzero(&struSafeFeedbackInfo, sizeof(struSafeFeedbackInfo));

	// 初始化数据缓冲区
	ProtoCockpit_SendBufferInit();
	ProtoCockpit_RecvBufferInit();

	/*命令协议处理接口数组初始化*/
	InitFuncArray_Cockpit();

	// 初始化业务处理线程
	if (pthread_create(&t_upload_data_id, NULL, threadAppUploadData, this) == -1)
	{
		MINE_LOG(SANY_LOG_ERROR, "create threadAppUploadData thread error!\n");
	}
	if (pthread_create(&t_download_data_id, NULL, threadAppDownloadData, this) == -1)
	{
		MINE_LOG(SANY_LOG_ERROR, "create threadAppDownloadData thread error!\n");
	}
	if (pthread_create(&t_task_id, NULL, threadTask, this) == -1)
	{
		MINE_LOG(SANY_LOG_ERROR, "create threadTask thread error!\n");
	}
}

/*重置变量*/
void Protocol_Cockpit::Task_Reset(){
	/* 初始化变量 */
	g_u8LoginFlag = 0;
	g_u8LoginSeqNum = 0;
	g_u8CtrlState = 0;
	g_u8EmergencyStopFlg = 0;
	g_u8RollNeedStopFlg = 0;
	g_BrakeLimitCount = 0;
	MINE_LOG(SANY_LOG_INFO, "Clear Flags\n");

	bzero(&struWireCtrlIn, sizeof(struWireCtrlIn));
	bzero(&struWireCtrlOut, sizeof(struWireCtrlOut));
	bzero(&WireInfo, sizeof(WireInfo));
	bzero(&struGpsInfo, sizeof(struGpsInfo));
	bzero(&imple_result_info, sizeof(imple_result_info));
	bzero(&struSafeFeedbackInfo, sizeof(struSafeFeedbackInfo));
	MINE_LOG(SANY_LOG_INFO, "Clear structDatas\n");

	// pthread_mutex_lock(&RecvBufferMutex);
	// g_u8RecvBufferWriteIndex = 0;
	// g_u8RecvBufferReadIndex = 0;
	// g_u8RecvBufferDateLen = 0;
	// memset(arrRecvBuffer,0,sizeof(Protocol_Cockpit_Buffer_st) * PROTOCOL_COCKPIT_RECV_BUF);
	// // for (int i = 0; i < PROTOCOL_COCKPIT_RECV_BUF; ++i) {
	// // 	// 确保将缓冲区的每个元素都初始化为0或其初始状态
	// // 	memset(&arrRecvBuffer[i], 0, sizeof(Protocol_Cockpit_Buffer_st));
	// // }
	// pthread_mutex_unlock(&RecvBufferMutex);
	// MINE_LOG(SANY_LOG_INFO, "Clear RecvBuffer finish\n");

	// pthread_mutex_lock(&SendBufferMutex);
	// g_u8SendBufferWriteIndex = 0;
	// g_u8SendBufferReadIndex = 0;
	// g_u8SendBufferDateLen = 0;
	// memset(arrSendBuffer,0,sizeof(Protocol_Cockpit_Buffer_st) * PROTOCOL_COCKPIT_SEND_BUF);
	// // for (int i = 0; i < PROTOCOL_COCKPIT_SEND_BUF; ++i) {
	// // 	// 确保将缓冲区的每个元素都初始化为0或其初始状态
	// // 	memset(&arrSendBuffer[i], 0, sizeof(Protocol_Cockpit_Buffer_st));
	// // }
	// pthread_mutex_unlock(&RecvBufferMutex);
	// MINE_LOG(SANY_LOG_INFO, "Clear SendBuffer finish\n");
}

/*模块业务开始接口，做收发缓冲区初始化，网络初始化，收发业务线程启动*/
void Protocol_Cockpit::Task_start()
{
	MINE_LOG(SANY_LOG_INFO, "ONE REMOTE TASK START - begin\n");
	// sany_log_set_output(logHandleLocal, SANY_LOG_OUT_FILE);
	// /*clear FailiureStatus*/
	m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 2, 0);
	m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 3, 0);
	m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 8, 0);
	m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 9, 0);
	m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 10, 0);
	m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 11, 0);
	m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 12, 0);
	m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 13, 0);
	m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3202, 1, 0);
	m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3203, 5, 0);
	m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3204, 1, 0);
	m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3204, 3, 0);
	g_u8LoginFlag = 0;
	g_u8TaskStartFlag = 1;
	// sleep(2);
	MINE_LOG(SANY_LOG_INFO, "ONE REMOTE TASK START\n");
}

/*模块业务关闭接口，做资源的反初始化*/
void Protocol_Cockpit::Task_stop()
{
	MINE_LOG(SANY_LOG_INFO, "ONE REMOTE TASK END\n");
	if(g_u8CtrlState == 1)
	{
		g_u8CtrlState = 2;
		MINE_LOG(SANY_LOG_INFO, "STOP, 1s back to unman\n");
		sleep(2);
	}
	// g_u8NetReady = 0;
	g_u8TaskStartFlag = 0;
	/*延时2S等待，线程退出*/
	sleep(2);

	Task_Reset();
	m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 15, 0);
	m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 16, 0);
	m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 8, 0);
	m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 2, 0);
	m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 3, 0);
	m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3202, 1, 0);
	m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3203, 5, 0);	
	m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3204, 1, 0);
	m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3204, 3, 0);
	MINE_LOG(SANY_LOG_INFO, "report failure clear! \n");
	g_u8IpFlag = 0;

	MINE_LOG(SANY_LOG_INFO, "ONE REMOTE TASK END, g_u8CtrlState is %d,  g_u8TaskStartFlag is %d, g_u8IpFlag is %d\n", g_u8CtrlState, g_u8TaskStartFlag, g_u8IpFlag);
}

/*任务线程*/
void Protocol_Cockpit::funcTask(){
	SANY_U8 Remote_Enable5 = 0;
	SANY_U32 Tick = 0;
	SANY_U8 Tick_Remote_Notice = -1;
	int flag = 0;
	MINE_LOG(SANY_LOG_INFO, "funcTask thread START!\n");
	while (true)
	{
		if(1 == g_u8TaskStartFlag)
		{
			if(0 == flag){
				flag = 1;
				MINE_LOG(SANY_LOG_INFO, "funcTask thread restart!\n");
			}
			if (1 == g_u8CtrlState)
			{
				/*获取通信模块遥控控制数据*/
				ProtoCockpit_getCtrlDate(&imple_result_info);
				/*遥控提示现场*/
				if(Tick_Remote_Notice == -1){
					Tick_Remote_Notice = 0;
				}
				Tick_Remote_Notice ++;
				if(Tick_Remote_Notice % 210 >= 200 && Tick_Remote_Notice % 210 < 210){
					imple_result_info.horn_start(true);
					MINE_LOG(SANY_LOG_INFO, "Remote-control is running, notice Horn\n");
				}
				if(Tick_Remote_Notice % 210 == 0){
					Tick_Remote_Notice = 0;
				}
				imple_result_info.Remote_enable(3);
				imple_result_info.unmanned_enable(1);
				imple_result_info.remote_unman_flag(8);
				/*发布遥控控制数据*/
				pPubImple->publish(imple_result_info);
				Remote_Enable5 = 0;
			}
			else if((2 == g_u8CtrlState) ||  (3 == g_u8CtrlState))
			{
				if(10 > Remote_Enable5)
				{
					Tick_Remote_Notice = -1;
					ProtoCockpit_getCtrlDate(&imple_result_info);
					imple_result_info.Remote_enable(5);
					imple_result_info.unmanned_enable(1);
					/* 20250220 liuss 将该物理按键改为130H补气开关：1.130无法加油补气；2. 领导指定排气制动；-----start*/
					imple_result_info.Parking_airSup_state(false);
					/* 20250220 liuss 将该物理按键改为130H补气开关：1.130无法加油补气；2. 领导指定排气制动；-----end*/
					pPubImple->publish(imple_result_info);
					Remote_Enable5++;
					// 130
					MINE_LOG(SANY_LOG_INFO, "exit from remote-control, Remote_enable: %d, current Gear: %d, BrakeOpening: %f, Parking State: %d, Parking airSup State: %d\n",
						imple_result_info.Remote_enable(), imple_result_info.trans_req_gear(), imple_result_info.traffic_brake_req(), imple_result_info.parking_req(), imple_result_info.Parking_airSup_state());
					// 105
					MINE_LOG(SANY_LOG_INFO, "exit from remote-control, Remote_enable: %d, current Gear: %d, BrakeOpening: %f, Parking State: %d\n",imple_result_info.Remote_enable(), imple_result_info.trans_req_gear(), imple_result_info.traffic_brake_req(), imple_result_info.parking_req());
			
				}	
			}
		}else{
			if(1 == flag){
				flag = 0;
				MINE_LOG(SANY_LOG_INFO, "funcTask thread stop!\n");
			}
			Remote_Enable5 = 0;
			Tick_Remote_Notice = -1;
		}
		Tick++;
		if((Tick % 100) == 0)
		{
			MINE_LOG(SANY_LOG_INFO, "funcTask is running, g_u8IpFlag is %d, g_u8TaskStartFlag is %d\n", g_u8IpFlag, g_u8TaskStartFlag);
			// Tick = 0;
		}
		usleep(50 * 1000);
	}
}

/*任务线程*/
void *Protocol_Cockpit::threadTask(void *args)
{
	Protocol_Cockpit *pThread = static_cast<Protocol_Cockpit *>(args);
	pThread->funcTask();
}



/**************************************************************************
* 函数名称:      传输包头封装
* 描述:        该函数用于对协议消息头进行封装
* 参数:        u8 u8Cmd                         命令标识
			 u8 u8AckFlg                      应答标志   ：0x01 接收到的信息正确，
														0x02设置未成功
														0x03 VIN重复错误，
														0x04表示数据包为命令包，而非应答包
			 CP_Header_st * pst_DataBuf      数据缓存变量
*
* 返回值:       u8XorByte
*
***************************************************************************/
SANY_U8 Protocol_Cockpit::ProtoCockpit_PackHeader(SANY_U8 u8Cmd, SANY_U8 u8AckFlg, CP_Header_st *pst_DataBuf)
{
	if (EV_WLAPP_MAX <= u8Cmd)
	{
		MINE_LOG(SANY_LOG_ERROR, "Error cmd %d\n", u8Cmd);
		return -1;
	}

	pst_DataBuf->ucHeader[0] = 0x23;
	pst_DataBuf->ucHeader[1] = 0x23;
	pst_DataBuf->ucCommand[0] = u8Cmd;
	pst_DataBuf->ucAckflg[0] = u8AckFlg;
	/*-------------------------------------------------后续添加VIN------------------------------------------------------------------------*/
	memcpy((SANY_S8 *)pst_DataBuf->ucVIN, vehicle_vin.c_str(), WP_VIN_SIZE); // #define VIN "SKT90202011300002"
	pst_DataBuf->ucEncryption[0] = EV_WP_NO_ENCRYPT;

	return RET_OK;
}

/**************************************************************************
* 函数名称:      异或校验和
* 描述:        该函数用于对数据进行异或校验求和
* 参数:         u8 *pu8DataBuf          字符串的地址
			  s32 s32auhXorSize       字符的长度
*
* 返回值:       u8XorByte
*
***************************************************************************/
SANY_U8 Protocol_Cockpit::ProtoCockpit_CalcXOR(SANY_U8 *pu8DataBuf, SANY_S32 s32auhXorSize)
{
	SANY_U8 u8XorByte = 0;
	SANY_S32 s32Index = 0;
	/*调过通用协议头的两字节标识符 ##*/
	pu8DataBuf = pu8DataBuf + 2;
	u8XorByte = *pu8DataBuf;
	pu8DataBuf++;

	/*校验长度是总长度-3，包括两字节的通用协议标识符和最后一字节的校验位*/
	for (s32Index = 0; s32Index < s32auhXorSize - 3; s32Index++, pu8DataBuf++)
	{
		u8XorByte ^= *pu8DataBuf;
	}

	MINE_LOG(SANY_LOG_INFO, "Calc XOR = 0x%02X\n", u8XorByte);
	return u8XorByte;
}

/**************************************************************************
 * 函数名称:      获取本地时间
 * 描述:        该函数用于获取 本地时间
 * 参数:        u8 * pucTime
 *
 * 返回值:       RET_OK
 *
 ***************************************************************************/
SANY_U8 Protocol_Cockpit::ProtoCockpit_GetLocalTime(SANY_U8 *pucTime)
{
	time_t timep = time(0);
	auto p = localtime(&timep);
	SANY_U8 u8loctime[6] = {0};

	u8loctime[0] = (SANY_U8)(p->tm_year + 1900 - 2000);
	u8loctime[1] = (SANY_U8)(p->tm_mon + 1);
	u8loctime[2] = (SANY_U8)(p->tm_mday);
	u8loctime[3] = (SANY_U8)(p->tm_hour);
	u8loctime[4] = (SANY_U8)(p->tm_min);
	u8loctime[5] = (SANY_U8)(p->tm_sec);

	if (20 > u8loctime[0])
	{
		/* 时间异常 */
		MINE_LOG(SANY_LOG_ERROR, "Local time is error, %d\n", u8loctime[0]);
		u8loctime[0] = 20;
		u8loctime[1] = 12;
		u8loctime[2] = 7;
		u8loctime[3] = 16;
		u8loctime[4] = 34;
		u8loctime[5] = 45;
		memcpy(pucTime, u8loctime, 6);
		return RET_ERROR;
	}

	memcpy(pucTime, u8loctime, 6);

	return RET_OK;
}

/**************************************************************************
 * 函数名称:      获取车辆登入登入流水号
 * 描述:        用于获取登入流水号实际上就是一个序号
 * 参数:        u8* pu8SeqNum
 * 返回值:       RET_OK
 *
 ***************************************************************************/
SANY_U8 Protocol_Cockpit::ProtoCockpit_GetSequenceNum(SANY_U8 *pu8SeqNum)
{
	g_u8LoginSeqNum++;
	pu8SeqNum[0] = g_u8LoginSeqNum;

	return RET_OK;
}

/**************************************************************************
 * 函数名称:      获取ICCID 号
 * 描述:        用于获取4G SIM 卡的ICCID 唯一识别号
 * 参数:        u8* pu8Iccid
 *
 * 返回值:       RET_OK
 *
 ***************************************************************************/
SANY_U8 Protocol_Cockpit::ProtoCockpit_GetICCID(SANY_U8 *pu8Iccid)
{
	memcpy(pu8Iccid, vehicle_iccid.c_str(), 20);
	// MINE_LOG(SANY_LOG_INFO, "Device ICCID is %s\n", pu8Iccid);

	return RET_OK;
}

/********************************************************************
* 函数名称:  ProtoCockpit_ParsePacket
* 描述:        协议包校验，用于进行异或校验和的验证
			 包分发处理接口
*
* parma in:   u8 * pu8DataBuf  单个私有协议包
			  u32 u32DataLen  数据长度
*
* 返回值:       RET_OK/RET_ERROR
*
*********************************************************************/
SANY_U8 Protocol_Cockpit::ProtoCockpit_ParsePacket(SANY_U8 *pu8DataBuf, SANY_U32 u32DataLen)
{
	SANY_U8 u8Ret = RET_ERROR;
	SANY_U8 u8CheckSum = 0;
	CP_Header_st *headST = (CP_Header_st *)pu8DataBuf;
	SANY_U32 u32Index = 0;
	Protocol_Cockpit_Buffer_st CockpitDateBufferSt;
	SANY_U8(Protocol_Cockpit::*pFun)
	(Protocol_Cockpit_Buffer_st * pDateBuffer) = NULL;

	if (!pu8DataBuf || 0 >= u32DataLen)
	{
		MINE_LOG(SANY_LOG_ERROR, "%02X ", pu8DataBuf[u32Index]);
		return RET_ERROR;
	}

	/* 计算checksum */
	u8CheckSum = ProtoCockpit_CalcXOR(pu8DataBuf, u32DataLen - 1);
	if (u8CheckSum != pu8DataBuf[u32DataLen - 1])
	{
		/* checksum异常 */
		MINE_LOG(SANY_LOG_ERROR, "check sum error! u8CheckSum %d u32DataLen %d\n", u8CheckSum, u32DataLen);
		/* 回复校验失败 */
		ProtoCockpit_SendAckCom(headST->ucCommand[0], 0x2);
		return RET_ERROR;
	}

#if 0
	/*判断是否是已登录状态，如果未登录则不响应-------------------发布放开*/
	if(!g_u8LoginFlag)
	{
		printf("ProtoCockpit_ParseCmdPacket g_u8LoginFlag %d\n",g_u8LoginFlag);
		/* 回复失败报文 */
        ProtoCockpit_SendComACK(headST->ucCommand[0], 0x2);
        return RET_ERROR;
	}
#endif

	bzero(&CockpitDateBufferSt, sizeof(CockpitDateBufferSt));

	memcpy(CockpitDateBufferSt.szDataBuf, pu8DataBuf, PROTO_PACK_MAX_LEN);
	CockpitDateBufferSt.u32SendLen = u32DataLen;

	pFun = GetRecvProc_Cockpit(headST->ucCommand[0], headST->ucAckflg[0]);
	// MINE_LOG(SANY_LOG_INFO, "ucCommand : %d,  ucAckflg : %d\n", headST->ucCommand[0], headST->ucAckflg[0]);

	if (NULL != pFun)
	{
		u8Ret = (this->*pFun)(&CockpitDateBufferSt);
	}
	else
	{
		MINE_LOG(SANY_LOG_ERROR, "GetRecvProc_Cockpit err\n");
		/* 回复失败协议 */
		ProtoCockpit_SendAckCom(headST->ucCommand[0], 0x2);
		// 无法识别的协议
		m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3204, 3, 1);
		// 通用应答
		ProtoCockpit_SendAckCom(headST->ucCommand[0], 0x2);
		return RET_ERROR;
	}

	return u8Ret;
}

/********************************************************************
 * 函数名称:      ProtoCockpit_RecvAckLoginPacket 包数据处理
 * 描述:      解析处理舱端发送的登录响应协议
 * param in:        u8 * pDateBuffer  包数据缓冲区指针
 * 返回值:       RET_OK/RET_ERROR
 *
 *********************************************************************/
SANY_U8 Protocol_Cockpit::ProtoCockpit_RecvAckLoginPacket(Protocol_Cockpit_Buffer_st *pDateBuffer)
{
	CP_Header_st *headST = NULL;
	CP_Register_Ack_st *dataST = NULL;

	if (!pDateBuffer)
	{
		MINE_LOG(SANY_LOG_ERROR, "ProtoCockpit_RecvAckLoginPacket param err\n");
		return RET_ERROR;
	}

	headST = (CP_Header_st *)pDateBuffer->szDataBuf;
	dataST = (CP_Register_Ack_st *)headST->ucData;

//	MINE_LOG(SANY_LOG_INFO, "ProtoCockpit_RecvAckLoginPacket ucSequenceNum %02X ucLoginStatus %02X \n", dataST->ucSequenceNum[0], dataST->ucLoginStatus[0]);

	/*判断与登录流水号是否相同,不同则报错处理*/
	if (g_u8LoginSeqNum != (SANY_U8)dataST->ucSequenceNum[0])
	{		
		// 响应流水号与登入流水号不同
		//m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 8, 1);
		MINE_LOG(SANY_LOG_ERROR, "g_u8LoginSeqNum %d dataST->ucSequenceNum[0] %d \n", g_u8LoginSeqNum, (SANY_U8)dataST->ucSequenceNum[0]);
	//	return RET_ERROR;
	}

	/*登入状态全0,则登录完成，否则报错处理*/
	if (0 != dataST->ucLoginStatus[0])
	{
		MINE_LOG(SANY_LOG_ERROR, "dataST->ucLoginStatus %d \n", dataST->ucLoginStatus[0]);
		int loginstatus = dataST->ucLoginStatus[0];
		// ICCID不匹配
		if (loginstatus & 0x80)
		{
			MINE_LOG(SANY_LOG_ERROR, "LOGIN ICCID ERROR\n");
			g_u8LoginFlag = 2;
			m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 9, 1);
			return RET_ERROR;
		}
		// 硬件类型不匹配
		if (loginstatus & 0x40)
		{
			MINE_LOG(SANY_LOG_ERROR, "LOGIN HARD TYPE ERROR\n");
			g_u8LoginFlag = 2;
			m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 10, 1);
			return RET_ERROR;
		}
		// 硬件版本号不匹配
		if (loginstatus & 0x20)
		{
			MINE_LOG(SANY_LOG_ERROR, "LOGIN HARD VER ERROR\n");
			g_u8LoginFlag = 2;
			m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 11, 1);
			return RET_ERROR;
		}
		// 软件类型不匹配
		if (loginstatus & 0x10)
		{
			MINE_LOG(SANY_LOG_ERROR, "LOGIN SOFT TYPE ERROR\n");
			g_u8LoginFlag = 2;
			m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 12, 1);
			return RET_ERROR;
		}
		// 软件版本号不匹配
		if (loginstatus & 0x08)
		{
			MINE_LOG(SANY_LOG_ERROR, "LOGIN SOFT VER ERROR\n");
			g_u8LoginFlag = 2;
			m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 13, 1);
			return RET_ERROR;
		}
		// VIN不匹配
		if (loginstatus & 0x04){
			g_u8LoginFlag = 2;
			MINE_LOG(SANY_LOG_ERROR, "LOGIN VIN ERROR\n");
			return RET_ERROR;
		}
	//	return RET_ERROR;
	}
	/* 置位登录标志位 ，未登录状态下不处理命令协议*/
	g_u8LoginFlag = 1;
	/*clear failure already in FailureStatus*/
	m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 8, 0);
	m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 9, 0);
	m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 10, 0);
	m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 11, 0);
	m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 12, 0);
	m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 13, 0);

	/* 开启实时上报 */
	// g_u32RTDataFlg = 1;
	/* 开启状态变化上报 */
	// g_u32NLDataFlg = 1;
	/*初始化遥控控制数据*/
	ProtoCockpit_initCtrlData();
	MINE_LOG(SANY_LOG_INFO, "Cockpit login success,dataST->ucLoginStatus %d\n", (int)dataST->ucLoginStatus[0]);
	return RET_OK;
}

/********************************************************************
 * 函数名称:      ProtoCockpit_RecvCmdRepCtrl 包数据处理
 * 描述:      解析处理舱端发送的请求控制命令协议
 * param in:        u8 * pDateBuffer  包数据缓冲区指针
 * 返回值:       RET_OK/RET_ERROR
 *
 *********************************************************************/
/*目前响应的数据均为假数据，待与车控调试后修改*/
SANY_U8 Protocol_Cockpit::ProtoCockpit_RecvCmdRepCtrl(Protocol_Cockpit_Buffer_st *pDateBuffer)
{
	SANY_U8 u8Ret = RET_ERROR;
	SANY_U32 u32SendDataLen = 0;
	SANY_U8 szTmpBuffer[PROTO_PACK_MAX_LEN] = {0};	
	if (!pDateBuffer)
	{
		MINE_LOG(SANY_LOG_ERROR, "ProtoCockpit_RecvRepCtrlCmd param err\n");
		return RET_ERROR;
	}

	CP_Header_st *headST = (CP_Header_st *)pDateBuffer->szDataBuf;
	CP_Req_Ctrl_Cmd_st *pReqCtrlCmdDataST = (CP_Req_Ctrl_Cmd_st *)headST->ucData;


	/*标志位非法返回错误*/
	if (0x01 != pReqCtrlCmdDataST->ucCtrlSwitch[0] && 0x00 != pReqCtrlCmdDataST->ucCtrlSwitch[0] && 0x02 != pReqCtrlCmdDataST->ucCtrlSwitch[0])
	{
		MINE_LOG(SANY_LOG_ERROR, "EV_CP_REQ_CTRL ucCtrlSwitch %d err \n", pReqCtrlCmdDataST->ucCtrlSwitch[0]);
		return RET_ERROR;
	}
	/*控车状态置位*/
	if (0x01 == pReqCtrlCmdDataST->ucCtrlSwitch[0])
	{
		MINE_LOG(SANY_LOG_INFO, "-----EV_CP_REQ_CTRL Open-----\n");
		g_u8CtrlState = 1;
	}
	else if(0x02 == pReqCtrlCmdDataST->ucCtrlSwitch[0])
	{
		MINE_LOG(SANY_LOG_INFO, "-----EV_CP_REQ_CTRL Close, Auto_takeover-----\n");
		g_u8CtrlState = 2;
	}
	else if(0x03 == pReqCtrlCmdDataST->ucCtrlSwitch[0])
	{
		MINE_LOG(SANY_LOG_INFO, "-----EV_CP_REQ_CTRL Close, Manual_takeover-----\n");
		g_u8CtrlState = 3;
	}

	//printf("3-Manual, 1-Remote, 2-Auto, Currnet Control Status: %d\n", pReqCtrlCmdDataST->ucCtrlSwitch[0]);
	/*通用ACK组包接口*/
	u8Ret = ProtoCockpit_PackAckRepCtrl((SANY_U8 *)szTmpBuffer, &u32SendDataLen, pReqCtrlCmdDataST->ucSequenceNum[0]);
	if (RET_OK != u8Ret)
	{
		MINE_LOG(SANY_LOG_ERROR, "ProtoCockpit_PackRepCtrlACK err \n");
		return RET_ERROR;
	}
	/*返回接口*/
	u8Ret = ProtoCockpit_WriteSendBuffer(szTmpBuffer, u32SendDataLen);
	if (RET_OK != u8Ret)
	{
		MINE_LOG(SANY_LOG_ERROR, "ProtoCockpit_PackRepCtrlACK err \n");
		return RET_ERROR;
	}

	return RET_OK;
}

/********************************************************************
 * 函数名称:      ProtoCockpit_RecvCmdRealtimeCtrl 包数据处理
 * 描述:      解析处理舱端发送的实时控制命令协议
 * param in:        u8 * pDateBuffer  包数据缓冲区指针
 * 返回值:       RET_OK/RET_ERROR
 *
 *********************************************************************/
/*目前响应的数据均为假数据，待与车控调试后修改*/
SANY_U8 Protocol_Cockpit::ProtoCockpit_RecvCmdRealtimeCtrl(Protocol_Cockpit_Buffer_st *pDateBuffer)
{
//	u8RealtimeCtrl = 1;
	SANY_U8 u8Ret = RET_ERROR;
	SANY_U32 u32SendDataLen = 0;
	SANY_U8 szTmpBuffer[PROTO_PACK_MAX_LEN] = {0};
	if (!pDateBuffer)
	{
		MINE_LOG(SANY_LOG_ERROR, "ProtoCockpit_RecvRepCtrlCmd param err\n");
		return RET_ERROR;
	}

	CP_Header_st *headST = (CP_Header_st *)pDateBuffer->szDataBuf;
	CP_Realtime_Ctrl_Cmd_st *pRealtimeCtrlCmdDataST = (CP_Realtime_Ctrl_Cmd_st *)headST->ucData;

	if (g_u8CtrlState != 1)
	{
		MINE_LOG(SANY_LOG_ERROR, "----EV_CP_REQ_CTRL ERR----\n");
		return RET_ERROR;
	}

	struWireCtrlIn.iWheelAngle = NET2SYS(pRealtimeCtrlCmdDataST->ucWheelAngle);		/*车轮转角*/
	struWireCtrlIn.iThrottleOpening = pRealtimeCtrlCmdDataST->ucThrottleOpening[0]; /*油门开度*/
	struWireCtrlIn.iBrakeOpening = pRealtimeCtrlCmdDataST->ucBrakeOpening[0];		/*制动开度*/
	struWireCtrlIn.iGearOpening = pRealtimeCtrlCmdDataST->ucGearOpening[0];			/*档位操作*/
	struWireCtrlIn.iCombinedGearL = pRealtimeCtrlCmdDataST->ucCombinedGearL[0];		/*组合开关左*/
	struWireCtrlIn.iCombinedGearR = pRealtimeCtrlCmdDataST->ucCombinedGearR[0];		/*组合开关右*/
	struWireCtrlIn.iKeyOperation1 = pRealtimeCtrlCmdDataST->ucKeyOperation1[0];		/*按键操作1*/
	struWireCtrlIn.iKeyOperation2 = pRealtimeCtrlCmdDataST->ucKeyOperation2[0];		/*按键操作2*/

	struWireCtrlIn.iSafeRelease= pRealtimeCtrlCmdDataST->ucSafeRelease[0];				/*安全措施-故障清除*/
	struWireCtrlIn.iSafeProtect = pRealtimeCtrlCmdDataST->UCSafeProtect[0];				/*安全措施-进入安全模式*/

	MINE_LOG(SANY_LOG_INFO, "-----EV_CP_REALTIME_CTRL start----\n");
	MINE_LOG(SANY_LOG_INFO, "ucTime %8s\n", pRealtimeCtrlCmdDataST->ucTime);
	MINE_LOG(SANY_LOG_INFO, "ucSequenceNum %d\n", pRealtimeCtrlCmdDataST->ucSequenceNum[0]);
	MINE_LOG(SANY_LOG_INFO, "iBrakeOpening %d\n", struWireCtrlIn.iBrakeOpening);
	MINE_LOG(SANY_LOG_INFO, "iCombinedGearL %d\n", struWireCtrlIn.iCombinedGearL);
	MINE_LOG(SANY_LOG_INFO, "iCombinedGearR %d\n", struWireCtrlIn.iCombinedGearR);
	MINE_LOG(SANY_LOG_INFO, "iGearOpening %d\n", struWireCtrlIn.iGearOpening);
	MINE_LOG(SANY_LOG_INFO, "iKeyOperation1 %d\n", struWireCtrlIn.iKeyOperation1);
	MINE_LOG(SANY_LOG_INFO, "iKeyOperation2 %d\n", struWireCtrlIn.iKeyOperation2);
	MINE_LOG(SANY_LOG_INFO, "iThrottleOpening %d\n", struWireCtrlIn.iThrottleOpening);
	MINE_LOG(SANY_LOG_INFO, "iWheelAngle %2d\n", struWireCtrlIn.iWheelAngle);
	MINE_LOG(SANY_LOG_INFO, "iSafeRelease%d\n", struWireCtrlIn.iSafeRelease);
	MINE_LOG(SANY_LOG_INFO, "iSafeProtect%d\n", struWireCtrlIn.iSafeProtect);
	MINE_LOG(SANY_LOG_INFO, "-----EV_CP_REALTIME_CTRL end----\n");

	return RET_OK;
}

/********************************************************************
 * 函数名称:      ProtoCockpit_RecvCmdEmergencyStop 包数据处理
 * 描述:      解析处理舱端发送的紧急刹车命令协议
 * param in:        u8 * pDateBuffer  包数据缓冲区指针
 * 返回值:       RET_OK/RET_ERROR
 *
 *********************************************************************/
/*目前响应的数据均为假数据，待与车控调试后修改*/
SANY_U8 Protocol_Cockpit::ProtoCockpit_RecvCmdEmergencyStop(Protocol_Cockpit_Buffer_st *pDateBuffer)
{
	SANY_U8 u8Ret = RET_ERROR;
	SANY_U32 u32SendDataLen = 0;
	SANY_U8 szTmpBuffer[PROTO_PACK_MAX_LEN] = {0};

	if (!pDateBuffer)
	{
		MINE_LOG(SANY_LOG_ERROR, "ProtoCockpit_RecvEmergencyStopCmd param err\n");
		return RET_ERROR;
	}

	CP_Header_st *headST = (CP_Header_st *)pDateBuffer->szDataBuf;
	CP_Emergency_Stop_Cmd_st *pEmergencyStopCmdDataSt = (CP_Emergency_Stop_Cmd_st *)headST->ucData;

	MINE_LOG(SANY_LOG_INFO, "EV_CP_SEND_EMERGENCY_STOP flag %d\n", pEmergencyStopCmdDataSt->ucStopFlag[0]);
	/*判断标志位合法性，发送控制命令到底层*/
	if (0x01 == pEmergencyStopCmdDataSt->ucStopFlag[0])
	{
		g_u8EmergencyStopFlg = 1; /*请求急停*/
	}
	if (0x00 == pEmergencyStopCmdDataSt->ucStopFlag[0])
	{
		g_u8EmergencyStopFlg = 0; /*解除急停*/
	}

	/*通用ACK组包接口*/
	u8Ret = ProtoCockpit_PackAckEmergencyStop((SANY_U8 *)szTmpBuffer, &u32SendDataLen, pEmergencyStopCmdDataSt->ucSequenceNum[0]);
	if (RET_OK != u8Ret)
	{
		MINE_LOG(SANY_LOG_ERROR, "ProtoCockpit_PackEmergencyStopACK err \n");
		return RET_ERROR;
	}

	/*返回接口*/
	u8Ret = ProtoCockpit_WriteSendBuffer(szTmpBuffer, u32SendDataLen);
	if (RET_OK != u8Ret)
	{
		MINE_LOG(SANY_LOG_ERROR, "ProtoCockpit_PackEmergencyStopACK err \n");
		return RET_ERROR;
	}

	return RET_OK;
}

/********************************************************************
 * 函数名称:      ProtoCockpit_RecvCmdHeartBeat包数据处理
 * 描述:      解析处理舱端发送的心跳命令协议
 * param in:        u8 * pDateBuffer  包数据缓冲区指针
 * 返回值:       RET_OK/RET_ERROR
 *
 *********************************************************************/
SANY_U8 Protocol_Cockpit::ProtoCockpit_RecvCmdHeartBeat(Protocol_Cockpit_Buffer_st *pDateBuffer)
{
	SANY_U8 u8Ret = RET_ERROR;
	SANY_U32 u32SendDataLen = 0;
	SANY_U16 u16SequenceNum = 0;
	SANY_U8 szTmpBuffer[PROTO_PACK_MAX_LEN] = {0};

	if (!pDateBuffer)
	{
		MINE_LOG(SANY_LOG_ERROR, "ProtoCockpit_RecvHeartBeatCmd param err\n");
		return RET_ERROR;
	}

	CP_Header_st *headST = (CP_Header_st *)pDateBuffer->szDataBuf;
	CP_HeartBeat_Cmd_st *pHeartBeatCmdDataSt = (CP_HeartBeat_Cmd_st *)headST->ucData;

	u16SequenceNum = NET2SYS(pHeartBeatCmdDataSt->ucSequenceNum);
	// MINE_LOG(SANY_LOG_ERROR, "u16SequenceNum : %d\n", u16SequenceNum);
	/*通用ACK组包接口*/
	u8Ret = ProtoCockpit_PackAckHeartBeat((SANY_U8 *)szTmpBuffer, &u32SendDataLen, u16SequenceNum);
	if (RET_OK != u8Ret)
	{
		MINE_LOG(SANY_LOG_ERROR, "ProtoCockpit_PackHeartBeatACK err \n");
		return RET_ERROR;
	}

	/*返回接口*/
	u8Ret = ProtoCockpit_WriteSendBuffer(szTmpBuffer, u32SendDataLen);
	if (RET_OK != u8Ret)
	{
		MINE_LOG(SANY_LOG_ERROR, "ProtoCockpit_PackHeartBeatACK err \n");
		return RET_ERROR;
	}

	return RET_OK;
}

/********************************************************************
 * 函数名称:      ProtoCockpit_RecvCmdLoginOut 包数据处理
 * 描述:      解析处理舱端发送的登出命令协议
 * param in:        u8 * pDateBuffer  包数据缓冲区指针
 * 返回值:       RET_OK/RET_ERROR
 *
 *********************************************************************/
/*目前响应的数据均为假数据，待与车控调试后修改*/
SANY_U8 Protocol_Cockpit::ProtoCockpit_RecvCmdLoginOut(Protocol_Cockpit_Buffer_st *pDateBuffer)
{
	SANY_U8 u8Ret = RET_ERROR;
	SANY_U32 u32SendDataLen = 0;
	SANY_U8 szTmpBuffer[PROTO_PACK_MAX_LEN] = {0};
	if (!pDateBuffer)
	{
		MINE_LOG(SANY_LOG_ERROR, "ProtoCockpit_RecvRepCtrlCmd param err\n");
		return RET_ERROR;
	}

	CP_Header_st *headST = (CP_Header_st *)pDateBuffer->szDataBuf;
	CP_LoginOut_Cmd_st *pLoginOutCmdDataSt = (CP_LoginOut_Cmd_st *)headST->ucData;

	MINE_LOG(SANY_LOG_INFO, "EV_CP_VEHICLE_SIGNOUT ucloginOutFlag %d\n", pLoginOutCmdDataSt->ucloginOutFlag[0]);
	/*判断标志位合法性，判断流水号合法性*/
	if (0x02 != pLoginOutCmdDataSt->ucloginOutFlag[0])
	{
		m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 15, 1);
		MINE_LOG(SANY_LOG_ERROR, "ucloginOutFlag err\n");
	//	return RET_ERROR;
	}
	if (g_u8LoginSeqNum != pLoginOutCmdDataSt->ucSequenceNum[0])
	{
		m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 16, 1);
		MINE_LOG(SANY_LOG_ERROR, "ucloginOutNum err\n");
	}

	/*通用ACK组包接口*/
	u8Ret = ProtoCockpit_PackAckLoginOut((SANY_U8 *)szTmpBuffer, &u32SendDataLen);
	if (RET_OK != u8Ret)
	{
		MINE_LOG(SANY_LOG_ERROR, "ProtoCockpit_PackRepCtrlACK err \n");
	//	return RET_ERROR;
	}

	/*返回接口*/
	u8Ret = ProtoCockpit_WriteSendBuffer(szTmpBuffer, u32SendDataLen);
	if (RET_OK != u8Ret)
	{
		MINE_LOG(SANY_LOG_ERROR, "ProtoCockpit_PackRepCtrlACK err \n");
	//	return RET_ERROR;
	}

	// g_u8IpFlag = 0;
	/*修改登出标志位*/
	g_u8LoginFlag = 2;
	/*关闭Socket，释放线程资源*/
	Task_stop();

	return RET_OK;
}

/********************************************************************
* 函数名称:      ProtoCockpit_PackAckRepCtrl
* 描述:        组装请求控制响应报文
* parma in:         u8*  pDataBuf    包缓冲区
					u8   ucSequenceNum	响应流水号
* parma out:		u8*  pOutDataLen  协议包长
* 返回值:       RET_OK/RET_ERROR
*********************************************************************/
SANY_U8 Protocol_Cockpit::ProtoCockpit_PackAckRepCtrl(SANY_U8 *pDataBuf, SANY_U32 *pOutDataLen, SANY_U8 ucSequenceNum)
{

	SANY_U8 u8Ret = RET_OK;
	SANY_U8 u8DataBuf[PROTO_PACK_MAX_LEN] = {0};
	SANY_S32 s32DataLen = 0;

	if (!pDataBuf || !pOutDataLen || ucSequenceNum < 1)
	{
		MINE_LOG(SANY_LOG_ERROR, "ProtoCockpit_PackRealtimeCtrlACK param err\n");
		return RET_ERROR;
	}
	CP_Header_st *pHeadST = (CP_Header_st *)u8DataBuf;
	CP_Req_Ctrl_Ack_st *pDataST = (CP_Req_Ctrl_Ack_st *)pHeadST->ucData;

	/* 填充消息内容 */
	u8Ret = ProtoCockpit_GetLocalTime(pDataST->ucTime);
	if (RET_OK != u8Ret)
	{
		MINE_LOG(SANY_LOG_ERROR, "Get Info Error!\n");
	}
	pDataST->ucSequenceNum[0] = ucSequenceNum;
	pDataST->ucCtrlStatus[0] = g_u8CtrlState;

	/* 计算消息长度 */
	SYS2NET(pHeadST->ucDataByteSize, (sizeof(CP_Req_Ctrl_Ack_st)));

	/* 创建消息头 */
	ProtoCockpit_PackHeader(EV_CP_REQ_CTRL, EV_WP_ACK, pHeadST);

	s32DataLen = sizeof(CP_Req_Ctrl_Ack_st);
	/* 计算Check Sum */
	pHeadST->ucData[s32DataLen] = ProtoCockpit_CalcXOR((SANY_U8 *)pHeadST, s32DataLen + sizeof(CP_Header_st));
	s32DataLen += sizeof(CP_Header_st) + 1;

	MINE_LOG(SANY_LOG_INFO, "ProtoCockpit_PackRepCtrlACK\n");

	memcpy(pDataBuf, u8DataBuf, s32DataLen);
	*pOutDataLen = s32DataLen;
	u8Ret = ProtoCockpit_WriteSendBuffer(u8DataBuf, s32DataLen);
	
	return u8Ret;
}

/********************************************************************
* 函数名称:      ProtoCockpit_PackAckEmergencyStop
* 描述:        组装紧急刹车响应报文
* parma in:         u8*  pDataBuf    包缓冲区
					u8   ucSequenceNum	响应流水号
* parma out:		u8*  pOutDataLen  协议包长
* 返回值:       RET_OK/RET_ERROR
*********************************************************************/
SANY_U8 Protocol_Cockpit::ProtoCockpit_PackAckEmergencyStop(SANY_U8 *pDataBuf, SANY_U32 *pOutDataLen, SANY_U8 ucSequenceNum)
{
	SANY_U8 u8Ret = RET_OK;
	SANY_U8 u8DataBuf[PROTO_PACK_MAX_LEN] = {0};
	SANY_S32 s32DataLen = 0;
	if (!pDataBuf || !pOutDataLen || ucSequenceNum < 1)
	{
		MINE_LOG(SANY_LOG_ERROR, "ProtoCockpit_PackEmergencyStopACK param err\n");
		return RET_ERROR;
	}
	CP_Header_st *pHeadST = (CP_Header_st *)u8DataBuf;
	CP_Emergency_Stop_Ack_st *pDataST = (CP_Emergency_Stop_Ack_st *)pHeadST->ucData;

	/* 填充消息内容 */
	u8Ret = ProtoCockpit_GetLocalTime(pDataST->ucTime);
	if (RET_OK != u8Ret)
	{
		MINE_LOG(SANY_LOG_ERROR, "Get Local-time Error!\n");
	}
	pDataST->ucSequenceNum[0] = ucSequenceNum;

	/* 计算消息长度 */
	SYS2NET(pHeadST->ucDataByteSize, (sizeof(CP_Emergency_Stop_Ack_st)));

	/* 创建消息头 */
	ProtoCockpit_PackHeader(EV_CP_SEND_EMERGENCY_STOP, EV_WP_ACK, pHeadST);

	s32DataLen = sizeof(CP_Emergency_Stop_Ack_st);
	/* 计算Check Sum */
	pHeadST->ucData[s32DataLen] = ProtoCockpit_CalcXOR((SANY_U8 *)pHeadST, s32DataLen + sizeof(CP_Header_st));
	s32DataLen += sizeof(CP_Header_st) + 1;

	MINE_LOG(SANY_LOG_INFO, "ProtoCockpit_PackEmergencyStopACK\n");

	memcpy(pDataBuf, u8DataBuf, s32DataLen);
	*pOutDataLen = s32DataLen;

	u8Ret = ProtoCockpit_WriteSendBuffer(u8DataBuf, s32DataLen);

	return u8Ret;
}

/********************************************************************
* 函数名称:      ProtoCockpit_PackAckHeartBeatStop
* 描述:        组装心跳响应报文
* parma in:         u8*  pDataBuf    包缓冲区
					u16   ucSequenceNum	响应流水号
* parma out:		u8*  pOutDataLen  协议包长
* 返回值:       RET_OK/RET_ERROR
*********************************************************************/
SANY_U8 Protocol_Cockpit::ProtoCockpit_PackAckHeartBeat(SANY_U8 *pDataBuf, SANY_U32 *pOutDataLen, SANY_U16 ucSequenceNum)
{
	SANY_U8 u8Ret = RET_OK;
	SANY_U8 u8DataBuf[PROTO_PACK_MAX_LEN] = {0};
	SANY_S32 s32DataLen = 0;
	if (!pDataBuf || !pOutDataLen || ucSequenceNum < 0)
	{
		MINE_LOG(SANY_LOG_ERROR, "ProtoCockpit_PackHeartBeatACK param err\n");
		return RET_ERROR;
	}
	CP_Header_st *pHeadST = (CP_Header_st *)u8DataBuf;
	CP_HeartBeat_Ack_st *pDataST = (CP_HeartBeat_Ack_st *)pHeadST->ucData;

	/* 填充消息内容 */
	u8Ret = ProtoCockpit_GetLocalTime(pDataST->ucTime);
	if (RET_OK != u8Ret)
	{
		MINE_LOG(SANY_LOG_ERROR, "Get Local-time Error!\n");
	}
	/*心跳流水号*/
	SYS2NET(pDataST->ucSequenceNum, ucSequenceNum);
	/* 计算消息长度 */
	SYS2NET(pHeadST->ucDataByteSize, (sizeof(CP_HeartBeat_Ack_st)));

	/* 创建消息头 */
	ProtoCockpit_PackHeader(EV_CP_HEART_BEAT, EV_WP_ACK, pHeadST);

	s32DataLen = sizeof(CP_HeartBeat_Ack_st);
	/* 计算Check Sum */
	pHeadST->ucData[s32DataLen] = ProtoCockpit_CalcXOR((SANY_U8 *)pHeadST, s32DataLen + sizeof(CP_Header_st));
	s32DataLen += sizeof(CP_Header_st) + 1;

	// MINE_LOG(SANY_LOG_INFO, "ProtoCockpit_PackHeartBeatACK\n");

	memcpy(pDataBuf, u8DataBuf, s32DataLen);
	*pOutDataLen = s32DataLen;

	u8Ret = ProtoCockpit_WriteSendBuffer(u8DataBuf, s32DataLen);

	return u8Ret;
}

/********************************************************************
 * 函数名称:      ProtoCockpit_PackAckLoginOut
 * 描述:        组装被动登出响应报文
 * parma in:         u8*  pDataBuf    包缓冲区
 * parma out:		u8*  pOutDataLen  协议包长
 * 返回值:       RET_OK/RET_ERROR
 *********************************************************************/
SANY_U8 Protocol_Cockpit::ProtoCockpit_PackAckLoginOut(SANY_U8 *pDataBuf, SANY_U32 *pOutDataLen)
{
	SANY_U8 u8Ret = RET_OK;
	SANY_U8 u8DataBuf[PROTO_PACK_MAX_LEN] = {0};
	SANY_S32 s32DataLen = 0;
	if (!pDataBuf || !pOutDataLen)
	{
		MINE_LOG(SANY_LOG_ERROR, "ProtoCockpit_PackEmergencyStopACK param err\n");
		return RET_ERROR;
	}
	CP_Header_st *pHeadST = (CP_Header_st *)u8DataBuf;
	CP_LoginOut_Ack_st *pDataST = (CP_LoginOut_Ack_st *)pHeadST->ucData;

	/* 填充消息内容 */
	u8Ret = ProtoCockpit_GetLocalTime(pDataST->ucTime);
	if (RET_ERROR == u8Ret)
	{
		MINE_LOG(SANY_LOG_ERROR, "Get Info Error!\n");
		return RET_ERROR;
	}

	/* 计算消息长度 */
	SYS2NET(pHeadST->ucDataByteSize, (sizeof(CP_LoginOut_Ack_st)));

	/* 创建消息头 */
	ProtoCockpit_PackHeader(EV_CP_VEHICLE_SIGNOUT, EV_WP_ACK, pHeadST);

	s32DataLen = sizeof(CP_LoginOut_Ack_st);
	/* 计算Check Sum */
	pHeadST->ucData[s32DataLen] = ProtoCockpit_CalcXOR((SANY_U8 *)pHeadST, s32DataLen + sizeof(CP_Header_st));
	s32DataLen += sizeof(CP_Header_st) + 1;

	MINE_LOG(SANY_LOG_INFO, "ProtoCockpit_PackLoginOutACK!\n");

	memcpy(pDataBuf, u8DataBuf, s32DataLen);
	*pOutDataLen = s32DataLen;
	u8Ret = ProtoCockpit_WriteSendBuffer(u8DataBuf, s32DataLen);

	return u8Ret;
}

/********************************************************************
* 函数名称:      ProtoCockpit_SendAckCom 通用应答
* 描述:        对接收的有问题的命令的数据，进行通用应答，
			 ucAckflg   ：1表示接收成功 ，2 表示接收错误，
* param in:     u8*  ucCommand
* param in		u8*  ucAckflg
* 返回值:      s32DataLen：   封装的数据长度
*
*********************************************************************/
SANY_U8 Protocol_Cockpit::ProtoCockpit_SendAckCom(SANY_U8 ucCmd, SANY_U8 ucAckflg)
{
	SANY_U8 szDataBuf[50] = {0};
	SANY_S32 s32DataLen = 0;
	CP_Header_st *pHeadST = (CP_Header_st *)szDataBuf;
	WP_ProAck_st *pDataST = (WP_ProAck_st *)pHeadST->ucData;
	ProtoCockpit_GetLocalTime(pDataST->ucTime);
	SYS2NET(pHeadST->ucDataByteSize, (sizeof(WP_ProAck_st)));
	ProtoCockpit_PackHeader(ucCmd, ucAckflg, pHeadST);
	s32DataLen = sizeof(WP_ProAck_st);
	pHeadST->ucData[s32DataLen] = ProtoCockpit_CalcXOR((SANY_U8 *)pHeadST, s32DataLen + sizeof(CP_Header_st));
	s32DataLen += sizeof(CP_Header_st) + 1;

	MINE_LOG(SANY_LOG_INFO, "Send General ACK\n");

	s32DataLen = ProtoCockpit_WriteSendBuffer(szDataBuf, s32DataLen);

	return s32DataLen;
}

/********************************************************************
 * 函数名称:      ProtoCockpit_SendCmdLogin
 * 描述:        组装并发送  登录驾驶舱私有协议报文
 * param :
 * 返回值:       RET_OK/RET_ERROR
 *********************************************************************/
SANY_U8 Protocol_Cockpit::ProtoCockpit_SendCmdLogin()
{
	SANY_U8 u8Ret = RET_OK;
	SANY_U8 u8Ret_time = RET_OK;
	SANY_U8 u8Ret_ICCID = RET_OK;
	SANY_U8 u8DataBuf[PROTO_PACK_MAX_LEN] = {0};
	SANY_U32 u32DataLen = 0;
	SANY_U8 g_current_state = 0;
	CP_Header_st *headST = (CP_Header_st *)u8DataBuf;
	CP_Register_Cmd_st *dataST = (CP_Register_Cmd_st *)headST->ucData;

	/* 填充消息内容 */
	u8Ret_time = ProtoCockpit_GetLocalTime(dataST->ucTime);
	u8Ret &= ProtoCockpit_GetSequenceNum(dataST->ucSequenceNum);
	u8Ret_ICCID &= ProtoCockpit_GetICCID(dataST->ucIccid);

	if (RET_ERROR == u8Ret)
	{
		MINE_LOG(SANY_LOG_ERROR, "Get SequenceNum Error!\n");
		return RET_ERROR;
	}

	memcpy(dataST->ucHardVer, vehicle_HardVer.c_str(), 4);
	memcpy(dataST->ucSoftVer, vehicle_SoftVer.c_str(), 4);
	//车辆驾驶状态 
	if((WireInfo.vcu_state() == 5) && (WireInfo.Remote_or_Unman() == 8))
	{
		g_current_state = 2;			//remote
	}
	else if((WireInfo.vcu_state() == 5) && (WireInfo.Remote_or_Unman() == 6))
	{
		g_current_state = 3;			//unman
	}
	else
	{
		g_current_state = 1;			//manual
	}
	dataST->ucWorkStatus[0] = g_current_state;
	// g_old_current_state = g_current_state;
	printf("login work status, vcu_state: %d,  Remote_or_Unman: %d\n", WireInfo.vcu_state(), WireInfo.Remote_or_Unman());
	dataST->ucCurrentGear[0] = WireInfo.trans_current_gear();
	// printf("login gear: %d\n", dataST->ucCurrentGear[0]);
	/* 计算消息长度 */
	SYS2NET(headST->ucDataByteSize, (sizeof(CP_Register_Cmd_st)));

	/* 创建消息头 */
	ProtoCockpit_PackHeader(EV_CP_VEHICLE_REGISTER, EV_WP_CMD, headST);
	/* 第一次登录打印车辆VIN */
	SANY_U8 u8Temp[WP_VIN_SIZE + 1] = {0};
	memcpy(u8Temp, headST->ucVIN, WP_VIN_SIZE);

	u32DataLen = sizeof(CP_Register_Cmd_st);
	/* 计算Check Sum */
	headST->ucData[u32DataLen] = ProtoCockpit_CalcXOR((SANY_U8 *)headST, u32DataLen + sizeof(CP_Header_st));
	u32DataLen += sizeof(CP_Header_st) + 1;

	MINE_LOG(SANY_LOG_INFO, "vcu status: %d, remote_or_unman: %d\n", WireInfo.vcu_state(), WireInfo.Remote_or_Unman());
	MINE_LOG(SANY_LOG_INFO, "Begin to login\n");
	MINE_LOG(SANY_LOG_INFO, "current work status: %d\n", dataST->ucWorkStatus[0]);
	MINE_LOG(SANY_LOG_INFO, "current gear: %d\n", dataST->ucCurrentGear[0]);	

	u8Ret = ProtoCockpit_WriteSendBuffer(u8DataBuf, u32DataLen);

	return u8Ret;
}

/********************************************************************
 * 函数名称:      ProtoCockpit_SendCmdRealtimeRep
 * 描述:        组装并发送实时上报（高频）报文组装
 * param :
 * 返回值:  RET_OK/RET_ERROR
 *********************************************************************/
SANY_U8 Protocol_Cockpit::ProtoCockpit_SendCmdRealtimeRep()
{
	SANY_U8 u8Ret = RET_OK;
	SANY_U8 u8Ret_Data = RET_OK;
	SANY_U8 u8DataBuf[PROTO_PACK_MAX_LEN] = {0};
	SANY_U32 u32DataLen = 0;
	SANY_U16 u16TmpDate = 0;
	SANY_U32 u32TmpDate = 0;

	CP_Header_st *pHeadST = (CP_Header_st *)u8DataBuf;
	CP_Realtime_Rep_Cmd_st *pDataST = (CP_Realtime_Rep_Cmd_st *)pHeadST->ucData;

	/* 填充消息内容 */
	u8Ret = ProtoCockpit_GetLocalTime(pDataST->ucTime);

	u8Ret_Data = ProtoCockpit_getFeedbackData();
	/*车速*/
	u16TmpDate = (int)(struWireCtrlOut.iVehicle_speed * 10);					
	SYS2NET(pDataST->ucCarSpeed, u16TmpDate);
	/*航向角*/
	u16TmpDate = (int)(struWireCtrlOut.iHeadingAngle * 10);
	SYS2NET(pDataST->ucHeadingAngle, u16TmpDate);
	/*车轮转角*/
	u16TmpDate = (int)((struWireCtrlOut.iWheelAngleL + 40) * 10);
	SYS2NET(pDataST->ucWheelAngleL, u16TmpDate);
	/*油门*/
	pDataST->ucThrottleOpening[0] = (SANY_U8)struWireCtrlOut.iThrottleOpening;
	/*制动*/
	pDataST->ucBrakeOpening[0] = (SANY_U8)struWireCtrlOut.iBrakeOpening;
	/*发动机转速, 20250306 22软件版本 区分油车、电车、混型*/
	u16TmpDate = (int)struWireCtrlOut.iEngineSpeed;
	SYS2NET(pDataST->ucEngineSpeed, u16TmpDate);
	/*累计里程*/
	u32TmpDate = (int)(struWireCtrlOut.iAccumulatedMileage * 10);
	*(SANY_U32 *)&(pDataST->ucAccumulatedMileage) = SYS2NET4((SANY_U8 *)&u32TmpDate);
	/*小计里程*/
	u16TmpDate = (int)(struWireCtrlOut.iSubtotalMileage * 10);
	SYS2NET(pDataST->ucSubtotalMileage, u16TmpDate);
	/*俯仰角*/
	u16TmpDate = (int)(struWireCtrlOut.iPitchAngle + 90);
	SYS2NET(pDataST->ucPitchAngle, u16TmpDate);
	/*档位*/
	pDataST->ucGearOper[0] = (SANY_U8)struWireCtrlOut.iGearOpening;
	/*剩余油量*/
	pDataST->ucRemainingOil[0] = (SANY_U8)struWireCtrlOut.iRemainingOil;
	/*水温*/
	/* 20250306 调整水温数值，实际范围为-50 ～150 ，传递正数 0～200，舱端同步做调整 电车无水温----- start*/
	pDataST->ucWaterTemp[0] = (SANY_U8)(struWireCtrlOut.iWaterTemp + 50);
	/* 20250306 调整水温数值，实际范围为-50 ～150 ，传递正数 0～200，舱端同步做调整 ----- end*/
	/*气压1\2 2.3.2  软件版本号为21 调整数值范围*/
	pDataST->ucBrakeAirPre1[0] = (SANY_U8)(struWireCtrlOut.iBrakeAirPre1 * 100);
	pDataST->ucBrakeAirPre2[0] = (SANY_U8)(struWireCtrlOut.iBrakeAirPre2 * 100);
	/*组合开关左右*/
	pDataST->ucCombinedGearL[0] = (SANY_U8)struWireCtrlOut.iCombinedGearL;
	pDataST->ucCombinedGearR[0] = (SANY_U8)struWireCtrlOut.iCombinedGearR;
	/*按键操作1\2*/
	pDataST->ucKeyOperation1[0] = (SANY_U8)struWireCtrlOut.iKeyOperation1;
	pDataST->ucKeyOperation2[0] = (SANY_U8)struWireCtrlOut.iKeyOperation2;
	/*车辆驾驶状态*/
	pDataST->ucWorkStatus[0] = (SANY_U8)struWireCtrlOut.iWorkStatus;
		/*举升到位状态*/
	pDataST->ucLiftUpStatus[0] = (SANY_U8)struWireCtrlOut.iLiftUpStatus;
	/*下降到位状态*/
	pDataST->ucLiftDownStatus[0] = (SANY_U8)struWireCtrlOut.iLiftDownStatus;
	/*safe模块-故障信息*/
	pDataST->ucSafeInfo1[0] = (SANY_U8)struWireCtrlOut.iSafeInfo1;
	pDataST->ucSafeInfo2[0] = (SANY_U8)struWireCtrlOut.iSafeInfo2;

	MINE_LOG(SANY_LOG_INFO, "--- ProtoCockpit_SendCmdRealtimeRep --- start\n");
	MINE_LOG(SANY_LOG_INFO, "iVehicle_speed %d\n", (int)(struWireCtrlOut.iVehicle_speed * 10));
	MINE_LOG(SANY_LOG_INFO, "iBrakeOpening %d\n", (int)struWireCtrlOut.iBrakeOpening);
	MINE_LOG(SANY_LOG_INFO, "iWheelAngleL %d\n", (int)((struWireCtrlOut.iWheelAngleL + 40) * 10));
	MINE_LOG(SANY_LOG_INFO, "iThrottleOpening %d\n", (int)(struWireCtrlOut.iThrottleOpening));
	MINE_LOG(SANY_LOG_INFO, "iEngineSpeed %d\n", (int)(struWireCtrlOut.iEngineSpeed));				//
	MINE_LOG(SANY_LOG_INFO, "iHeadingAngle %d\n", (int)(struWireCtrlOut.iHeadingAngle * 10));
	MINE_LOG(SANY_LOG_INFO, "iAccumulatedMileage %u\n", (SANY_U32)(struWireCtrlOut.iAccumulatedMileage * 10));
	MINE_LOG(SANY_LOG_INFO, "iSubtotalMileage %d\n", (int)(struWireCtrlOut.iSubtotalMileage * 10));
	MINE_LOG(SANY_LOG_INFO, "iPitchAngle %d\n", (int)(struWireCtrlOut.iPitchAngle + 90));
	MINE_LOG(SANY_LOG_INFO, "iGearOpening %d\n", (int)(struWireCtrlOut.iGearOpening));					//
	MINE_LOG(SANY_LOG_INFO, "iRemainingOil %d\n", (int)(struWireCtrlOut.iRemainingOil));				//
	MINE_LOG(SANY_LOG_INFO, "iWaterTemp %d\n", (int)(struWireCtrlOut.iWaterTemp));					//
	MINE_LOG(SANY_LOG_INFO, "iBrakeAirPre1 %d\n", (int)(struWireCtrlOut.iBrakeAirPre1 * 100));				//    
	MINE_LOG(SANY_LOG_INFO, "iBrakeAirPre2 %d\n", (int)(struWireCtrlOut.iBrakeAirPre2 * 100));
	MINE_LOG(SANY_LOG_INFO, "iCombinedGearL %d\n", (int)(struWireCtrlOut.iCombinedGearL));
	MINE_LOG(SANY_LOG_INFO, "iCombinedGearR %d\n", (int)(struWireCtrlOut.iCombinedGearR));
	MINE_LOG(SANY_LOG_INFO, "iKeyOperation1 %d\n", (int)(struWireCtrlOut.iKeyOperation1));
	MINE_LOG(SANY_LOG_INFO, "iKeyOperation2 %d\n", (int)(struWireCtrlOut.iKeyOperation2));
	MINE_LOG(SANY_LOG_INFO, "iWorkStatus %d\n", (int)(struWireCtrlOut.iWorkStatus));
	MINE_LOG(SANY_LOG_INFO, "iLiftUpStatus %d\n", (int)(struWireCtrlOut.iLiftUpStatus));
	MINE_LOG(SANY_LOG_INFO, "iLiftDownStatus %d\n", (int)(struWireCtrlOut.iLiftDownStatus));
	MINE_LOG(SANY_LOG_INFO, "iSafeInfo %d %d %d %d\n", (int)(struWireCtrlOut.iSafeInfo1), (int)(struWireCtrlOut.iSafeInfo2), (int)pDataST->ucSafeInfo1[0], (int)pDataST->ucSafeInfo2[0]);
	MINE_LOG(SANY_LOG_INFO, "--- ProtoCockpit_SendCmdRealtimeRep --- end\n");

	/* 计算消息长度 */
	SYS2NET(pHeadST->ucDataByteSize, (sizeof(CP_Realtime_Rep_Cmd_st)));

	/* 创建消息头 */
	ProtoCockpit_PackHeader(EV_CP_INFO_REALTIME_REP, EV_WP_CMD, pHeadST);

	u32DataLen = sizeof(CP_Realtime_Rep_Cmd_st);
	/* 计算Check Sum */
	pHeadST->ucData[u32DataLen] = ProtoCockpit_CalcXOR((SANY_U8 *)pHeadST, u32DataLen + sizeof(CP_Header_st));
	u32DataLen += sizeof(CP_Header_st) + 1;

	MINE_LOG(SANY_LOG_INFO, "ProtoCockpit_SendCmdRealtimeRep\n");

	u8Ret = ProtoCockpit_WriteSendBuffer(u8DataBuf, u32DataLen);

	return u8Ret;
}

/*******************************************************************************
 * 函数名称:      ProtoCockpit_HeartCheck
 * 描述:      未控车时监测心跳，控车时监测控车数据
 * 参数:        Losttimes
 * 返回值:       RET_OK/RET_ERROR
 *
 ********************************************************************************/
SANY_U8 Protocol_Cockpit::ProtoCockpit_HeartCheck(const SANY_U32 Losttimes)
{
	SANY_U8 u8Ret = RET_OK;

	if(g_u8LoginFlag == 1)
	{
		if(2 == g_u8ResponseCtrlState)
		{
			if (500  <= Losttimes)
			{
				if(vehicle_type == "SKT-I")
				{
					struWireCtrlIn.iGearOpening = 0x7D;
				}
				else if(vehicle_type== "SKT-E" || vehicle_type == "SKT-H")
				{
					struWireCtrlIn.iGearOpening = 0;
				}
				struWireCtrlIn.iBrakeOpening = 100;
				struWireCtrlIn.iThrottleOpening = 0;
				struWireCtrlIn.iWheelAngle = 0x0190;
				struWireCtrlIn.iKeyOperation2 = 0x08;	
				m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3203, 5, 1);			
				// 重置网络状态
				g_u8NetReady = 0;
				g_u8LoginFlag = 0;
				g_BrakeLimitCount = 0;
				MINE_LOG(SANY_LOG_ERROR, "DownloadData timeout, report failure, u8LostCtrlTimes: %d\n", Losttimes);		
			}	
			u8Ret = RET_ERROR;
		}
		else
		{
			if(5 * 1000  <= Losttimes)
			{
				// 重置网络状态
				g_u8NetReady = 0;
				g_u8LoginFlag = 0;
				g_BrakeLimitCount = 0;
				MINE_LOG(SANY_LOG_ERROR, "Lost HeartBeat 5s, do disconnect!!!!  u8LostCtrlTimes: %d\n", Losttimes);		
			}
			u8Ret = RET_ERROR;
		}
	}else{
		g_BrakeLimitCount = 0;
	}
	return u8Ret;
}


/*******************************************************************************
 * 函数名称:      ProtoCockpit_SendBufferInit
 * 描述:      初始化发送缓冲区
 * 参数:        none
 * 返回值:       none
 *
 ********************************************************************************/
void Protocol_Cockpit::ProtoCockpit_SendBufferInit(void)
{
	pthread_mutex_init(&SendBufferMutex, NULL);
	pthread_mutex_lock(&SendBufferMutex);
	g_u8SendBufferWriteIndex = 0; //
	g_u8SendBufferReadIndex = 0;	// 指向循环buf中的第一个位置
	g_u8SendBufferDateLen = 0;
	arrSendBuffer = (Protocol_Cockpit_Buffer_st *)calloc(PROTOCOL_COCKPIT_SEND_BUF, sizeof(Protocol_Cockpit_Buffer_st));
	pthread_mutex_unlock(&SendBufferMutex);
	MINE_LOG(SANY_LOG_INFO, "INIT SEND BUFFER!!\n");
}

/*******************************************************************************
 * 函数名称:      即时发送消息发送
 * 描述:      初始化接收缓冲区
 * 参数:        none
 * 返回值:       none
 *
 ********************************************************************************/
void Protocol_Cockpit::ProtoCockpit_RecvBufferInit(void)
{
	pthread_mutex_init(&RecvBufferMutex, NULL);
	pthread_mutex_lock(&RecvBufferMutex);
	g_u8RecvBufferWriteIndex = 0; //
	g_u8RecvBufferReadIndex = 0;	// 指向循环buf中的第一个位置
	g_u8RecvBufferDateLen = 0;
	arrRecvBuffer = (Protocol_Cockpit_Buffer_st *)calloc(PROTOCOL_COCKPIT_RECV_BUF, sizeof(Protocol_Cockpit_Buffer_st));

	pthread_mutex_unlock(&RecvBufferMutex);
	MINE_LOG(SANY_LOG_INFO, "INIT RECV BUFFER!!\n");
}

/*******************************************************************************
* 函数名称:      ProtoCockpit_WriteSendBuffer
* 描述:        将入参缓冲区写入环形发送缓冲区
* parma in:  u8 * pu8DataBuf    待写入数据缓冲区指针
			 u32 u32DataLen     数据长度
* 返回值:       RET_OK/RET_ERROR
*
********************************************************************************/
SANY_U8 Protocol_Cockpit::ProtoCockpit_WriteSendBuffer(SANY_U8 *pu8DataBuf, SANY_U32 u32DataLen)
{
	if (0 == u32DataLen || NULL == pu8DataBuf)
	{
	//	sany_log_set_output(logHandleLocal, SANY_LOG_OUT_BOTH);
		MINE_LOG(SANY_LOG_ERROR, "ProtoCockpit_WriteSendBuffer param err!\n");
		return RET_ERROR;
	}

	pthread_mutex_lock(&SendBufferMutex);

	if (PROTOCOL_COCKPIT_SEND_BUF <= g_u8SendBufferWriteIndex)
	{
		if (PROTOCOL_COCKPIT_SEND_BUF > g_u8SendBufferDateLen)
		{
			g_u8SendBufferWriteIndex = 0;
		}
		else
		{
		//	sany_log_set_output(logHandleLocal, SANY_LOG_OUT_BOTH);
			MINE_LOG(SANY_LOG_ERROR, "WriteSendBuffer full!\n");
			g_u8SendBufferFull = 1;	
			pthread_mutex_unlock(&SendBufferMutex);
			return RET_ERROR;
		}
	}
	// MINE_LOG(SANY_LOG_INFO, "writesendbuffer: s%", pu8DataBuf);
	memcpy(arrSendBuffer[g_u8SendBufferWriteIndex].szDataBuf, pu8DataBuf, u32DataLen);
	arrSendBuffer[g_u8SendBufferWriteIndex].u32SendLen = u32DataLen;

	g_u8SendBufferWriteIndex++;
	g_u8SendBufferDateLen++;

	pthread_mutex_unlock(&SendBufferMutex);

	return RET_OK;
}

/*******************************************************************************
* 函数名称:      ProtoCockpit_ReadSendBuffer
* 描述:        从环形发送缓冲区读取数据
* parma out:  u8 * pu8DataBuf    待写入数据缓冲区指针
			 u32 u32DataLen     数据长度
* 返回值:       RET_OK/RET_ERROR
*
********************************************************************************/
SANY_U8 Protocol_Cockpit::ProtoCockpit_ReadSendBuffer(SANY_U8 *pu8DataBuf, SANY_U32 *u32DataLen)
{

	if (NULL == pu8DataBuf || NULL == u32DataLen)
	{
		// printf("ProtoCockpit_ReadSendBuffer param err!\n");
		return RET_ERROR;
	}

	pthread_mutex_lock(&SendBufferMutex);

	if (0 >= g_u8SendBufferDateLen)
	{
//		MINE_LOG(SANY_LOG_ERROR, "ProtoCockpit_ReadSendBuffer  g_u8SendBufferDatalen  <= 0!!!\n");
		pthread_mutex_unlock(&SendBufferMutex);
		return RET_ERROR;
	}

	if (PROTOCOL_COCKPIT_SEND_BUF <= g_u8SendBufferReadIndex)
	{
		g_u8SendBufferReadIndex = 0;
	}

	if (0 >= arrSendBuffer[g_u8SendBufferReadIndex].u32SendLen)
	{
		// MINE_LOG(SANY_LOG_ERROR, "ProtoCockpit_ReadSendBuffer  arrSendBuffer[g_u8SendBufferReadIndex].u32SendLen  <= 0!!!\n");
		pthread_mutex_unlock(&SendBufferMutex);
		return RET_ERROR;
	}
	/* 有发送的内容，开始发送 */
	memcpy(pu8DataBuf, arrSendBuffer[g_u8SendBufferReadIndex].szDataBuf, arrSendBuffer[g_u8SendBufferReadIndex].u32SendLen);
	*u32DataLen = arrSendBuffer[g_u8SendBufferReadIndex].u32SendLen;

	g_u8SendBufferReadIndex++;
	g_u8SendBufferDateLen--;

	if((PROTOCOL_COCKPIT_SEND_BUF > g_u8SendBufferWriteIndex) || (PROTOCOL_COCKPIT_SEND_BUF > g_u8SendBufferDateLen))
	{
		g_u8SendBufferFull = 0;				
	}
	pthread_mutex_unlock(&SendBufferMutex);

	return RET_OK;
}

/*******************************************************************************
* 函数名称:      ProtoCockpit_WriteRecvBuffer
* 描述:        将入参缓冲区写入环形接收缓冲区
* parma in:  u8 * pu8DataBuf    待写入数据缓冲区指针
			 u32 u32DataLen     数据长度
* 返回值:       RET_OK/RET_ERROR
*
********************************************************************************/
SANY_U8 Protocol_Cockpit::ProtoCockpit_WriteRecvBuffer(SANY_U8 *pu8DataBuf, SANY_U32 u32DataLen)
{

	if (0 == u32DataLen || NULL == pu8DataBuf)
	{
		MINE_LOG(SANY_LOG_ERROR, "ProtoCockpit_WriteSendBuffer param err!\n");
		return RET_ERROR;
	}

	pthread_mutex_lock(&RecvBufferMutex);

	if (PROTOCOL_COCKPIT_RECV_BUF <= g_u8RecvBufferWriteIndex)
	{
		if (PROTOCOL_COCKPIT_RECV_BUF > g_u8RecvBufferDateLen)
		{
			g_u8RecvBufferWriteIndex = 0;			
		}
		else
		{
			MINE_LOG(SANY_LOG_ERROR, "WriteRecvBuffer full!\n");
			pthread_mutex_unlock(&RecvBufferMutex);
			return RET_ERROR;
		}
	}

	memcpy(arrRecvBuffer[g_u8RecvBufferWriteIndex].szDataBuf, pu8DataBuf, u32DataLen);
	arrRecvBuffer[g_u8RecvBufferWriteIndex].u32SendLen = u32DataLen;
	g_u8RecvBufferWriteIndex++;
	g_u8RecvBufferDateLen++;
	pthread_mutex_unlock(&RecvBufferMutex);

	return RET_OK;
}

/*******************************************************************************
* 函数名称:      ProtoCockpit_ReadRecvBuffer
* 描述:        从环形接收缓冲区读取数据
* parma out:  u8 * pu8DataBuf    待写入数据缓冲区指针
			 u32 u32DataLen     数据长度
* 返回值:       RET_OK/RET_ERROR
*
********************************************************************************/
SANY_U8 Protocol_Cockpit::ProtoCockpit_ReadRecvBuffer(SANY_U8 *pu8DataBuf, SANY_U32 *u32DataLen)
{

	if (NULL == pu8DataBuf || NULL == u32DataLen)
	{
		MINE_LOG(SANY_LOG_ERROR, "ProtoCockpit_ReadSendBuffer param err!\n");
		return RET_ERROR;
	}

	pthread_mutex_lock(&RecvBufferMutex);
	if (0 >= g_u8RecvBufferDateLen)
	{
		pthread_mutex_unlock(&RecvBufferMutex);
		return RET_ERROR;
	}

	if (PROTOCOL_COCKPIT_RECV_BUF <= g_u8RecvBufferReadIndex)
	{
		g_u8RecvBufferReadIndex = 0;
	}

	if (0 >= arrRecvBuffer[g_u8RecvBufferReadIndex].u32SendLen)
	{
		MINE_LOG(SANY_LOG_ERROR, "arrRecvBuffer[%d] u32SendLen err!\n", arrRecvBuffer[g_u8RecvBufferReadIndex].u32SendLen);
		pthread_mutex_unlock(&RecvBufferMutex);
		return RET_ERROR;
	}

	/* 有发送的内容，开始发送 */
	memcpy(pu8DataBuf, arrRecvBuffer[g_u8RecvBufferReadIndex].szDataBuf, arrRecvBuffer[g_u8RecvBufferReadIndex].u32SendLen);
	*u32DataLen = arrRecvBuffer[g_u8RecvBufferReadIndex].u32SendLen;

	g_u8RecvBufferReadIndex++;
	g_u8RecvBufferDateLen--;
	pthread_mutex_unlock(&RecvBufferMutex);
	return RET_OK;
}

/*******************************************************************************
threadAppUploadData 上行处理业务线程，做数据主动上报等业务处理，运行成果是把上行报文存储到发送缓冲区内
********************************************************************************/
void Protocol_Cockpit::funcAppUploadData()
{
	SANY_U32 Tick = 0;
	SANY_U8 u8Ret = RET_OK;
	static SANY_U32 u32Tick = 0; // 发送分频器
//	static SANY_U32 u8LostCtrlTimes = 0;
	SANY_U8 LoginTimes = 0;
//	SANY_U8 LoginoutTimes = 0;
	SANY_U8 flag = 0;
	MINE_LOG(SANY_LOG_INFO, "funcAppUploadData thread start!!\n");
	while (true)
	{
		if(1 == g_u8TaskStartFlag)
		{
			if(0 == flag){
				MINE_LOG(SANY_LOG_INFO, "funcAppUploadData thread restart, g_u8LoginFlag is %d!!\n", g_u8LoginFlag);
				flag = 1;
			}
			/* 判断登入状态 */
			if (0 == g_u8LoginFlag) // 未登录
			{
				/*判断是否需要登录驾舱，需要分频登录*/
				/*1s发一次*/
				if (0 == (u32Tick % SEND_LOGIN_TICK))
				{
					/*5次登录失败，则上报登录失败*/
					if (4 < LoginTimes)
					{
						MINE_LOG(SANY_LOG_ERROR, "Login 5 times fail, exit!!\n");
						m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 2, 1);
						/*登入失败，退出线程*/
						//pthread_exit(NULL);     //退出当前线程
						g_u8LoginFlag = 2;
						Task_stop();
					}
					/* 发送登录消息 */
					u8Ret = ProtoCockpit_SendCmdLogin();
					if (RET_OK != u8Ret)
					{
						continue;
					}
					LoginTimes++;
				}
			}
			else if(2 == g_u8LoginFlag){ // 登陆失败
				LoginTimes = 0;
				MINE_LOG(SANY_LOG_ERROR, "Login Reponse ERROR, exit!!\n");
				Task_stop();
			}
			else if(1 == g_u8LoginFlag){ // 登录成功
				if(g_u8SendBufferFull == 0)
				{
					LoginTimes = 0;
						/*100ms上报一次*/
					if(0 == (u32Tick % SEND_RTDATA_UPLOAD_TICK))
					{
						u8Ret = ProtoCockpit_SendCmdRealtimeRep();
						if (RET_OK != u8Ret)
						{
							MINE_LOG(SANY_LOG_ERROR, "ProtoCockpit_SendCmdRealtimeRep err!!\n");
							u32Tick++;
							continue;
						}				
					}		
				}
				else
				{
					MINE_LOG(SANY_LOG_ERROR, "SendBuffer is ful, g_u8SendBufferFull is %d\n",g_u8SendBufferFull);
				}
			}
			/*已经登录完成，则进行需要上报的数据协议组装*/
			u32Tick++;
			// }
		}else{
			u8Ret = RET_OK;
			u32Tick = 0;
			LoginTimes = 0;
			if(1 == flag){
				MINE_LOG(SANY_LOG_INFO, "funcAppUploadData thread stop!!\n");
				flag = 0;
			}
		}
		Tick++;
		if((Tick % 10000) == 0)
		{
			MINE_LOG(SANY_LOG_INFO, "funcAppUploadData thread is running\n");
			// Tick = 0;
		}
		
		usleep(1000); // 1ms
	}
}

/*******************************************************************************
threadAppDownloadData 下行处理业务线程，做协议包解析或响应
********************************************************************************/
void Protocol_Cockpit::funcAppDownloadData()
{
	SANY_U32 Tick = 0;
	SANY_U8 u8Ret = RET_OK;
	SANY_U32 u32DataLen = 0;
	SANY_U8 szRecvBuffer[PROTO_PACK_MAX_LEN] = {0};
	static SANY_U32 u8LostCtrlTimes = 0;
	SANY_U8 flag = 0;
	MINE_LOG(SANY_LOG_INFO, "funcAppDownloadData thread start!!\n");
	while (true)
	{
		if(1 == g_u8TaskStartFlag)
		{
			if(0 == flag){
				MINE_LOG(SANY_LOG_INFO, "funcAppDownloadData thread restart!!\n");
				flag = 1;
			}
			/* 读取数据 */
			bzero(szRecvBuffer, sizeof(szRecvBuffer));
			u8Ret = ProtoCockpit_ReadRecvBuffer(szRecvBuffer, &u32DataLen);
			if (RET_OK != u8Ret || 0 >= u32DataLen)
			{
				if(ProtoCockpit_HeartCheck(u8LostCtrlTimes))
				{
						//lost heart 处理
					u8LostCtrlTimes = 0;
					continue;
				}
				u8LostCtrlTimes++;
			}
			else
			{
				u8LostCtrlTimes = 0;
				/*处理数据包*/
				ProtoCockpit_ParsePacket(szRecvBuffer, u32DataLen);
			}
		}else{
			u32DataLen = 0;
			szRecvBuffer[PROTO_PACK_MAX_LEN] = {0};
			u8LostCtrlTimes = 0;
			if(1 == flag){
				MINE_LOG(SANY_LOG_INFO, "funcAppDownloadData thread stop!!\n");
				flag = 0;
			}
		}	
		
		Tick ++;
		if((Tick % 10000) == 0)
		{
			MINE_LOG(SANY_LOG_INFO, "funcAppDownloadData thread is running\n");
			// Tick = 0;
		}
		usleep(1000);
	}
}

/*数据上行业务处理线程*/
void *Protocol_Cockpit::threadAppUploadData(void *args)
{
	Protocol_Cockpit *pThread = static_cast<Protocol_Cockpit *>(args);
	pThread->funcAppUploadData();
}

/*数据下行业务处理线程*/
void *Protocol_Cockpit::threadAppDownloadData(void *args)
{
	Protocol_Cockpit *pThread = static_cast<Protocol_Cockpit *>(args);
	pThread->funcAppDownloadData();
}

/*******************************************************************************
 * 函数名称：		ProtocolCockpit_getFeedbackData
 * 描述：			获取底盘反馈数据
 * parma in:			
 * 返回值：
 * ****************************************************************************/
SANY_U8 Protocol_Cockpit::ProtoCockpit_getFeedbackData()
{
	static float finitTalDistance = 0;
	//未获取到上报数据
	// if ((WireInfo.tachograph_vehicle_speed() == 0) && (struGpsInfo.gps_yaw() == 0) && (WireInfo.steering_left_angle_info_mv() == 0) &&
	// 	(WireInfo.actual_engine_torque() == 0) && (WireInfo.brake_pedal_SYMC() == 0) && (WireInfo.engine_speed() == 0) &&
	// 	(WireInfo.total_vehicle_distance() == 0))
	// {
	// 	m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3202, 1, 1);
	// 	return RET_ERROR;
	// }

	if (WireInfo.brake_pedal_SYMC() < 250)
	{
		struWireCtrlOut.iBrakeOpening = WireInfo.brake_pedal_SYMC(); // 制动
	}
	struWireCtrlOut.iVehicle_speed = WireInfo.tachograph_vehicle_speed();	 // 车速
	// MINE_LOG(SANY_LOG_INFO, "297 iVehicle_speed (%2f)\n", WireInfo.tachograph_vehicle_speed());
	struWireCtrlOut.iWheelAngleL = WireInfo.steering_left_angle_info();		 // 车轮转角
	struWireCtrlOut.iAccumulatedMileage = WireInfo.total_vehicle_distance(); // 总计里程
	struWireCtrlOut.iPitchAngle = struGpsInfo.gps_pitch();					//俯仰角
	struWireCtrlOut.iLiftUpStatus = WireInfo.lift_longo_position_status();			//举升到位状态
	struWireCtrlOut.iLiftDownStatus = WireInfo.drop_longo_position_status();				//下降到位状态
	struWireCtrlOut.iGearOpening =  WireInfo.trans_current_gear();				//档位
	// MINE_LOG(SANY_LOG_INFO, "iGearOpening trans_current_gear (%d)\n", WireInfo.trans_current_gear());

	struWireCtrlOut.iBrakeAirPre1 = WireInfo.air_pressure_1();							//气压1
	struWireCtrlOut.iBrakeAirPre2 = WireInfo.air_pressure_2();							//气压2

	struWireCtrlOut.iHeadingAngle = struGpsInfo.gps_heading();					 // 航向角

	// MINE_LOG(SANY_LOG_INFO, " AIR_PRESSURE_1 is %f", WireInfo.air_pressure_1());

	if (WireInfo.left_turn_start_status())
	{
		struWireCtrlOut.iCombinedGearL |= 0x01;
	}
	else
	{
		struWireCtrlOut.iCombinedGearL &= 0xFE;
	}
	if (WireInfo.right_turn_start_status())
	{
		struWireCtrlOut.iCombinedGearL |= 0x01 << 1;
	}
	else
	{
		struWireCtrlOut.iCombinedGearL &= 0xFD;
	}
	if (WireInfo.low_light_start_status())
	{
		struWireCtrlOut.iCombinedGearL |= 0x01 << 2;
	}
	else
	{
		struWireCtrlOut.iCombinedGearL &= 0xFB;
	}
	if (WireInfo.high_beam_start_status())
	{
		struWireCtrlOut.iCombinedGearL |= 0x01 << 3;
	}
	else
	{
		struWireCtrlOut.iCombinedGearL &= 0xF7;
	}
	if (WireInfo.outline_lamp_start_status())
	{
		struWireCtrlOut.iCombinedGearL |= 0x01 << 5;
	}
	else
	{
		struWireCtrlOut.iCombinedGearL &= 0xDF;
	}
	if (WireInfo.wiper_start_status())
	{
		struWireCtrlOut.iCombinedGearR |= 0x01;
	}
	else
	{
		struWireCtrlOut.iCombinedGearR &= 0xFE;
	}
	if (WireInfo.wiper_slow_start_status())
	{
		struWireCtrlOut.iCombinedGearR |= 0x01 << 1;
	}
	else
	{
		struWireCtrlOut.iCombinedGearR &= 0xFD;
	}
	if (WireInfo.horn_start_status())
	{
		struWireCtrlOut.iCombinedGearR |= 0x01 << 2;
	}
	else
	{
		struWireCtrlOut.iCombinedGearR &= 0xFB;
	}
	if(g_u8EmergencyStopFlg == 1)												//急停状态上报
	{
		struWireCtrlOut.iCombinedGearR |= 0x01 << 3;
	}
	else
	{
		struWireCtrlOut.iCombinedGearR &= 0xF7;
	}
	if(g_u8RollNeedStopFlg == 1)														//溜车状态上报
	{
		struWireCtrlOut.iCombinedGearR |= 0x01 << 4;
	}
	else
	{
		struWireCtrlOut.iCombinedGearR &= 0xEF;
	}
	// 1118 刹车策略 刹车启动状态上报
	if(g_BrakeLimitCount == 1)
	{
		struWireCtrlOut.iCombinedGearR |= 0x01 << 5;
	}
	else
	{
		struWireCtrlOut.iCombinedGearR &= 0xDF;
	}
	if (WireInfo.hazard_start_status())
	{
		struWireCtrlOut.iKeyOperation1 |= 0x01;
	}
	else
	{
		struWireCtrlOut.iKeyOperation1 &= 0xFE;
	}
	if (WireInfo.rear_light_start_status())
	{
		struWireCtrlOut.iKeyOperation1 |= 0x01 << 2;
	}
	else
	{
		struWireCtrlOut.iKeyOperation1 &= 0xFB;
	}
	if (WireInfo.fog_lamp_start_status())
	{
		struWireCtrlOut.iKeyOperation1 |= 0x01 << 3;
	}
	else
	{
		struWireCtrlOut.iKeyOperation1 &= 0xF7;
	}
	if (WireInfo.head_light_status())
	{
		struWireCtrlOut.iKeyOperation1 |= 0x01 << 4;
	}
	else
	{
		struWireCtrlOut.iKeyOperation1 &= 0xEF;
	}
	if (WireInfo.exhaust_brake_status())
	{
		struWireCtrlOut.iKeyOperation1 |= 0x01 << 5;
	}
	else
	{
		struWireCtrlOut.iKeyOperation1 &= 0xDF;
	}
	if (WireInfo.axle_differential())
	{
		struWireCtrlOut.iKeyOperation1 |= 0x01 << 6;
	}
	else
	{
		struWireCtrlOut.iKeyOperation1 &= 0xBF;
	}
	if (1 == WireInfo.lift_condition())
	{
		struWireCtrlOut.iKeyOperation1 |= 0x01 << 7;
	}
	else
	{
		struWireCtrlOut.iKeyOperation1 &= 0x7F;
	}
	if (WireInfo.flash_start_status())
	{
		struWireCtrlOut.iKeyOperation2 |= 0x01 << 1;
	}
	else
	{
		struWireCtrlOut.iKeyOperation2 &= 0xFD;
	}
	if(WireInfo.Force_State())
	{
		struWireCtrlOut.iKeyOperation2 |= 0x01 << 2;
	}
	else
	{
		struWireCtrlOut.iKeyOperation2 &= 0xFB;
	}
	if (WireInfo.parking())
	{
		struWireCtrlOut.iKeyOperation2 |= 0x01 << 3;
	}
	else
	{
		struWireCtrlOut.iKeyOperation2 &= 0xF7;
	}
	if (2 == WireInfo.lift_condition())
	{
		struWireCtrlOut.iKeyOperation2 |= 0x01 << 7;
	}
	else
	{
		struWireCtrlOut.iKeyOperation2 &= 0x7F;
	}
	
	//车辆驾驶状态 
	if((WireInfo.vcu_state() == 5) && (WireInfo.Remote_or_Unman() == 8))
	{
		struWireCtrlOut.iWorkStatus = 2;			//remote
		g_u8ResponseCtrlState = 2;
		if(g_BrakeLimitCount == 0){
			g_BrakeLimitCount = 2;
		}
	}
	else if((WireInfo.vcu_state() == 5) && (WireInfo.Remote_or_Unman() == 6))
	{
		struWireCtrlOut.iWorkStatus = 3;			//unman
		g_u8ResponseCtrlState = 3;
		g_BrakeLimitCount = 0;
	}
	else
	{
		struWireCtrlOut.iWorkStatus = 1;			//manual
		g_u8ResponseCtrlState = 1;
		g_u8CtrlState = 0;
		g_BrakeLimitCount = 0;
	}

	if (vehicle_type == "SKT-I")
	 {
		struWireCtrlOut.iThrottleOpening = WireInfo.accel_pedal_position(); // 油门

		/* 20250306 调整发动机转速，实际范围为0~5000 ，传递 0~40000，舱端同步做调整 ----- start*/
		struWireCtrlOut.iEngineSpeed = WireInfo.engine_speed() * 8;	 // 发动机转速 SKT-I
		/* 20250306 调整发动机转速，实际范围为0~5000 ，传递 0~40000，舱端同步做调整 ----- end*/

		// struWireCtrlOut.iHeadingAngle = struGpsInfo.gps_heading();					 // 航向角
		struWireCtrlOut.iRemainingOil = WireInfo.fuel_level();					//剩余油量
		struWireCtrlOut.iWaterTemp = WireInfo.engine_coolant_temperature();				//水温
		if (finitTalDistance == 0)
		{
			finitTalDistance = WireInfo.total_vehicle_distance();
		}
		/*小计里程 = 总里程-初始里程*/
		struWireCtrlOut.iSubtotalMileage = (WireInfo.total_vehicle_distance() - finitTalDistance);
	 }
	else if(vehicle_type == "SKT-E")
	 {
		struWireCtrlOut.iThrottleOpening = WireInfo.actual_engine_torque(); // 油门

		/* 20250306 调整发动机转速，实际范围为-6000 ～6000 ，传递正数 0～12000，舱端同步做调整 ----- start*/
		struWireCtrlOut.iEngineSpeed = WireInfo.main_driveMotor_speed_raw() + 6000;			// 发动机转速
		/* 20250306 调整发动机转速，实际范围为-6000 ～6000 ，传递正数 0～12000，舱端同步做调整 ----- end*/

		// struWireCtrlOut.iHeadingAngle = struGpsInfo.gps_yaw();							// 航向角
		struWireCtrlOut.iRemainingOil = WireInfo.SOC();					//剩余电量
		struWireCtrlOut.iWaterTemp = 0;									//水温 电车无水温
		if (finitTalDistance == 0)
		{
			finitTalDistance = WireInfo.total_vehicle_distance();
		}
		/*小计里程 = 总里程-初始里程*/
		struWireCtrlOut.iSubtotalMileage = (WireInfo.total_vehicle_distance() - finitTalDistance) / 1000;
	 }
	 /* 20250306 增加130混型数据解析 ----- start*/
	 else if(vehicle_type == "SKT-H"){
		struWireCtrlOut.iThrottleOpening = WireInfo.accel_pedal_position(); // 油门

		/* 20250306 调整发动机转速，实际范围为0~5000 ，传递 0~40000，舱端同步做调整 与油车一致 ----- start*/
		struWireCtrlOut.iEngineSpeed = WireInfo.engine_speed() * 8;	 // 发动机转速 SKT-H
		/* 20250306 调整发动机转速，实际范围为0~5000 ，传递 0~40000，舱端同步做调整 ----- end*/

	 	
		struWireCtrlOut.iRemainingOil = WireInfo.fuel_level();			// 油箱油量
		struWireCtrlOut.iWaterTemp = WireInfo.engine_coolant_temperature();				//水温
		if (finitTalDistance == 0)
		{
			finitTalDistance = WireInfo.total_vehicle_distance();
		}
		/*小计里程 = 总里程-初始里程*/
		struWireCtrlOut.iSubtotalMileage = (WireInfo.total_vehicle_distance() - finitTalDistance) / 1000;
	}
	 /* 20250306 增加130混型数据解析 ----- end*/
	// 安全措施-故障信息
	ProtoCockpit_PackSafeFeedbackInfo();

	MINE_LOG(SANY_LOG_INFO, "---ProtoCockpit_getFeedbackData --- start\n");
	MINE_LOG(SANY_LOG_INFO, " VCU_State %d, 5 - unman\n", WireInfo.vcu_state());
	MINE_LOG(SANY_LOG_INFO, "Remote_or_Unman  %d, 6 - unman; 8 - remote\n", WireInfo.Remote_or_Unman());
	MINE_LOG(SANY_LOG_INFO, "--- ProtoCockpit_getFeedbackData --- end\n");
	return RET_OK;
}

/*************************************************************************************
 * 获取安全模块上报的故障信息，并解析
*/
SANY_U8 Protocol_Cockpit::ProtoCockpit_PackSafeFeedbackInfo(){
	/* 溜车*/
	if(struSafeFeedbackInfo.rolling_need_stop()){
		struWireCtrlOut.iSafeInfo1 |= 0x01;
	}else{
		struWireCtrlOut.iSafeInfo1 &= 0xFE;
	}
	/* 驻车溜车*/
	if(struSafeFeedbackInfo.parking_rolling()){
		struWireCtrlOut.iSafeInfo1 |= 0x01 << 1;
	}else{
		struWireCtrlOut.iSafeInfo1 &= 0xFD;
	}
	/*刹车异常*/
	if(struSafeFeedbackInfo.braking_failure()){
		struWireCtrlOut.iSafeInfo1 |= 0x01 << 2;
	}else{
		struWireCtrlOut.iSafeInfo1 &= 0xFB;
	}
	/* 打滑*/
	if(struSafeFeedbackInfo.slip_rep()){
		struWireCtrlOut.iSafeInfo1 |= 0x01 << 3;
	}else{
		struWireCtrlOut.iSafeInfo1 &= 0xF7;
	}
	/*位置突变停车*/
	if(struSafeFeedbackInfo.distance_stop_flag()){
		struWireCtrlOut.iSafeInfo1 |= 0x01 << 4;
	}else{
		struWireCtrlOut.iSafeInfo1 &= 0xEF;
	}
	/*位置不变停车*/
	 if(struSafeFeedbackInfo.position_nochange_flag()){
		struWireCtrlOut.iSafeInfo1 |= 0x01 << 5;
	}else{
		struWireCtrlOut.iSafeInfo1 &= 0xDF;
	}
	/*驻车异常*/
	if(struSafeFeedbackInfo.parking_failure()){
		struWireCtrlOut.iSafeInfo1 |= 0x01 << 6;
	}else{
		struWireCtrlOut.iSafeInfo1 &= 0xBF;
	}
	/*制动力衰减*/
	if(struSafeFeedbackInfo.braking_deceleration_weakened()){
		struWireCtrlOut.iSafeInfo1 |= 0x01 << 7;
	}else{
		struWireCtrlOut.iSafeInfo1 &= 0x7F;
	}
	/* 热衰退故障*/
	if(struSafeFeedbackInfo.braking_heat_fade()){
		struWireCtrlOut.iSafeInfo2 |= 0x01;
	}else{
		struWireCtrlOut.iSafeInfo2 &= 0xFE;
	}
	/* AEB停车*/
	if(struSafeFeedbackInfo.AEB_stop()){
		struWireCtrlOut.iSafeInfo2 |= 0x01 << 1;
	}else{
		struWireCtrlOut.iSafeInfo2 &= 0xFD;
	}
	if(struSafeFeedbackInfo.transmission_state()){
		struWireCtrlOut.iSafeInfo2 |= 0x01 << 7;
	}else{
		struWireCtrlOut.iSafeInfo2 &= 0x7F;
	}
	MINE_LOG(SANY_LOG_INFO, "---ProtoCockpit_SafeFeedbackInfo --- start\n");
	MINE_LOG(SANY_LOG_INFO, "rolling_need_stop: %d\n", struSafeFeedbackInfo.rolling_need_stop());
	MINE_LOG(SANY_LOG_INFO, "parking_rolling: %d\n", struSafeFeedbackInfo.parking_rolling());
	MINE_LOG(SANY_LOG_INFO, "braking_failure: %d\n", struSafeFeedbackInfo.braking_failure());
	MINE_LOG(SANY_LOG_INFO, "slip_rep: %d\n", struSafeFeedbackInfo.slip_rep());
	MINE_LOG(SANY_LOG_INFO, "distance_stop_flag: %d\n", struSafeFeedbackInfo.distance_stop_flag());
	MINE_LOG(SANY_LOG_INFO, "position_nochange_flag: %d\n", struSafeFeedbackInfo.position_nochange_flag());
	MINE_LOG(SANY_LOG_INFO, "parking_failure: %d\n", struSafeFeedbackInfo.parking_failure());
	MINE_LOG(SANY_LOG_INFO, "braking_deceleration_weakened: %d\n", struSafeFeedbackInfo.braking_deceleration_weakened());
	MINE_LOG(SANY_LOG_INFO, "braking_heat_fade: %d\n", struSafeFeedbackInfo.braking_heat_fade());
	MINE_LOG(SANY_LOG_INFO, "AEB_stop: %d\n", struSafeFeedbackInfo.AEB_stop());
	MINE_LOG(SANY_LOG_INFO, "transmission_state: %d\n", struSafeFeedbackInfo.transmission_state());
	MINE_LOG(SANY_LOG_INFO, "---ProtoCockpit_SafeFeedbackInfo --- end\n");
	return RET_OK;
}

/*
溜车
*/
float Protocol_Cockpit::ProtoCockpit_SpeedHeading(const float gps_vn, const float gps_ve, const float heading)
{
	 float speed_heading = 0,vn = 0, ve = 0, theta = 0;
    vn = fabs(gps_vn);
    ve = fabs(gps_ve);

    if(vn > 0.1 && ve > 0.1 )
    {
        theta = atan(ve / vn)*(180/3.14);
        if(gps_vn > 0 && gps_ve >= 0)
            speed_heading = theta;
        else if(gps_vn < 0 && gps_ve > 0)
            speed_heading = 180 - theta;
        else if(gps_vn < 0 && gps_ve <= 0)
            speed_heading = 180 + theta;
        else if(gps_vn >0 && gps_ve < 0)
            speed_heading = 360 - theta;
    }
    else
    {
        if(gps_ve > 0.1 && vn < 0.1)
            speed_heading = 90;
        else if (gps_ve < (-0.1) && vn < 0.1)
            speed_heading = 270;
        else if (gps_vn > 0.1 && ve < 0.1)
            speed_heading = 0;
        else if (gps_vn < (-0.1) && ve < 0.1)
            speed_heading = 180;
        else
            speed_heading = heading;
    }
    // if(steering_angle > 10 || steering_angle < -10)
    //         speed_heading = heading;
    return speed_heading;
}

SANY_U8 Protocol_Cockpit::ProtoCockpit_Rolling()
{
	float speed_heading = ProtoCockpit_SpeedHeading(struGpsInfo.gps_vn(),struGpsInfo.gps_ve(),struGpsInfo.gps_heading());
    float vehicle_heading = struGpsInfo.gps_heading();
	
	if ( (fabs(struGpsInfo.gps_vn()) > 0.1 || fabs(struGpsInfo.gps_ve()) > 0.1) && (WireInfo.trans_current_gear() == -1 || WireInfo.trans_current_gear() == 13))
    {
        vehicle_heading += 180;
        if (vehicle_heading >= 360)
                vehicle_heading -= 360;
    }
    float heading_error = fabs(vehicle_heading - speed_heading);
    static SANY_U32 heading_count = 0;
    if (heading_error > 150 && heading_error < 210)
    {
        if (heading_count >= 20)
        {
            heading_count = 20;
            g_u8RollNeedStopFlg = 1;
            MINE_LOG(SANY_LOG_ERROR, "g_u8RollNeedStopFlg : %d\n", g_u8RollNeedStopFlg);
			MINE_LOG(SANY_LOG_INFO, "speed_heading, gps_vn : %f, gps_ve : %f, speed_heading is : %f, heading is : %f\n", struGpsInfo.gps_vn(), struGpsInfo.gps_ve(), speed_heading, struGpsInfo.gps_heading());
        }
        else
        {
            heading_count++;
        }
    }
    else
    {
        heading_count = 0;
		g_u8RollNeedStopFlg = 0;
    }
	return g_u8RollNeedStopFlg;
}

SANY_U8 Protocol_Cockpit::ProtoCockpit_initCtrlData()
{
	if(vehicle_type == "SKT-E" || vehicle_type == "SKT-H")
	{
		struWireCtrlIn.iGearOpening = 0;
	}
	else
	{
		struWireCtrlIn.iGearOpening = 0x7D;
	}
	struWireCtrlIn.iBrakeOpening = 100;
	struWireCtrlIn.iWheelAngle =  0x0190;
	struWireCtrlIn.iThrottleOpening = 0;
	struWireCtrlIn.iCombinedGearL = 0;
	struWireCtrlIn.iCombinedGearR = 0;
	struWireCtrlIn.iKeyOperation1 = 0;
	// struWireCtrlIn.iKeyOperation2 = 0x20;
	BrakeValue = 0;
	struWireCtrlIn.iKeyOperation2 = 0x08;		//驻车
	struWireCtrlIn.iSafeRelease = 0x00;	// 安全模块-关闭清除故障
	struWireCtrlIn.iSafeProtect = 0x00;	// 安全故障-进入安全模式
	return true;
}

/*坡起策略*/
float Protocol_Cockpit::ProtoCockpit_SlopeStart(){
	float current_pitch = fabs(struGpsInfo.gps_pitch());

	float return_brake = 0;
	// bool is_danger_Slope = false;
	// // 坡起延迟上锁
	// if(fabs(current_pitch) >= DANGER_PITCH_THRESHOLD && brake_count < BRAKE_DELAY_THRESHOLD && 
	// 	((struWireCtrlIn.iBrakeOpening == 0) && (BrakeValue > 0.1f) && (struWireCtrlIn.iThrottleOpening < 1))  ||
	// 	(struWireCtrlIn.iThrottleOpening > 10) && (struWireCtrlOut.iVehicle_speed < 1.0f) && (WireInfo.trans_current_gear() != 0) && (WireInfo.trans_current_gear() != -1)){
	// 		is_danger_Slope = true;
	// }else{
	// 	// 坡起延迟解锁
	// 	if((fabs(current_pitch) > DANGER_PITCH_THRESHOLD && ((struWireCtrlIn.iThrottleOpening <= 10) && (BrakeValue == 0)) || (struWireCtrlOut.iVehicle_speed >= 1.0f)) ||
	// 			fabs(current_pitch) < DANGER_PITCH_THRESHOLD){
	// 				recovery_count ++;
	// 				if(recovery_count < RECOVERY_TIME_THRESHOLD)
	// 				{
	// 					is_danger_Slope  = false;
	// 				}
	// 	}
	// }
	
	if(current_pitch >= DANGER_PITCH_THRESHOLD)
	{
		BRAKE_DELAY_THRESHOLD =  static_cast<unsigned int>((mass_coefficient_thr *(std::max(0.5f, std::min(1.5f, current_pitch * 0.3f )) * 1000.0f ))/ 50.0f);
		// BRAKE_DELAY_THRESHOLD = std::max(BRAKE_DELAY_THRESHOLD, brake_delay_threshold_here);
		// MINE_LOG(SANY_LOG_INFO, "config brakeing delay: %d, current_pitch: %f\n", BRAKE_DELAY_THRESHOLD, current_pitch);
		if((struWireCtrlIn.iBrakeOpening == 0) && (BrakeValue > 0.1f) && (brake_count < BRAKE_DELAY_THRESHOLD) && (struWireCtrlIn.iThrottleOpening < 1))
		{
			return_brake = (float)read_brakeing;
		//	pCtrlInfo->traffic_brake_req((float)read_brakeing);
			brake_count ++;
			recovery_count = 0;
			MINE_LOG(SANY_LOG_INFO, "brakeing delay %d times, gps_pitch is : %f, brake_count: %d,  traffic_brake_req: %f\n", BRAKE_DELAY_THRESHOLD, current_pitch, brake_count, return_brake);
			return return_brake;
		}
		else if((struWireCtrlIn.iThrottleOpening > 30) && (struWireCtrlOut.iVehicle_speed < 1) && (WireInfo.trans_current_gear() != 0) && (WireInfo.trans_current_gear() != -1) && (brake_count < BRAKE_DELAY_THRESHOLD))
		{
			return_brake = (float)read_brakeing;
		//	pCtrlInfo->traffic_brake_req((float)read_brakeing);
			brake_count ++;
			recovery_count = 0;
			MINE_LOG(SANY_LOG_INFO, "brakeing delay %d times, gps_pitch is : %f, current speed: %f, brake_count: %d, traffic_brake_req: %f, current gear: %d\n", BRAKE_DELAY_THRESHOLD, current_pitch, struWireCtrlOut.iVehicle_speed, brake_count, return_brake, WireInfo.trans_current_gear());
			return return_brake;
		}
		else if((struWireCtrlIn.iThrottleOpening <= 30) && (BrakeValue == 0) || (struWireCtrlOut.iVehicle_speed >= 1.0f)){
			brake_count = 0;
			recovery_count = 0;
			BrakeValue = (float)struWireCtrlIn.iBrakeOpening;
			return_brake = (float)struWireCtrlIn.iBrakeOpening;
			MINE_LOG(SANY_LOG_INFO, "no brakeing delay ,clear brake_count, iThrottleOpening: %d, BrakeValue: %f, iVehicle_speed: %f, iPitch: %f\n", struWireCtrlIn.iThrottleOpening, BrakeValue, struWireCtrlOut.iVehicle_speed, current_pitch);
			return return_brake;
		}
		// else if((struWireCtrlIn.iThrottleOpening <= 10) && (BrakeValue == 0))
		// {
		// 	recovery_count++;
		// 	BrakeValue = (float)struWireCtrlIn.iBrakeOpening;
		// 	if(recovery_count >= RECOVERY_TIME_THRESHOLD){
		// 		brake_count = 0;
		// 		return_brake = (float)struWireCtrlIn.iBrakeOpening;
		// 		MINE_LOG(SANY_LOG_INFO, "no brakeing delay ,clear brake_count, iThrottleOpening: %d, BrakeValue: %f, iPitch: %f\n", struWireCtrlIn.iThrottleOpening, BrakeValue, current_pitch);
		// 	}else{
		// 		return_brake = brake_count == 0 ? (float)struWireCtrlIn.iBrakeOpening : (float)read_brakeing;
		// 		MINE_LOG(SANY_LOG_INFO, "re brakeing delay, recovery_count: %d, iThrottleOpening: %d, BrakeValue: %f, iPitch: %f\n", recovery_count, struWireCtrlIn.iThrottleOpening, BrakeValue, current_pitch);
		// 	}
		// 	return return_brake;
		// 	// MINE_LOG(SANY_LOG_INFO, "no brakeing delay, recovery_count: %d, iThrottleOpening: %d, BrakeValue: %dn", recovery_count, struWireCtrlIn.iThrottleOpening, BrakeValue);
		// }else if(struWireCtrlOut.iVehicle_speed >= 1.0f){
		// 	recovery_count ++;
		// 	BrakeValue = (float)struWireCtrlIn.iBrakeOpening;
		// 	if(recovery_count >= RECOVERY_TIME_THRESHOLD){
		// 		brake_count = 0;
		// 		return_brake = (float)struWireCtrlIn.iBrakeOpening;
		// 		MINE_LOG(SANY_LOG_INFO, "no brakeing delay ,clear brake_count, iVehicle_speed: %f, iPitch: %f\n", struWireCtrlOut.iVehicle_speed, current_pitch);
		// 	}else{
		// 		return_brake = brake_count == 0 ? (float)struWireCtrlIn.iBrakeOpening : (float)read_brakeing;
		// 		MINE_LOG(SANY_LOG_INFO, "re brakeing delay, recovery_count: %d, iVehicle_speed: %f, iPitch: %f\n", recovery_count, struWireCtrlOut.iVehicle_speed, current_pitch);
		// 	}
		// 	return return_brake;
		// }
		else 
		{
			// brake_count = 0;
			// recovery_count = 0;
			return_brake = (float)struWireCtrlIn.iBrakeOpening;
			BrakeValue = (float)struWireCtrlIn.iBrakeOpening;
			MINE_LOG(SANY_LOG_INFO, "no brakeing delay !\n");
			return return_brake;
		}	
	} 
	else
	{
		recovery_count ++;
		if(recovery_count >= RECOVERY_TIME_THRESHOLD){
			brake_count = 0;
			// pCtrlInfo->traffic_brake_req((float)struWireCtrlIn.iBrakeOpening);
			return_brake = (float)struWireCtrlIn.iBrakeOpening;
			MINE_LOG(SANY_LOG_INFO, "no brakeing delay ,clear brake_count, iPitch: %f\n", current_pitch);
		}else{
			return_brake = brake_count == 0 ? (float)struWireCtrlIn.iBrakeOpening : (float)read_brakeing;
			MINE_LOG(SANY_LOG_INFO, "re brakeing delay, recovery_count: %d, iPitch: %f\n", recovery_count, current_pitch);
		}
	}
	return return_brake;
}

/* 舱端控制数据转为底盘数据*/
SANY_U8 Protocol_Cockpit::ProtoCockpit_getCtrlDate(ImpleResultInfo *pCtrlInfo)
{
	if (!pCtrlInfo)
	{
		MINE_LOG(SANY_LOG_ERROR, "ProtoCockpit_getCtrlDate param invalid \n");

		return RET_ERROR;
	}
	bzero(pCtrlInfo, sizeof(ImpleResultInfo));

	pCtrlInfo->vilid_count(g_297count);
	if (g_297count < 255)
	{
		g_297count++;
	}
	else
	{
		g_297count = 0;
	}

	// if (g_u8CtrlState == 1) 
	// {
	// 	pCtrlInfo->Remote_enable(3);
	// 	pCtrlInfo->unmanned_enable(1);
	// }
	// else if (g_u8CtrlState == 2)
	// {
	// 		pCtrlInfo->Remote_enable(5);
	// 		pCtrlInfo->unmanned_enable(1);
	// }
	// else if (g_u8CtrlState == 3)
	// {
	// 		pCtrlInfo->Remote_enable(5);
	// 		pCtrlInfo->unmanned_enable(0);
	// }
			
	// 20250115 增加条件 响应状态不是遥控时，下发的数据为初始化数据
	if(g_u8LoginFlag == 0 || g_u8ResponseCtrlState != 2)
	{
		ProtoCockpit_initCtrlData();	
		MINE_LOG(SANY_LOG_ERROR, "Login flag is :  %d, init control data!!\n", g_u8LoginFlag);
	}

	pCtrlInfo->engine_override_ctrl_mode(0b10);
	pCtrlInfo->engine_req_speed_ctrl_condition(0b10);
	pCtrlInfo->override_ctrl_mode_priority(0b00);
	pCtrlInfo->engine_req_speed(2100); // rpm

	// 2.3.3 增加默认值
	pCtrlInfo->on_start_enable(false);
	/*-------------------油门，0-100-------------------*/
	pCtrlInfo->engine_req_torque((float)struWireCtrlIn.iThrottleOpening);
	printf("engine_req_torque: %f\n", pCtrlInfo->engine_req_torque());
	/*-------------------车轮转角，-40  -40--------------------*/
	pCtrlInfo->steering_angle_req((float)struWireCtrlIn.iWheelAngle * 0.1 - 40);
	printf("steering angle  req: %f\n", pCtrlInfo->steering_angle_req());
	/*-------------------档位，1-R, 3-D, 其他-N-------------------*/
	if(vehicle_type == "SKT-I")
	{
		if (1 == struWireCtrlIn.iGearOpening) // R
		{
			pCtrlInfo->trans_req_gear(0xDF); // 0xDF;									
		}
		else if (3 == struWireCtrlIn.iGearOpening) // D
		{
			pCtrlInfo->trans_req_gear(0xFC); // 0xFC;
		}
		else // N
		{
			pCtrlInfo->trans_req_gear(0x7D); // 0x7D;
		}
	}
	else if(vehicle_type == "SKT-E" || vehicle_type == "SKT-H")
	{
		if (1 == struWireCtrlIn.iGearOpening) // R
		{
			pCtrlInfo->trans_req_gear(2); // 0xDF;									
		}
		else if (3 == struWireCtrlIn.iGearOpening) // D
		{
			pCtrlInfo->trans_req_gear(1); // 0xFC;
		}
		else // N
		{
			pCtrlInfo->trans_req_gear(0); // 0x7D;
		}
	}
	printf("ctrl gear: %d\n", pCtrlInfo->trans_req_gear());
		
	/*-------------------制动，0-100-------------------*/
	//pCtrlInfo->traffic_brake_req((float)struWireCtrlIn.iBrakeOpening);
	
	
	/// 原坡起策略位置
	pCtrlInfo->traffic_brake_req(ProtoCockpit_SlopeStart());


	if (pCtrlInfo->traffic_brake_req() > 0)
	{
		pCtrlInfo->brake_req_status(true);
		pCtrlInfo->brake_light_ctrl(true);
	}
	else
	{
		pCtrlInfo->brake_req_status(false);
		pCtrlInfo->brake_light_ctrl(false);
	}

	/*组合开关左*/
	if (struWireCtrlIn.iCombinedGearL & 0x01)
	{
		// 开左转向
		pCtrlInfo->left_turn_signal_ctrl(true);
	}
	if (struWireCtrlIn.iCombinedGearL & 0x02)
	{
		// 开右转向
		pCtrlInfo->right_turn_signal_ctrl(true);
	}
	if (struWireCtrlIn.iCombinedGearL & 0x04)
	{
		// 开近光灯
		pCtrlInfo->low_light_ctrl(true);
	}
	if (struWireCtrlIn.iCombinedGearL & 0x08)
	{
		// 开远光灯
		pCtrlInfo->high_beam_ctrl(true);
	}
	if (struWireCtrlIn.iCombinedGearL & 0x10)
	{
		// 开雾灯
		pCtrlInfo->fog_lamp_ctrl(true);
	}
	if (struWireCtrlIn.iCombinedGearL & 0x20)
	{
		// 开示廓灯
		pCtrlInfo->outline_lamp_ctrl(true);
	}

	/*组合开关右*/
	if (struWireCtrlIn.iCombinedGearR & 0x01)
	{
		// 开雨刮喷水
		pCtrlInfo->wiper_ctrl(true);
	}
	if (struWireCtrlIn.iCombinedGearR & 0x02)
	{
		// 开雨刮低速
		pCtrlInfo->wiper_low_speed_ctrl(true);
	}
	if (struWireCtrlIn.iCombinedGearR & 0x04)
	{
		// 开喇叭
		pCtrlInfo->horn_start(true);
	}

	/*按键操作1*/
	if (struWireCtrlIn.iKeyOperation1 & 0x01)
	{
		// 开双闪
		pCtrlInfo->hazard_warn_lamp_ctrl(true);
	}
	if (struWireCtrlIn.iKeyOperation1 & 0x02)
	{
		// 开气喇叭
		pCtrlInfo->horn_start(true);
	}
	if (struWireCtrlIn.iKeyOperation1 & 0x04)
	{
		// 开后照灯
		pCtrlInfo->rear_light_ctrl(true);
	}
	if (struWireCtrlIn.iKeyOperation1 & 0x08)
	{
		// 开雾灯
		pCtrlInfo->fog_lamp_ctrl(true);
	}
	if (struWireCtrlIn.iKeyOperation1 & 0x10)
	{
		// 开前照灯
		pCtrlInfo->head_light_ctrl(true);
	}
	/* 20250220 liuss 将该物理按键改为130H补气开关：1.130无法加油补气；2. 领导指定排气制动；-----start*/
	if (struWireCtrlIn.iKeyOperation1 & 0x20)
	{
		// 发动机排气制动开
		// pCtrlInfo->exhaust_brake_req(true);
		if(vehicle_type == "SKT-H"){
			pCtrlInfo->Parking_airSup_state(true);
		}else{
			pCtrlInfo->exhaust_brake_req(true);
		}
	}else{
		if(vehicle_type == "SKT-H"){
			pCtrlInfo->Parking_airSup_state(false);
		}else{
			pCtrlInfo->exhaust_brake_req(false);
		}
	}
	//130
	MINE_LOG(SANY_LOG_INFO, "receive Parking_airSup_state (%d)\n", pCtrlInfo->Parking_airSup_state());
	/* 20250220 liuss 将该物理按键改为130H补气开关：1.130无法加油补气；2. 领导指定排气制动；-----end*/
	if (struWireCtrlIn.iKeyOperation1 & 0x40)
	{
		// 轮差开
		pCtrlInfo->axial_ctrl(true);
	}
	if (struWireCtrlIn.iKeyOperation1 & 0x80)
	{
		// 举升开
	}
	/*按键操作2*/
	if (struWireCtrlIn.iKeyOperation2 & 0x01)
	{
		// 开侧雨刷
	}
	if (struWireCtrlIn.iKeyOperation2 & 0x02)
	{
		// 开爆闪灯
		pCtrlInfo->flash_ctrl(true);
	}
	if (struWireCtrlIn.iKeyOperation2 & 0x04)
	{
		// 开取力
	}
	if (struWireCtrlIn.iKeyOperation2 & 0x08)
	{
		// 开驻车
		pCtrlInfo->parking_req(1);
	}
	else
	{
		pCtrlInfo->parking_req(0);
	}
	if (struWireCtrlIn.iKeyOperation2 & 0x10)
	{
		// 开遥控驾驶
	}
	if (struWireCtrlIn.iKeyOperation2 & 0x20)
	{
		// 开电磁
	}
	if (struWireCtrlIn.iKeyOperation2 & 0x40)
	{
		// 开发动机自检
	}
	if(vehicle_type == "SKT-I")
	{
		if((struWireCtrlIn.iKeyOperation2 & 0x80) && !WireInfo.drop_longo_position_status())
		{
			// 开举升下降
			pCtrlInfo->container_ctrl(2);
			printf("begin to lift down, container ctrl status: %d\n", pCtrlInfo->container_ctrl());
		}
		else if((struWireCtrlIn.iKeyOperation1 & 0x80) && !WireInfo.lift_longo_position_status())
		{
			// 开举升上升
			pCtrlInfo->container_ctrl(1);
			printf("begin to lift up, container ctrl status: %d\n", pCtrlInfo->container_ctrl());
		}
		else
		{
			// 举升暂停
			pCtrlInfo->container_ctrl(0);
		}
	}
	else if(vehicle_type == "SKT-E" || vehicle_type == "SKT-H")
	{
		// 电车举升需取力
		if((struWireCtrlIn.iKeyOperation2 & 0x80) && (struWireCtrlIn.iKeyOperation2 & 0x04))
		{
			// 开举升下降
			pCtrlInfo->container_ctrl(2);
			printf("begin lift down, container status: %d, PTO: %d\n", pCtrlInfo->container_ctrl(), struWireCtrlIn.iKeyOperation2 & 0x04);
		}
		else if((struWireCtrlIn.iKeyOperation1 & 0x80) && (struWireCtrlIn.iKeyOperation2 & 0x04))
		{
			// 开举升上升
			pCtrlInfo->container_ctrl(1);
			printf("begin lift up, container status: %d, PTO: %d\n", pCtrlInfo->container_ctrl(), struWireCtrlIn.iKeyOperation2 & 0x04);
		}
		else
		{
			// 举升暂停
			pCtrlInfo->container_ctrl(0);
		}
	}
	
	if(1 == struWireCtrlIn.iSafeRelease){
		// 安全措施-开启清除故障
		pCtrlInfo->release_measure(true);
	}else if(0 == struWireCtrlIn.iSafeRelease){
		// 安全措施-关闭清除故障
		pCtrlInfo->release_measure(false);
	}

	if(1 == struWireCtrlIn.iSafeProtect){
		// 安全措施-进入安全模式
		pCtrlInfo->protect_measure(true);
	}else{
		pCtrlInfo->protect_measure(false);
	}
	MINE_LOG(SANY_LOG_INFO, "receive SafeRelease: %d, SafeProtect: %d\n", pCtrlInfo->release_measure(), pCtrlInfo->protect_measure());

	/*判断溜车*/
	if(ProtoCockpit_Rolling())
	{
		g_u8EmergencyStopFlg = 1;
	}

	SANY_U8 brakeLimitRET = ProtoCockpit_CheckBrakeLimit();
	/*判断是否急停，请求急停则请求驻车*/
	if (1 == g_u8EmergencyStopFlg || brakeLimitRET == RET_ERROR)
	{
		if("SKT-I" == vehicle_type)
		{
			pCtrlInfo->trans_req_gear(0x7D);		//N
		}
		else if ("SKT-E" == vehicle_type || vehicle_type == "SKT-H")
		{
			pCtrlInfo->trans_req_gear(0);		//N
		}
		pCtrlInfo->traffic_brake_req(100);		//制动
		pCtrlInfo->parking_req(true);		//驻车
		pCtrlInfo->engine_req_torque(0);	//油门
		pCtrlInfo->brake_req_status(true);		//刹车请求状态
		pCtrlInfo->brake_light_ctrl(true);		//制动灯
		MINE_LOG(SANY_LOG_INFO, "init Emergency, gear: %d,  parking_req: %d,  traffic_brake_req: %f, g_u8EmergencyStopFlg: %d, brakeLimitRET: %d\n", pCtrlInfo->trans_req_gear(), pCtrlInfo->parking_req(), pCtrlInfo->traffic_brake_req(), g_u8EmergencyStopFlg, brakeLimitRET);
		// printf("start EmergencyStop, gear: %d,  parking_req: %d,  traffic_brake_req: %f\n", pCtrlInfo->trans_req_gear(), pCtrlInfo->parking_req(), pCtrlInfo->traffic_brake_req());
	}
	MINE_LOG(SANY_LOG_INFO, "send control, gear: %d, parking_req: %d, brake_req: %f, throttle_req: %f\n", pCtrlInfo->trans_req_gear(), pCtrlInfo->parking_req(), pCtrlInfo->traffic_brake_req(), pCtrlInfo->engine_req_torque());
	
	return RET_OK;
}

SANY_U8 Protocol_Cockpit::ProtoCockpit_CheckBrakeLimit(){
	SANY_U8 ret = RET_ERROR;
	if(g_BrakeLimitCount == 1){
		return RET_OK;
	}
	if(g_BrakeLimitCount >= 2){
		if(struWireCtrlIn.iBrakeOpening >= 50){
			g_BrakeLimitCount++;
		}
	}
	if(g_BrakeLimitCount >= 12){
		g_BrakeLimitCount = 1;
	}
	MINE_LOG(SANY_LOG_INFO, "BrakeStartLimit, g_BrakeLimitCount %d, g_u8ResponseCtrlState %d, iBrakeOPening %d\n", g_BrakeLimitCount, g_u8ResponseCtrlState, struWireCtrlIn.iBrakeOpening);
	return RET_ERROR;
}

/*初始化函数列表数组*/
void Protocol_Cockpit::InitFuncArray_Cockpit()
{
	/*初始化函数列表,新增协议则在此处新增列表即可*/
	Protocol_Cockpit_Recv_Func_st a[PROTOCOL_COCKPIT_MAX_RECV_FUN_NUM] = {
		{EV_CP_REQ_CTRL, EV_WP_CMD, &Protocol_Cockpit::ProtoCockpit_RecvCmdRepCtrl},
		{EV_CP_REALTIME_CTRL, EV_WP_CMD, &Protocol_Cockpit::ProtoCockpit_RecvCmdRealtimeCtrl},
		{EV_CP_HEART_BEAT, EV_WP_CMD, &Protocol_Cockpit::ProtoCockpit_RecvCmdHeartBeat},
		{EV_CP_SEND_EMERGENCY_STOP, EV_WP_CMD, &Protocol_Cockpit::ProtoCockpit_RecvCmdEmergencyStop},
		{EV_CP_VEHICLE_SIGNOUT, EV_WP_CMD, &Protocol_Cockpit::ProtoCockpit_RecvCmdLoginOut},

		{EV_CP_VEHICLE_REGISTER, EV_WP_ACK, &Protocol_Cockpit::ProtoCockpit_RecvAckLoginPacket},

	};

	memcpy(arrRecvFunc, a, sizeof(Protocol_Cockpit_Recv_Func_st) * PROTOCOL_COCKPIT_MAX_RECV_FUN_NUM);
}

/*******************************************************************************
* 函数名称:      GetRecvProc_Cockpit
* 描述:        根据协议命令获取处理函数
* parma in:  u8 u8Cmd    协议命令字
			 u8 u8Ack    协议响应表示
* 返回值:       NULL/Protocol_Cockpit::*函数指针
*
********************************************************************************/
SANY_U8(Protocol_Cockpit::*Protocol_Cockpit::GetRecvProc_Cockpit(SANY_U8 u8Cmd, SANY_U8 u8Ack))
(Protocol_Cockpit_Buffer_st *)
{

	SANY_U8 i = 0;
	for (i = 0; i < PROTOCOL_COCKPIT_MAX_RECV_FUN_NUM; i++)
	{
		if (arrRecvFunc[i].Cmd == u8Cmd && arrRecvFunc[i].Ack == u8Ack)
		{
			MINE_LOG(SANY_LOG_INFO, "arrRecvFunc[i]: %d, arrRecvFunc[i].ACK: %d\n", u8Cmd, u8Ack);
			return arrRecvFunc[i].pFun;
		}
	}

	return NULL;
}
