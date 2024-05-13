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

#include "./protoDef.h"
#include "protocol_cockpit.h"
#include <time.h>
#include <sys/time.h>
#include "sany_utils.h"


extern Protocol_Cockpit *pCockpit;
extern SANY_U8 g_IpFlag;
extern SANY_U8 g_u8CtrlState;

void wireFeedbackCallback(WirectlFeedbackInfo *info)
{
	
	// memcpy(&pCockpit->OldWireInfo, &pCockpit->WireInfo, sizeof(WirectlFeedbackInfo));
	memcpy(&pCockpit->WireInfo, info, sizeof(WirectlFeedbackInfo));

	return;
}

void gpsFeedbackCallback(GpsImuInfo *info)
{
	memcpy(&pCockpit->struGpsInfo, info, sizeof(GpsImuInfo));
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
	/* 初始化变量 */
	g_u8LoginFlag = 0;
//	u8LoginoutFlag = 0;
	g_bNetReady = 0;
	g_u32RTDataFlg = 0;
	g_u32NLDataFlg = 0;
	g_u8LoginSeqNum = 0;
	g_u8CtrlState = 0;
	g_u8TaskStopFlag = 0;
	g_u8EmergencyStopFlg = 0;

	bzero(&struWireCtrlIn, sizeof(struWireCtrlIn));
	bzero(&struWireCtrlOut, sizeof(struWireCtrlOut));
	bzero(&WireInfo, sizeof(WireInfo));
	bzero(&struGpsInfo, sizeof(struGpsInfo));

	pSubWire = new ddsSubscriber<WirectlFeedbackInfo>(pDataTypeWire, &wireFeedbackCallback);
	pSubGps =  new ddsSubscriber<GpsImuInfo>(pDataTypeGps, &gpsFeedbackCallback);
}
/*
void Protocol_Cockpit::Read_INI(ini::iniReader config)
{
	bool ret = config.ReadConfig("../config/remote.ini");
	if(ret == true)
	{
		iccid = config.ReadString("Vehicle", "ICCID", "");
		vin = config.ReadString("Vehicle", "VIN", "");
		HardVer = config.ReadString("Vehicle", "hardware_version", "");
		SoftVer = config.ReadString("Vehicle", "software_version", "");
		type = config.ReadString("Vehicle", "vehicle_type", "");
	}
	else
	{
		MINE_LOG(SANY_LOG_ERROR, "Read Config-remote.ini fail, %d\n", ret);
	}
	
	return;
}
*/
void Protocol_Cockpit::Read_Config()
{
	YAML::Node config = YAML::LoadFile("../../../../config/runSetup.yaml");
	if(! config)
	{
		MINE_LOG(SANY_LOG_ERROR, "Read Config-runSetup.yaml fail!!!");
	}
	else
	{
		vin = config["vehicle"]["vehicle_VIN"].as<std::string>();
		iccid = config["vehicle"]["vehicle_ICCID"].as<std::string>();
		v_type = config["vehicle"]["vehicle_type"].as<int>();
	//	brake_delay = config["vehicle"]["vehicle_delay"].as<int>();
	//	read_brakeing = config["vehicle"]["vehicle_brake"].as<int>();
		if(v_type == 0)
		{
			MINE_LOG(SANY_LOG_ERROR, "Read Config-runSetup.yaml vehicle_type is 0 !!!");
		}
		else if((v_type % 2) != 0)
		{
			type = "SKT-I";
		}
		else
		{
			type = "SKT-E";
		}
		MINE_LOG(SANY_LOG_INFO, "Device VIN is %s\n", vin.c_str());
		MINE_LOG(SANY_LOG_INFO, "Device ICCID is %s\n", iccid.c_str());
		MINE_LOG(SANY_LOG_INFO, "Device type is %s\n", type.c_str());
	}
}


/*
SANY_U8 Protocol_Cockpit::ProtoCockpit_getMaxSpeed(Remote2Control *pRemoteCtrl)
{
	pRemoteCtrl->MaxRemoteVehSpd(max_speed);
	pRemoteCtrl->RemoteIn(g_u8CtrlState);
}
*/
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
	memcpy((SANY_S8 *)pst_DataBuf->ucVIN, vin.c_str(), WP_VIN_SIZE); // #define VIN "SKT90202011300002"
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
	memcpy(pu8Iccid, iccid.c_str(), 20);
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
		m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 8, 1);
		MINE_LOG(SANY_LOG_ERROR, "g_u8LoginSeqNum %d dataST->ucSequenceNum[0] %d \n", g_u8LoginSeqNum, (SANY_U8)dataST->ucSequenceNum[0]);
	//	return RET_ERROR;
	}

	/*登入状态全0,则登录完成，否则报错处理*/
	if (0 != dataST->ucLoginStatus[0])
	{
		MINE_LOG(SANY_LOG_ERROR, "dataST->ucLoginStatus %d \n", dataST->ucLoginStatus[0]);
		// ICCID不匹配
		if (0x00 != ((dataST->ucLoginStatus[0] & 0x02) << 2))
		{
			m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 9, 1);
		}
		// 硬件类型不匹配
		if (0x00 != ((dataST->ucLoginStatus[0] & 0x04) << 3))
		{
			m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 10, 1);
		}
		// 硬件版本号不匹配
		if (0x00 != ((dataST->ucLoginStatus[0] & 0x08) << 4))
		{
			m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 11, 1);
		}
		// 软件类型不匹配
		if (0x00 != (dataST->ucLoginStatus[0] & 0x10))
		{
			m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 12, 1);
		}
		// 软件版本号不匹配
		if (0x00 != (dataST->ucLoginStatus[0] & 0x20))
		{
			m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 13, 1);
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
	g_u32RTDataFlg = 1;
	/* 开启状态变化上报 */
	g_u32NLDataFlg = 1;
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

	g_IpFlag = 0;
	/*修改登出标志位*/
	// g_u8LoginFlag = 0;
	m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 15, 0);
	m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 16, 0);
	m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3203, 5, 0);	
	m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3204, 1, 0);
	m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3204, 3, 0);
	MINE_LOG(SANY_LOG_INFO, "report failure clear,! \n");
	/*关闭Socket，释放线程资源*/
	stop();

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
 * 描述:        组装bei动登出响应报文
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

	memcpy(dataST->ucHardVer, HardVer.c_str(), 4);
	memcpy(dataST->ucSoftVer, SoftVer.c_str(), 4);
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
	g_old_current_state = g_current_state;
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

	MINE_LOG(SANY_LOG_INFO, "vcu status: %d, remote_or_unman: %d\n, ", WireInfo.vcu_state(), WireInfo.Remote_or_Unman());
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
	u16TmpDate = (int)struWireCtrlOut.iVehicle_speed * 10;					
	SYS2NET(pDataST->ucCarSpeed, u16TmpDate);
	/*航向角*/
	u16TmpDate = (int)struWireCtrlOut.iHeadingAngle * 10;
	SYS2NET(pDataST->ucHeadingAngle, u16TmpDate);
	/*车轮转角*/
	u16TmpDate = (int)(struWireCtrlOut.iWheelAngleL + 40) * 10;
	SYS2NET(pDataST->ucWheelAngleL, u16TmpDate);
	/*油门*/
	pDataST->ucThrottleOpening[0] = (SANY_U8)struWireCtrlOut.iThrottleOpening;
	/*制动*/
	pDataST->ucBrakeOpening[0] = (SANY_U8)struWireCtrlOut.iBrakeOpening;
	/*发动机转速*/
	u16TmpDate = (int)struWireCtrlOut.iEngineSpeed / 0.125;
	SYS2NET(pDataST->ucEngineSpeed, u16TmpDate);
	/*累计里程*/
	u32TmpDate = (int)struWireCtrlOut.iAccumulatedMileage * 10;
	*(SANY_U32 *)&(pDataST->ucAccumulatedMileage) = SYS2NET4((SANY_U8 *)&u32TmpDate);
	/*小计里程*/
	u16TmpDate = (int)struWireCtrlOut.iSubtotalMileage * 10;
	SYS2NET(pDataST->ucSubtotalMileage, u16TmpDate);
	/*俯仰角*/
	u16TmpDate = (int)(struWireCtrlOut.iPitchAngle + 90);
	SYS2NET(pDataST->ucPitchAngle, u16TmpDate);
	/*档位*/
	pDataST->ucGearOper[0] = (SANY_U8)struWireCtrlOut.iGearOpening;
	/*剩余油量*/
	pDataST->ucRemainingOil[0] = (SANY_U8)struWireCtrlOut.iRemainingOil;
	/*水温*/
	pDataST->ucWaterTemp[0] = (SANY_U8)struWireCtrlOut.iWaterTemp;
	/*气压1\2*/
	pDataST->ucBrakeAirPre1[0] = (SANY_U8)struWireCtrlOut.iBrakeAirPre1;
	pDataST->ucBrakeAirPre2[0] = (SANY_U8)struWireCtrlOut.iBrakeAirPre2;
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

	MINE_LOG(SANY_LOG_INFO, "--- ProtoCockpit_SendCmdRealtimeRep --- start\n");
	MINE_LOG(SANY_LOG_INFO, "iVehicle_speed %d\n", (int)struWireCtrlOut.iVehicle_speed * 10);
	MINE_LOG(SANY_LOG_INFO, "iBrakeOpening %d\n", (int)struWireCtrlOut.iBrakeOpening);
	MINE_LOG(SANY_LOG_INFO, "iWheelAngleL %d\n", (int)(struWireCtrlOut.iWheelAngleL + 40) * 10);
	MINE_LOG(SANY_LOG_INFO, "iThrottleOpening %d\n", (int)struWireCtrlOut.iThrottleOpening);
	MINE_LOG(SANY_LOG_INFO, "iEngineSpeed %d\n", (int)(struWireCtrlOut.iEngineSpeed / 0.125));				//
	MINE_LOG(SANY_LOG_INFO, "iHeadingAngle %d\n", (int)struWireCtrlOut.iHeadingAngle * 10);
	MINE_LOG(SANY_LOG_INFO, "iAccumulatedMileage %d\n", (int)struWireCtrlOut.iAccumulatedMileage * 10);
	MINE_LOG(SANY_LOG_INFO, "iSubtotalMileage %d\n", (int)struWireCtrlOut.iSubtotalMileage * 10);
	MINE_LOG(SANY_LOG_INFO, "iPitchAngle %d\n", (int)(struWireCtrlOut.iPitchAngle + 90));
	MINE_LOG(SANY_LOG_INFO, "iGearOpening %d\n", (int)(struWireCtrlOut.iGearOpening));					//
	MINE_LOG(SANY_LOG_INFO, "iRemainingOil %d\n", (int)(struWireCtrlOut.iRemainingOil));				//
	MINE_LOG(SANY_LOG_INFO, "iWaterTemp %d\n", (int)(struWireCtrlOut.iWaterTemp));					//
	MINE_LOG(SANY_LOG_INFO, "iBrakeAirPre1 %d\n", (int)(struWireCtrlOut.iBrakeAirPre1));				//    
	MINE_LOG(SANY_LOG_INFO, "iBrakeAirPre2 %d\n", (int)(struWireCtrlOut.iBrakeAirPre2));
	MINE_LOG(SANY_LOG_INFO, "iCombinedGearL %d\n", (int)(struWireCtrlOut.iCombinedGearL));
	MINE_LOG(SANY_LOG_INFO, "iCombinedGearR %d\n", (int)(struWireCtrlOut.iCombinedGearR));
	MINE_LOG(SANY_LOG_INFO, "iKeyOperation1 %d\n", (int)(struWireCtrlOut.iKeyOperation1));
	MINE_LOG(SANY_LOG_INFO, "iKeyOperation2 %d\n", (int)(struWireCtrlOut.iKeyOperation2));
	MINE_LOG(SANY_LOG_INFO, "iWorkStatus %d\n", (int)(struWireCtrlOut.iWorkStatus));
	MINE_LOG(SANY_LOG_INFO, "iLiftUpStatus %d\n", (int)(struWireCtrlOut.iLiftUpStatus));
	MINE_LOG(SANY_LOG_INFO, "iLiftDownStatus %d\n", (int)(struWireCtrlOut.iLiftDownStatus));
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

	if(g_u8LoginFlag)
	{
		if(1 == g_u8CtrlState)
		{
			if (500  <= Losttimes)
			{
				if(type == "SKT-I")
				{
					struWireCtrlIn.iGearOpening = 0x7D;
					struWireCtrlIn.iBrakeOpening = 100;
					struWireCtrlIn.iThrottleOpening = 0;
					struWireCtrlIn.iWheelAngle = 0x0190;
				}
				else if(type == "SKT-E")
				{
					struWireCtrlIn.iGearOpening = 0;
					struWireCtrlIn.iBrakeOpening = 100;
					struWireCtrlIn.iThrottleOpening = 0;
					struWireCtrlIn.iWheelAngle = 0x0190;
				}
				struWireCtrlIn.iKeyOperation2 = 0x08;	
				m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3203, 5, 1);												
				m_Terminal.comm_socket_client_do_disconnect(m_Socket_CockpitClient);	
				MINE_LOG(SANY_LOG_ERROR, "DownloadData timeout, report failure, u8LostCtrlTimes: %d, b_connected: %d\n", Losttimes, m_Socket_CockpitClient->b_connected);		
			}	
			u8Ret = RET_ERROR;
		}
		else
		{
			if(5 * 1000  <= Losttimes)
			{
				m_Terminal.comm_socket_client_do_disconnect(m_Socket_CockpitClient);	
				MINE_LOG(SANY_LOG_ERROR, "Lost HeartBeat 5s, do disconnect!!!!  u8LostCtrlTimes: %d, b_connected: %d\n", Losttimes, m_Socket_CockpitClient->b_connected);		
			}
			u8Ret = RET_ERROR;
		}
	}	
	return u8Ret;
}

SANY_U8 Protocol_Cockpit::ProtoCockpit_DoStop()
{
	
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
	SANY_U8 u8Ret = RET_OK;
	static SANY_U32 u32Tick = 0; // 发送分频器
//	static SANY_U32 u8LostCtrlTimes = 0;
	SANY_U8 LoginTimes = 0;
//	SANY_U8 LoginoutTimes = 0;

	while (!g_u8TaskStopFlag)
	{
		if (m_Socket_CockpitClient->b_connected)
		{
			/* 判断登入状态 */
			if (0 == g_u8LoginFlag)
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
						stop();
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
			/*已经登录完成，则进行需要上报的数据协议组装*/
			else if(g_u8SendBufferFull == 0)
			{
				LoginTimes = 0;
				/*100ms上报一次*/
				if(0 == (u32Tick % SEND_RTDATA_UPLOAD_TICK))
				{
					/*实时上报*/
					// if(g_u32RTDataFlg )
					// {
						u8Ret = ProtoCockpit_SendCmdRealtimeRep();
						if (RET_OK != u8Ret)
						{
							MINE_LOG(SANY_LOG_ERROR, "ProtoCockpit_SendCmdRealtimeRep err!!\n");
							u32Tick++;
							continue;
						}
					// }					
				}		
			}
			else
			{
				MINE_LOG(SANY_LOG_ERROR, "SendBuffer is ful, g_u8SendBufferFull is %d\n",g_u8SendBufferFull);
			}
			u32Tick++;
		}
		usleep(1000); // 1ms
	}
}

/*******************************************************************************
threadAppDownloadData 下行处理业务线程，做协议包解析或响应
********************************************************************************/
void Protocol_Cockpit::funcAppDownloadData()
{
	SANY_U8 u8Ret = RET_OK;
	SANY_U32 u32DataLen = 0;
	SANY_U8 szRecvBuffer[PROTO_PACK_MAX_LEN] = {0};
	static SANY_U32 u8LostCtrlTimes = 0;
	// static SANY_U32 u8LostHeartTimes = 0;
	SANY_U32 u32NetStatus = 0;
	SANY_U8 u8ConnectTimes = 0;

	while (!g_u8TaskStopFlag)
	{
		if (m_Socket_CockpitClient->b_connected)
		{
			/* 发送数据 */
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
		}
		else
		{
			/* 断开连接后进行重连 */
			g_u8LoginFlag = 0; // 修改登入状态
			//g_u8CtrlState = 0;	//
			/* 定时器到时后尝试重连 */
			MINE_LOG(SANY_LOG_ERROR, "Reconnect TCP, current login flag: %d\n", g_u8LoginFlag);
			if(19 < u8ConnectTimes)
			{
				MINE_LOG(SANY_LOG_ERROR, " reconnect 10 seconds, exit!!! \n");
				m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 3, 1);
				stop();
			}
			else
			{
				u32NetStatus = m_Terminal.comm_socket_client_do_connect(m_Socket_CockpitClient);
				MINE_LOG(SANY_LOG_ERROR, "comm_socket_client_do_connect, u32NetStatus: %d\n", u32NetStatus);
				if (1 != u32NetStatus)
				{
					ProtoCockpit_initCtrlData();
					u8ConnectTimes++;
					MINE_LOG(SANY_LOG_ERROR, "sa_comm_socket_client_send fail, u8ConnectTimes: %d\n", u8ConnectTimes);					
					/*延时500ms重连*/
					usleep(500 * 1000);
					continue;
				}
				/* 网络连接成功，更新网络状态 */
				g_bNetReady = 1;
				u8ConnectTimes = 0;
				MINE_LOG(SANY_LOG_ERROR, "Reconnect TCP Success!!! m_Socket_CockpitClient->b_connected: %d, clear u8ConnectTimes: %d\n", m_Socket_CockpitClient->b_connected, u8ConnectTimes);
			}
		}
		usleep(1000);
	}
}

/*******************************************************************************
threadSendData 数据发送线程，检测环形缓冲区中有无数据，有数据则立即发送
********************************************************************************/
void Protocol_Cockpit::funcSendData()
{
	SANY_U8 u8Ret = RET_OK;
	SANY_U32 u32SendLen = 0;
	SANY_U8 szSendBuffer[PROTO_PACK_MAX_LEN] = {0};

	while (!g_u8TaskStopFlag)
	{
		if (m_Socket_CockpitClient->b_connected)
		{
			memset(szSendBuffer, 0, sizeof(szSendBuffer));

			u8Ret = ProtoCockpit_ReadSendBuffer(szSendBuffer, &u32SendLen);
			if (RET_OK != u8Ret || 0 >= u32SendLen)
			{
			//	MINE_LOG(SANY_LOG_ERROR, "ProtoCockpit_ReadSendBuffer error !!\n");
				continue;
			}
			if (!m_Terminal.sa_comm_socket_client_send(m_Socket_CockpitClient, (char *)szSendBuffer, (int)u32SendLen))
			{
				MINE_LOG(SANY_LOG_ERROR, "sa_comm_socket_client_send err, u32SendLen: %d,  g_bNetReady set 0 !!\n", u32SendLen);
				/*发送失败，复位网络状态*/
				g_bNetReady = 0;
				continue;
			}
		}
		usleep(1000);
	}
	MINE_LOG(SANY_LOG_ERROR, "funcSendData g_u8TaskStopFlag\n");
}

/*******************************************************************************
threadSendData 数据接收线程，如有网络数据则将数据存储到接收缓冲区等待处理
********************************************************************************/
void Protocol_Cockpit::funcRecvData()
{
	SANY_U32 u32ReadLen = 0;
	SANY_U32 u32DateLen = 0;
	fd_set fdR;
	struct timeval timeout;
	CP_Header_st *pHeadST = NULL;
	SANY_U8 u8HaveRecvHead = 0;

	while (!g_u8TaskStopFlag)
	{
		u32ReadLen = 0;
		if (m_Socket_CockpitClient->b_connected)
		{
			timeout.tv_sec = 0;
			timeout.tv_usec = 100;

			FD_ZERO(&fdR);
			FD_SET(m_Socket_CockpitClient->socket_fd, &fdR);

			switch (select(m_Socket_CockpitClient->socket_fd + 1, &fdR, NULL, NULL, &timeout))
			{
			case -1:
				break;
			case 0:
				break;
			default:
			{
				if (FD_ISSET(m_Socket_CockpitClient->socket_fd, &fdR))
				{
					/*先接收包头数据，获取数据长度*/
					if (!u8HaveRecvHead)
					{
						bzero(m_Socket_CockpitClient->recv_buffer, sizeof(m_Socket_CockpitClient->recv_buffer));
						u32ReadLen = m_Terminal.m_Sa_Comm.sa_comm_socket_read(m_Socket_CockpitClient->socket_fd,
																			m_Socket_CockpitClient->recv_buffer,
																			sizeof(m_Socket_CockpitClient->recv_buffer),
																			sizeof(CP_Header_st));
						if (sizeof(CP_Header_st) != u32ReadLen)
						{					
							continue;
						}
						else
						{
							/*校验包头并获取数据长度,目前先写死成 sizeof(CP_Header_st)*/
							pHeadST = (CP_Header_st *)m_Socket_CockpitClient->recv_buffer;
							/*数据区长度 + 校验位*/
							u32DateLen = (pHeadST->ucDataByteSize[1] * 256 + pHeadST->ucDataByteSize[0]) + 1;
							if (1 >= u32DateLen)
							{
								/* 长度异常 */
								// 舱端下发的报文数据为空
								m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3204, 1, 1);
								// 通用回复
								ProtoCockpit_SendAckCom(pHeadST->ucCommand[0], 0x2);
								// sany_log_set_output(logHandleLocal, SANY_LOG_OUT_BOTH);
								MINE_LOG(SANY_LOG_ERROR, "Message len %d empty\n", u32DateLen);
							}

							if ((Message_Max_Len - sizeof(CP_Header_st)) < u32DateLen)
							{
								MINE_LOG(SANY_LOG_ERROR, "Message len %d out of range\n", u32DateLen);
								u32DateLen = 0;
								continue;
							}

							MINE_LOG(SANY_LOG_INFO, "funcRecvData ucDataByteSize %x %x u32DateLen %d\n", pHeadST->ucDataByteSize[0], pHeadST->ucDataByteSize[1], u32DateLen);
						}
						u8HaveRecvHead = 1;
					}
					else
					{
						u32ReadLen = m_Terminal.m_Sa_Comm.sa_comm_socket_read(m_Socket_CockpitClient->socket_fd,
																			m_Socket_CockpitClient->recv_buffer + sizeof(CP_Header_st),
																			sizeof(m_Socket_CockpitClient->recv_buffer) - sizeof(CP_Header_st),
																			u32DateLen);

						if ((u32ReadLen != u32DateLen) || (u32DateLen == 1))
						{
							u8HaveRecvHead = 0;						
							continue;
						}
						else
						{
							ProtoCockpit_WriteRecvBuffer((SANY_U8 *)m_Socket_CockpitClient->recv_buffer, sizeof(CP_Header_st) + u32DateLen);
							MINE_LOG(SANY_LOG_INFO, "receive data, write to recv buffer!!!!!!!!!!!\n");
						}
						u8HaveRecvHead = 0;
						u32DateLen = 0;
					}
				}
			}
			}
		}
		usleep(1000);
	}
}

void Protocol_Cockpit::InitSocket_Cockpit()
{
	m_Socket_CockpitClient = m_Terminal.sa_comm_socket_client_new((const char *)cockpitServerName, (const char *)HostclientName, false);

	/*死等网络连接成功*/
	SANY_U8 g_count = 0;
	while (!m_Terminal.comm_socket_client_do_connect(m_Socket_CockpitClient))
	{
		// sany_log_set_output(logHandleLocal, SANY_LOG_OUT_BOTH);
		MINE_LOG(SANY_LOG_ERROR, "Connect server fail\n");
		if(g_count > 10)
		{
			MINE_LOG(SANY_LOG_ERROR, "Connect server fail 10 times, exit(0)!!!!!!!\n");
			m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 3, 1);
			stop();
		}
		g_count++;
		sleep(1);
	}
	g_count = 0;
	m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 3, 0);
	g_bNetReady = 1;
	
	/*连接失败后不处理，接收线程自行做重连*/
	if (m_Terminal.comm_socket_client_do_connect(m_Socket_CockpitClient))
	{
		g_bNetReady = 1;
		printf("Connect server success\n");
	}

}

/*数据接收线程*/
void *Protocol_Cockpit::threadRecvData(void *args)
{
	Protocol_Cockpit *pThread = static_cast<Protocol_Cockpit *>(args);
	pThread->funcRecvData();
}
/*数据发送线程*/
void *Protocol_Cockpit::threadSendData(void *args)
{
	Protocol_Cockpit *pThread = static_cast<Protocol_Cockpit *>(args);
	pThread->funcSendData();
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

/*模块业务开始接口，做收发缓冲区初始化，网络初始化，收发业务线程启动*/
void Protocol_Cockpit::start()
{
//	Read_INI(config);
	Read_Config();
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
	
	g_u8TaskStopFlag = 0;
	/* trigger init */
	ProtoCockpit_SendBufferInit();
	ProtoCockpit_RecvBufferInit();
	InitSocket_Cockpit();
	InitFuncArray_Cockpit();

	if (pthread_create(&t_upload_data_id, NULL, threadAppUploadData, this) == -1)
	{
		MINE_LOG(SANY_LOG_ERROR, "create threadAppUploadData thread error!\n");
	}

	if (pthread_create(&t_download_data_id, NULL, threadAppDownloadData, this) == -1)
	{
		MINE_LOG(SANY_LOG_ERROR, "create threadAppDownloadData thread error!\n");
	}

	if (pthread_create(&t_recv_id, NULL, threadRecvData, this) == -1)
	{
		MINE_LOG(SANY_LOG_ERROR, "create threadRecvData thread error!\n");
	}

	if (pthread_create(&t_send_id, NULL, threadSendData, this) == -1)
	{
		MINE_LOG(SANY_LOG_ERROR, "create threadSendData thread error!\n");
	}
}

/*模块业务关闭接口，做资源的反初始化*/
void Protocol_Cockpit::stop()
{
	if(g_u8CtrlState == 1 )
	{
		g_u8CtrlState = 2;
	}
	g_u8TaskStopFlag = 1;
	MINE_LOG(SANY_LOG_ERROR, "STOP, g_u8CtrlState is  %d,  g_u8TaskStopFlag is  %d\n", g_u8CtrlState, g_u8TaskStopFlag);
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
	/*延时2S等待，线程退出*/
	sleep(2);

	/*断开网络连接*/
	m_Terminal.comm_socket_client_do_disconnect(m_Socket_CockpitClient);
	m_Terminal.sa_comm_socket_client_release(m_Socket_CockpitClient);
	MINE_LOG(SANY_LOG_ERROR, "STOP, sa_comm_socket_client_release!!!\n");

	/*等待线程退出*/
	pthread_join(t_upload_data_id, NULL);
	pthread_join(t_download_data_id, NULL);
	pthread_join(t_recv_id, NULL);
	pthread_join(t_send_id, NULL);
	MINE_LOG(SANY_LOG_ERROR, "STOP, wait for pthread_join!!!\n");

	/*释放动态申请的内存空间*/
	free(arrSendBuffer);
	free(arrRecvBuffer);
	MINE_LOG(SANY_LOG_ERROR, "STOP,  free up space!!!\n");

	/*销毁互斥锁*/
	pthread_mutex_destroy(&SendBufferMutex);
	pthread_mutex_destroy(&RecvBufferMutex);
	MINE_LOG(SANY_LOG_ERROR, "STOP, destroy pthread mutex!!!\n");

	exit(0);
}

/*******************************************************************************
 * 函数名称:      ProtoCockpit_setServerName
 * 描述:        设置驾舱服务端地址
 * parma in:  char * pServerName	驾舱IP+端口号  例："192.168.8.76:8080"
 * 返回值:       RET_OK/RET_ERROR
 *
 ********************************************************************************/
void Protocol_Cockpit::ProtoCockpit_setServerName(const char *pServerName)
{
	if (!pServerName)
	{
		return;
	}
	memcpy(cockpitServerName, pServerName, MIN(strlen(pServerName), sizeof(cockpitServerName)));
	MINE_LOG(SANY_LOG_INFO, "ProtoCockpit_setServerName cockpitServerName %s\n", cockpitServerName);

	return;
}

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
	struWireCtrlOut.iWheelAngleL = WireInfo.steering_left_angle_info();		 // 车轮转角
	struWireCtrlOut.iAccumulatedMileage = WireInfo.total_vehicle_distance(); // 总计里程
	struWireCtrlOut.iPitchAngle = struGpsInfo.gps_pitch();					//俯仰角
	struWireCtrlOut.iLiftUpStatus = WireInfo.lift_longo_position_status();			//举升到位状态
	struWireCtrlOut.iLiftDownStatus = WireInfo.drop_longo_position_status();				//下降到位状态
	struWireCtrlOut.iGearOpening =  WireInfo.trans_current_gear();				//档位
	struWireCtrlOut.iRemainingOil = WireInfo.fuel_level();					//剩余油量
	struWireCtrlOut.iWaterTemp = WireInfo.engine_coolant_temperature();				//水温
	struWireCtrlOut.iBrakeAirPre1 = WireInfo.air_pressure_1();							//气压1
	struWireCtrlOut.iBrakeAirPre2 = WireInfo.air_pressure_2();							//气压2

	if (WireInfo.left_turn_start_status())
	{
		struWireCtrlOut.iCombinedGearL |= 0x01;
	}
	if (WireInfo.right_turn_start_status())
	{
		struWireCtrlOut.iCombinedGearL |= 0x01 << 1;
	}
	if (WireInfo.low_light_start_status())
	{
		struWireCtrlOut.iCombinedGearL |= 0x01 << 2;
	}
	if (WireInfo.high_beam_start_status())
	{
		struWireCtrlOut.iCombinedGearL |= 0x01 << 3;
	}
	if (WireInfo.outline_lamp_start_status())
	{
		struWireCtrlOut.iCombinedGearL |= 0x01 << 5;
	}

	if (WireInfo.wiper_start_status())
	{
		struWireCtrlOut.iCombinedGearR |= 0x01;
	}
	if (WireInfo.wiper_slow_start_status())
	{
		struWireCtrlOut.iCombinedGearR |= 0x01 << 1;
	}
	if (WireInfo.horn_start_status())
	{
		struWireCtrlOut.iCombinedGearR |= 0x01 << 2;
	}
	if(g_u8EmergencyStopFlg == 1)												//急停状态上报
	{
		struWireCtrlOut.iCombinedGearR |= 0x01 << 3;
	}
	if(rolling_need_stop == 1)														//溜车状态上报
	{
		struWireCtrlOut.iCombinedGearR |= 0x01 << 4;
	}

	if (WireInfo.hazard_start_status())
	{
		struWireCtrlOut.iKeyOperation1 |= 0x01;
	}
	if (WireInfo.rear_light_start_status())
	{
		struWireCtrlOut.iKeyOperation1 |= 0x01 << 2;
	}
	if (WireInfo.fog_lamp_start_status())
	{
		struWireCtrlOut.iKeyOperation1 |= 0x01 << 3;
	}
	if (WireInfo.head_light_status())
	{
		struWireCtrlOut.iKeyOperation1 |= 0x01 << 4;
	}
	if (WireInfo.exhaust_brake_status())
	{
		struWireCtrlOut.iKeyOperation1 |= 0x01 << 5;
	}
	if (WireInfo.axle_differential())
	{
		struWireCtrlOut.iKeyOperation1 |= 0x01 << 6;
	}
	if (1 == WireInfo.lift_condition())
	{
		struWireCtrlOut.iKeyOperation1 |= 0x01 << 7;
	}

	if (WireInfo.flash_start_status())
	{
		struWireCtrlOut.iKeyOperation2 |= 0x01 << 1;
	}
	if (WireInfo.parking())
	{
		struWireCtrlOut.iKeyOperation2 |= 0x01 << 3;
	}
	if (2 == WireInfo.lift_condition())
	{
		struWireCtrlOut.iKeyOperation2 |= 0x01 << 7;
	}

	//车辆驾驶状态 
	if((WireInfo.vcu_state() == 5) && (WireInfo.Remote_or_Unman() == 8))
	{
		struWireCtrlOut.iWorkStatus = 2;			//remote
	}
	else if((WireInfo.vcu_state() == 5) && (WireInfo.Remote_or_Unman() == 6))
	{
		struWireCtrlOut.iWorkStatus = 3;			//unman
	}
	else
	{
		struWireCtrlOut.iWorkStatus = 1;			//manual
		g_u8CtrlState = 0;
	}

	if (type == "SKT-I")
	 {
		struWireCtrlOut.iThrottleOpening = WireInfo.accel_pedal_position(); // 油门
		struWireCtrlOut.iEngineSpeed = WireInfo.engine_speed();	 // 发动机转速 SKT-I
		struWireCtrlOut.iHeadingAngle = struGpsInfo.gps_heading();					 // 航向角
		if (finitTalDistance == 0)
		{
			finitTalDistance = WireInfo.total_vehicle_distance();
		}
		/*小计里程 = 总里程-初始里程*/
		struWireCtrlOut.iSubtotalMileage = (WireInfo.total_vehicle_distance() - finitTalDistance);
	 }
	else if(type == "SKT-E")
	 {
		struWireCtrlOut.iThrottleOpening = WireInfo.actual_engine_torque(); // 油门
		struWireCtrlOut.iEngineSpeed = WireInfo.main_driveMotor_speed_raw();			// 发动机转速
		struWireCtrlOut.iHeadingAngle = struGpsInfo.gps_yaw();							// 航向角
		if (finitTalDistance == 0)
		{
			finitTalDistance = WireInfo.total_vehicle_distance();
		}
		/*小计里程 = 总里程-初始里程*/
		struWireCtrlOut.iSubtotalMileage = (WireInfo.total_vehicle_distance() - finitTalDistance) / 1000;
	 }

	return RET_OK;
}

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
	
	if ( (fabs(struGpsInfo.gps_vn()) > 0.1 || fabs(struGpsInfo.gps_ve()) > 0.1) && (WireInfo.trans_current_gear() == -1))
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
            rolling_need_stop = 1;
            MINE_LOG(SANY_LOG_ERROR, "rolling_need_stop : %d\n", rolling_need_stop);
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
		rolling_need_stop = 0;
    }
	return rolling_need_stop;
}



SANY_U8 Protocol_Cockpit::ProtoCockpit_initCtrlData()
{
	if(type == "SKT-E")
	{
		struWireCtrlIn.iGearOpening = 0;
		struWireCtrlIn.iBrakeOpening = 100;
	}
	else
	{
		struWireCtrlIn.iGearOpening = 0x7D;
		struWireCtrlIn.iBrakeOpening = 100;
	}
	struWireCtrlIn.iWheelAngle =  0x0190;
	struWireCtrlIn.iThrottleOpening = 0;
	struWireCtrlIn.iCombinedGearL = 0;
	struWireCtrlIn.iCombinedGearR = 0;
	struWireCtrlIn.iKeyOperation1 = 0;
	// struWireCtrlIn.iKeyOperation2 = 0x20;
	BrakeValue = 0;
	struWireCtrlIn.iKeyOperation2 = 0x08;		//住车

	return true;
}

SANY_U8 Protocol_Cockpit::ProtoCockpit_getCtrlDate(ImpleResultInfo *pCtrlInfo)
{
	if (!pCtrlInfo)
	{
		MINE_LOG(SANY_LOG_ERROR, "ProtoCockpit_getCtrlDate param invalid \n");

		return RET_ERROR;
	}
	bzero(pCtrlInfo, sizeof(ImpleResultInfo));

	if (g_u8CtrlState == 1)
	{
		pCtrlInfo->Remote_enable(3);
		pCtrlInfo->unmanned_enable(1);
	}
	else if(g_u8CtrlState == 2)
	{
		pCtrlInfo->Remote_enable(5);
		pCtrlInfo->unmanned_enable(1);
		ProtoCockpit_initCtrlData();				
	}
	else if(g_u8CtrlState == 3)
	{
		pCtrlInfo->Remote_enable(5);
		pCtrlInfo->unmanned_enable(0);
		ProtoCockpit_initCtrlData();			
	}

	pCtrlInfo->vilid_count(g_count);
	if (g_count < 255)
	{
		g_count++;
	}
	else
	{
		g_count = 0;
	}

	pCtrlInfo->engine_override_ctrl_mode(0b10);
	pCtrlInfo->engine_req_speed_ctrl_condition(0b10);
	pCtrlInfo->override_ctrl_mode_priority(0b00);
	pCtrlInfo->engine_req_speed(2100); // rpm

	/*-------------------油门，0-100-------------------*/
	pCtrlInfo->engine_req_torque((float)struWireCtrlIn.iThrottleOpening);
	printf("engine_req_torque: %f\n", pCtrlInfo->engine_req_torque());
	/*-------------------车轮转角，-40  -40--------------------*/
	pCtrlInfo->steering_angle_req((float)struWireCtrlIn.iWheelAngle * 0.1 - 40);
	printf("steering angle  req: %f\n", pCtrlInfo->steering_angle_req());
	/*-------------------档位，1-R, 3-D, 其他-N-------------------*/
	if(type == "SKT-I")
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
	else if(type == "SKT-E")
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
	
	if((struGpsInfo.gps_pitch() > 2))
	{
		MINE_LOG(SANY_LOG_INFO, "config brake_delay:  %d\n", brake_delay);
		if((struWireCtrlIn.iBrakeOpening == 0) && (BrakeValue > 0.1) && (brake_count < brake_delay) && (struWireCtrlIn.iThrottleOpening < 1))
		{
			pCtrlInfo->traffic_brake_req((float)read_brakeing);
		//	pCtrlInfo->traffic_brake_req((float)read_brakeing);
			brake_count ++;
			MINE_LOG(SANY_LOG_INFO, "brakeing delay 1 seconds, gps_pitch is : %f, brake_count: %d,  traffic_brake_req: %f\n", struWireCtrlOut.iPitchAngle, brake_count, pCtrlInfo->traffic_brake_req());
		}
		else if((struWireCtrlIn.iThrottleOpening > 10) && (struWireCtrlOut.iVehicle_speed < 1) && (WireInfo.trans_current_gear() != 0) && (WireInfo.trans_current_gear() != -1) && (brake_count < brake_delay))
		{
			pCtrlInfo->traffic_brake_req((float)read_brakeing );
		//	pCtrlInfo->traffic_brake_req((float)read_brakeing);
			brake_count ++;
			MINE_LOG(SANY_LOG_INFO, "brakeing delay, current speed : %f, brake_count: %d,  traffic_brake_req: %f, current gear: %d\n", struWireCtrlOut.iVehicle_speed, brake_count, pCtrlInfo->traffic_brake_req(), WireInfo.trans_current_gear());
		}
		else if(((struWireCtrlIn.iThrottleOpening <= 10) && (BrakeValue == 0)) || (struWireCtrlOut.iVehicle_speed >= 1))
		{
			BrakeValue = (float)struWireCtrlIn.iBrakeOpening;
			brake_count = 0;
			pCtrlInfo->traffic_brake_req((float)struWireCtrlIn.iBrakeOpening);
			MINE_LOG(SANY_LOG_INFO, "no brakeing delay ,clear brake_count\n");	
		}
		else 
		{
			pCtrlInfo->traffic_brake_req((float)struWireCtrlIn.iBrakeOpening);
			BrakeValue = (float)struWireCtrlIn.iBrakeOpening;
			MINE_LOG(SANY_LOG_INFO, "no brakeing delay !\n");	
		}	
	} 
	else
	{
		brake_count = 0;
		pCtrlInfo->traffic_brake_req((float)struWireCtrlIn.iBrakeOpening);
	}


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
	if (struWireCtrlIn.iKeyOperation1 & 0x20)
	{
		// 发动机排气制动开
		pCtrlInfo->exhaust_brake_req(true);
	}
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
	if(type == "SKT-I")
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
	else if(type == "SKT-E")
	{
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

	/*判断溜车*/
	if(ProtoCockpit_Rolling())
	{
		g_u8EmergencyStopFlg = 1;
	}
	
	/*判断是否急停，请求急停则请求驻车*/
	if (1 == g_u8EmergencyStopFlg)
	{
		if("SKT-I" == type)
		{
			pCtrlInfo->trans_req_gear(0x7D);		//N
			pCtrlInfo->traffic_brake_req(100);		//制动
		}
		else if ("SKT-E" == type)
		{
			pCtrlInfo->trans_req_gear(0);		//N
			pCtrlInfo->traffic_brake_req(100);		//制动
		}
		pCtrlInfo->parking_req(true);		//驻车
		pCtrlInfo->engine_req_torque(0);	//油门
		pCtrlInfo->brake_req_status(true);		//刹车请求状态
		pCtrlInfo->brake_light_ctrl(true);		//制动灯
		MINE_LOG(SANY_LOG_INFO, "receive EmergencyStop cmd, gear: %d,  parking_req: %d,  traffic_brake_req: %f\n", pCtrlInfo->trans_req_gear(), pCtrlInfo->parking_req(), pCtrlInfo->traffic_brake_req());
		// printf("start EmergencyStop, gear: %d,  parking_req: %d,  traffic_brake_req: %f\n", pCtrlInfo->trans_req_gear(), pCtrlInfo->parking_req(), pCtrlInfo->traffic_brake_req());
	}
	

	return RET_OK;
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
