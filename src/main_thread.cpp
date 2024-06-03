//----------------sanyi Remote control----------------
// version:      V1.0
// author:       Zhipeng Lin
// date:         2022.02.18

#include <unistd.h>
#include <sys/signal.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <sys/time.h>
#include <iostream>
#include "common/sany_type.h"
#include "protocol/protocol_cockpit.h"

Protocol_Cockpit *pCockpit = new Protocol_Cockpit();
FailureReporter *m_pFailureSDK = new FailureReporter();
SANY_U8 g_IpFlag = 0;
SANY_U8 g_u8CtrlState = 0;
CmdCtrlACK ctrlACK;
SANY_U8 g_u8TaskStopFlag = 0;
TopicDataType *pDataTypeACK = new CmdCtrlACKPubSubType();
ddsPublisher<CmdCtrlACK> *pPubACK = new ddsPublisher<CmdCtrlACK>(pDataTypeACK);
/*驾驶舱IP+PORT订阅*/
TopicDataType *pDataTypeCockpit = NULL;
ddsSubscriber<CmdtoRemoteCtrl> *pSubCockpit = NULL;

void cockpit_ctrlACK(CmdtoRemoteCtrl *addr)
{
	ctrlACK.Seq_Num() = addr->Seq_Num();
	pPubACK->publish(ctrlACK);
	return;
}

void cockpit_addressCallback(CmdtoRemoteCtrl *addr)
{
	MINE_LOG(SANY_LOG_INFO, "receive cockpit address,  ip_flag is %d\n", g_IpFlag);
	if (0 == g_IpFlag)
	{
		pCockpit->ProtoCockpit_setServerName(addr->ipaddr().c_str());
		cockpit_ctrlACK(addr);
		pCockpit->start();
		g_IpFlag = 1;		
	}
	MINE_LOG(SANY_LOG_INFO, "setServerName,  ip_flag is %d\n", g_IpFlag);

	return;
}



void write_Gps2log(int signum)
{
	MINE_LOG(SANY_LOG_INFO, "receive gps_info,   gps_lat %f\n", pCockpit->struGpsInfo.gps_lat());
}


int main()
{
	zkbs::init("syzk-remotectl");
	MINE_LOG(SANY_LOG_INFO, "syzk-remotectl V2.2.2 start!!\n");
	DaemonHandl Hdl = daemon_create("syzk-remotectl", 500, DEV_TYPE_XINGYUN);
	MINE_LOG(SANY_LOG_INFO, "SEND HEARTBEAT - DAEMON SUCCESS!!\n");
	zkbs::setVersion(VER("2.2"));
	MINE_LOG(SANY_LOG_INFO, "SET VERSION SUCCESS!!\n");
	mine_log::set_log_level(SANY_LOG_INFO);
	mine_log::set_log_output(SANY_LOG_OUT_FILE);
	MINE_LOG(SANY_LOG_INFO, "SET LOG LEVEL SUCCESS!!\n");
	/*驾驶舱IP+PORT订阅*/
	// TopicDataType *pDataTypeCockpit = new CmdtoRemoteCtrlPubSubType();
	// ddsSubscriber<CmdtoRemoteCtrl> *pSubCockpit = new ddsSubscriber<CmdtoRemoteCtrl>(pDataTypeCockpit, &cockpit_addressCallback);
	pDataTypeCockpit = new CmdtoRemoteCtrlPubSubType();
	pSubCockpit = new ddsSubscriber<CmdtoRemoteCtrl>(pDataTypeCockpit, &cockpit_addressCallback);

	MINE_LOG(SANY_LOG_INFO, "CREATE IP SUBSCRIBER SUCCESS!!\n");
	m_pFailureSDK->init();
	MINE_LOG(SANY_LOG_INFO, "FAILURE_SDK INIT!!\n");
	m_pFailureSDK->setModule(FailureModule_Nebula_RemoteCtrl);
	MINE_LOG(SANY_LOG_INFO, "FAILURE_SDK SET_MODULE!!\n");
	m_pFailureSDK->start();
	MINE_LOG(SANY_LOG_INFO, "FAILURE_SDK START!!\n");

	// 发布遥控驾驶控制数据
	ImpleResultInfo imple_result_info;
	TopicDataType *pDataTypeImple = new ImpleResultInfoPubSubType();
	ddsPublisher<ImpleResultInfo> *pPubImple = new ddsPublisher<ImpleResultInfo>(pDataTypeImple);
	MINE_LOG(SANY_LOG_INFO, "CREATE CONTROL_DATA PUBLISHER!!\n");
	SANY_U8 Remote_Enable5 = 0;
	SANY_U8 Tick = 0;

	timer_t timer;
    struct sigevent sev;
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = SIGALRM;
    sev.sigev_value.sival_ptr = &timer;
    signal(SIGALRM, write_Gps2log);

    if(timer_create(CLOCK_REALTIME, &sev, &timer) == -1)
        MINE_LOG(SANY_LOG_ERROR, "Timer_Create Fail!!!!\n");
    struct itimerspec its;
    its.it_value.tv_sec = 5;
    its.it_value.tv_nsec = 0;
    its.it_interval.tv_sec = 5;
    its.it_interval.tv_nsec = 0;
    if(timer_settime(timer, 0, &its, NULL) == -1)
        MINE_LOG(SANY_LOG_ERROR, "Timer_Settime Fail!!!!\n");
	MINE_LOG(SANY_LOG_INFO, "CREATE TIMER!!\n");
	while (! g_u8TaskStopFlag)
	{
		if (1 == g_u8CtrlState)
		{
			/*获取通信模块遥控控制数据*/
			pCockpit->ProtoCockpit_getCtrlDate(&imple_result_info);
			imple_result_info.Remote_enable(3);
			imple_result_info.unmanned_enable(1);
			/*发布遥控控制数据*/
			pPubImple->publish(imple_result_info);
			Remote_Enable5 = 0;
		}
		else if((2 == g_u8CtrlState) ||  (3 == g_u8CtrlState))
		{
			if(10 > Remote_Enable5)
			{
				pCockpit->ProtoCockpit_getCtrlDate(&imple_result_info);
				imple_result_info.Remote_enable(5);
				imple_result_info.unmanned_enable(1);
				pPubImple->publish(imple_result_info);
				Remote_Enable5++;
				MINE_LOG(SANY_LOG_INFO, "exit from remote-control, Remote_enable: %d, current Gear: %d, BrakeOpening: %f, Parking State: %d\n",imple_result_info.Remote_enable(), imple_result_info.trans_req_gear(), imple_result_info.traffic_brake_req(), imple_result_info.parking_req());
			}	

		}

		if((Tick % 100) == 0)
		{
			MINE_LOG(SANY_LOG_INFO, "syzk-remotectl V2.2.2 main is running!! ip_flag is  %d\n", g_IpFlag);
			Tick = 0;
		}
		Tick++;

		usleep(50 * 1000);
	}
	MINE_LOG(SANY_LOG_INFO, "syzk-remotectl V2.2.2 exit!!!\n");
	exit(0);
	// return 0;
}
