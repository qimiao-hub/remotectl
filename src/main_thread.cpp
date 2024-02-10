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

CmdCtrlACK ctrlACK;
TopicDataType *pDataTypeACK = new CmdCtrlACKPubSubType();
ddsPublisher<CmdCtrlACK> *pPubACK = new ddsPublisher<CmdCtrlACK>(pDataTypeACK);
Protocol_Cockpit *pCockpit = new Protocol_Cockpit();
FailureReporter *m_pFailureSDK = new FailureReporter();
SANY_U8 g_IpFlag = 0;
SANY_U8 g_u8CtrlState = 0;

void cockpit_ctrlACK(CmdtoRemoteCtrl *addr)
{
	ctrlACK.Seq_Num() = addr->Seq_Num();
	pPubACK->publish(ctrlACK);
	return;
}

void cockpit_addressCallback(CmdtoRemoteCtrl *addr)
{
	if (0 == g_IpFlag)
	{
		pCockpit->ProtoCockpit_setServerName(addr->ipaddr().c_str());
		cockpit_ctrlACK(addr);
		pCockpit->start();
		g_IpFlag = 1;		
	}
	return;
}

int main()
{
	zkbs::init("syzk-remotectl");
	DaemonHandl Hdl = daemon_create("syzk-remotectl", 500, DEV_TYPE_XINGYUN);
	zkbs::setVersion(VER("1.4"));

	mine_log::set_log_level(SANY_LOG_INFO);
	mine_log::set_log_output(SANY_LOG_OUT_FILE);
	/*驾驶舱IP+PORT订阅*/
	TopicDataType *pDataTypeCockpit = new CmdtoRemoteCtrlPubSubType();
	ddsSubscriber<CmdtoRemoteCtrl> *pSubCockpit = new ddsSubscriber<CmdtoRemoteCtrl>(pDataTypeCockpit, &cockpit_addressCallback);

	m_pFailureSDK->init();
	m_pFailureSDK->setModule(FailureModule_Nebula_RemoteCtrl);
	m_pFailureSDK->start();
	// 发布遥控驾驶控制数据
	ImpleResultInfo imple_result_info;
	TopicDataType *pDataTypeImple = new ImpleResultInfoPubSubType();
	ddsPublisher<ImpleResultInfo> *pPubImple = new ddsPublisher<ImpleResultInfo>(pDataTypeImple);
	// 给主控发遥控接管使能位+最大车速
	// Remote2Control remote_ctrl;
	// TopicDataType *pDataTypeCtrl = new Remote2ControlPubSubType();
	// ddsPublisher<Remote2Control> *pPubCtrl = new ddsPublisher<Remote2Control>(pDataTypeCtrl);
	SANY_U8 Remote_Enable5 = 0;

	while (1)
	{

		if (1 == g_u8CtrlState)
		{
			/*获取通信模块遥控控制数据*/
			pCockpit->ProtoCockpit_getCtrlDate(&imple_result_info);
			/*发布遥控控制数据*/
			pPubImple->publish(imple_result_info);
			Remote_Enable5 = 0;
			/*遥控接管使能位和最大车速to主控*/
			// pCockpit->ProtoCockpit_getMaxSpeed(&remote_ctrl);
			// pPubCtrl->publish(remote_ctrl);
		}
		else if((2 == g_u8CtrlState) ||  (3 == g_u8CtrlState))
		{
			if(10 > Remote_Enable5)
			{
				pCockpit->ProtoCockpit_getCtrlDate(&imple_result_info);
				pPubImple->publish(imple_result_info);
				Remote_Enable5++;
				MINE_LOG(SANY_LOG_INFO, "exit from remote-control, current Gear: %d, BrakeOpening: %f, Parking State: %d\n", imple_result_info.trans_req_gear(), imple_result_info.traffic_brake_req(), imple_result_info.parking_req());
			}
		
		}

		usleep(50 * 1000);
	}

	

	return 0;
}
