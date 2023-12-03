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
#include "common/debugLog.h"

CmdCtrlACK ctrlACK;
TopicDataType *pDataTypeACK = new CmdCtrlACKPubSubType();
ddsPublisher<CmdCtrlACK> *pPubACK = new ddsPublisher<CmdCtrlACK>(pDataTypeACK);
Protocol_Cockpit *pCockpit = new Protocol_Cockpit();
FailureReporter *m_pFailureSDK = new FailureReporter();
SANY_U8 ip_flag = 0;

SANY_U8 u8CtrlStatus = 0;

void sig_handler(int sig)
{
	//printf("-------------sig_handler Signal %d-------------\n", sig);
	// if (sig == SIGINT)
	// {
	// }
	MINE_LOG(SANY_LOG_ERROR, "-------------sig_handler Signal %d-------------\n", sig);
	//	return;
	exit(0);
}

void cockpit_ctrlACK(CmdtoRemoteCtrl *addr)
{
	ctrlACK.Seq_Num() = addr->Seq_Num();
	pPubACK->publish(ctrlACK);
	return;
}

void cockpit_addressCallback(CmdtoRemoteCtrl *addr)
{
	if (0 == ip_flag)
	{
		pCockpit->ProtoCockpit_setServerName(addr->ipaddr().c_str());
		cockpit_ctrlACK(addr);
		pCockpit->start();
	}
	return;
}

int main()
{
	zkbs::init("syzk-remotectl");
	DaemonHandl Hdl = daemon_create("syzk-remotectl", 500, DEV_TYPE_XINGYUN);
	zkbs::setVersion(VER("1.1"));

	mine_log::set_log_level(SANY_LOG_INFO);
	mine_log::set_log_output(SANY_LOG_OUT_FILE);
	/*驾驶舱IP+PORT订阅*/
	TopicDataType *pDataTypeCockpit = new CmdtoRemoteCtrlPubSubType();
	ddsSubscriber<CmdtoRemoteCtrl> *pSubCockpit = new ddsSubscriber<CmdtoRemoteCtrl>(pDataTypeCockpit, &cockpit_addressCallback);

	printf("------------------main strart--------------------\n");

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
	SANY_U8 u32NetStatus = 0;
	SANY_U8 count  = 0;
	SANY_U8 Remote_Enable5 = 0;
	while (! pCockpit->u8TaskStopFlag)
	{
		if(1 == ip_flag)
		{
			/* 断开连接后进行重连 */
			/*尝试重连6次未成功，程序退出*/
			if(9 < count)
			{
				MINE_LOG(SANY_LOG_ERROR, "Reconnect TCP Fail, stop!!!!!\n");
				printf("Reconnect TCP Fail, stop!!!!!\n");
				pCockpit->stop();
			}
			/* 定时器到时后尝试重连 */
			MINE_LOG(SANY_LOG_ERROR, "Reconnect TCP!\n");
			printf("Reconnect TCP!\n");
			u32NetStatus = pCockpit->m_Terminal.comm_socket_client_do_connect(pCockpit->m_Socket_CockpitClient);
			if (1 != u32NetStatus)
			{
				count++;
				MINE_LOG(SANY_LOG_ERROR, "sa_comm_socket_client_send fail\n");
				/*延时1s重连*/
				sleep(1);
				continue;
			}
			count = 0;
			/* 网络连接成功，更新网络状态 */
			pCockpit->bNetReady = 1;
			MINE_LOG(SANY_LOG_ERROR, "Reconnect Success! \n");
			printf("Reconnect Success! \n");
		}

		if (1 == u8CtrlStatus)
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
		else if((2 == u8CtrlStatus) && (Remote_Enable5 < 10))
		{
			Remote_Enable5++;
			imple_result_info.Remote_enable(5);
			imple_result_info.unmanned_enable(1);
			pPubImple->publish(imple_result_info);
		}
		else if(3 == u8CtrlStatus)
		{
			imple_result_info.Remote_enable(5);
			imple_result_info.unmanned_enable(0);
			pPubImple->publish(imple_result_info);
		}
		
		usleep(50 * 1000);
	}

	return 0;
}
