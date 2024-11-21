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
#include <FailureReporter.h>
#include "common/sany_type.h"
#include "protocol/protocol_cockpit.h"
#include "InitTask.h"

FailureReporter *m_pFailureSDK = new FailureReporter();
// InitTask initTask = new InitTask();

/*接收IP+PORT标志位，0 未收到 1 收到*/
SANY_U8 g_u8IpFlag = 0;
/*网络状态 0 断网 1 连接正常*/
SANY_U8 g_u8NetReady = 0;
	/*任务开始标识，1 开始，0 结束*/
SANY_U8 g_u8TaskStartFlag = 0;

void write_Gps2log(int signum)
{
	MINE_LOG(SANY_LOG_INFO, "syzk-remotectl is running!! g_u8Ipflag is %d, g_u8TaskStartFlag is %d, g_u8NetReady is %d\n", g_u8IpFlag, g_u8TaskStartFlag, g_u8NetReady);
}

int main()
{
	zkbs::init("syzk-remotectl");
	MINE_LOG(SANY_LOG_INFO, "syzk-remotectl V2.3.3 air+brake+newidl start!!\n");
	
	zkbs::setVersion(VER("3.3"));
	MINE_LOG(SANY_LOG_INFO, "SET VERSION SUCCESS!!\n");
	mine_log::set_log_level(SANY_LOG_INFO);
	mine_log::set_log_output(SANY_LOG_OUT_FILE);
	MINE_LOG(SANY_LOG_INFO, "SET LOG LEVEL SUCCESS!!\n");
	
	/*故障诊断模块*/
	MINE_LOG(SANY_LOG_INFO, "CREATE IP SUBSCRIBER SUCCESS!!\n");
	m_pFailureSDK->init();
	MINE_LOG(SANY_LOG_INFO, "FAILURE_SDK INIT!!\n");
	m_pFailureSDK->setModule(FailureModule_Nebula_RemoteCtrl);
	MINE_LOG(SANY_LOG_INFO, "FAILURE_SDK SET_MODULE!!\n");
	m_pFailureSDK->start();
	MINE_LOG(SANY_LOG_INFO, "FAILURE_SDK START!!\n");
	
	// 守护进程心跳
	DaemonHandl Hdl = daemon_create("syzk-remotectl", 500, DEV_TYPE_XINGYUN);
	MINE_LOG(SANY_LOG_INFO, "SEND HEARTBEAT - DAEMON SUCCESS!!\n");
	
	timer_t timer;
    struct sigevent sev;
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = SIGALRM;
    sev.sigev_value.sival_ptr = &timer;
    signal(SIGALRM, write_Gps2log);

    if(timer_create(CLOCK_REALTIME, &sev, &timer) == -1)
        MINE_LOG(SANY_LOG_ERROR, "Timer_Create Fail!!!!\n");
    struct itimerspec its;
    its.it_value.tv_sec = 10;
    its.it_value.tv_nsec = 0;
    its.it_interval.tv_sec = 10;
    its.it_interval.tv_nsec = 0;
    if(timer_settime(timer, 0, &its, NULL) == -1){
		MINE_LOG(SANY_LOG_ERROR, "Timer_Settime Fail!!!!\n");
	}
	MINE_LOG(SANY_LOG_INFO, "CREATE TIMER!!\n");


	InitTask *initTASK = new InitTask();
	initTASK->InitStart();

	while(1){
		sleep(1);
	}
	MINE_LOG(SANY_LOG_INFO, "syzk-remotectl V2.3.3 exit!!!\n");
	exit(0);
	// return 0;
}
