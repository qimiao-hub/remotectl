/**
 * 初始化任务
*/

#ifndef INITTASK_H
#define INITTASK_H
#include "functional"
#include "./common/read_ini.h"
#include "./protoDef.h"
#include "./common/yaml.h"
#include "./protocol_cockpit.h"

// #include <zkbs/ddsBase/ddsPubBase.h>
// #include <zkbs/ddsBase/ddsSubBase.h>
// #include "zkbs/zkbs.hpp"

#define CONFIG_PATH "/data/sany/config/runSetup.yaml"

// extern InitTask initTask;
extern SANY_U8 g_u8IpFlag;
extern SANY_U8 g_u8TaskStartFlag;
extern SANY_U8 g_u8NetReady;

class InitTask{
public:
     InitTask();
     ~InitTask();

public:
    void InitStart();
    SANY_U8 Start();
    SANY_U8 Stop();

private:
    // 读取配置文件
    void ReadConfig();
	Protocol_Cockpit *m_Cockpit;    

private:

	char cockpitServerName[25] = {0};
	const char HostclientName[22] = "192.168.8.249:8080";
	terminal_client m_Terminal;
    sa_comm_socket_client *m_Socket_CockpitClient;

	/*网络状态 0 断网 1 连接正常*/
	// SANY_U8 g_u8NetReady = 0;
		/*接收IP+PORT标志位，0 未收到 1 收到*/
	// static volatile SANY_U8 g_u8IpFlag = 0;
	
	//dds定义
	/*驾驶舱IP+PORT订阅*/
	TopicDataType *pDataTypeCockpit = NULL;
	TopicDataType *pDataTypeACK = NULL;

	/*驾驶舱IP+PORT确认发布*/
	CmdCtrlACK ctrlACK;
	ddsSubscriber<CmdtoRemoteCtrl> *pSubCockpit = NULL;
	ddsPublisher<CmdCtrlACK> *pPubACK = NULL;

private:
	//
	SANY_U8 ProtoCockpit_setServerName(const char *pServerName);
    SANY_U8 InitSocket_Cockpit();
	void StopSocket_Cockpit();
	// dss回调
	void cockpit_addressCallback(CmdtoRemoteCtrl &addr);
	void cockpit_ctrlACK(CmdtoRemoteCtrl &addr);

private:
	/*线程id*/
    pthread_t t_recv_id;
    pthread_t t_send_id;

	// /*线程开始标识，置位0后四个线程阻塞，置位1后4个线程开始工作*/
	// volatile SANY_U8 g_u8ThreadStartFlag = 0;
private:
    /*数据接收线程*/
	void funcRecvData();
	/*数据发送线程*/
	void funcSendData();
	/*数据接收线程静态成员*/
	static void *threadRecvData(void *args);
	/*数据发送线程静态成员*/
	static void *threadSendData(void *args);

	void funcConnect();

	static void *threadConnect(void *args);
	
};


#endif