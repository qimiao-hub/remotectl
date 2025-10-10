#include "../include/InitTask.h"


InitTask::InitTask(){

}

InitTask::~InitTask(){
	pthread_join(t_recv_id, NULL);
	pthread_join(t_send_id, NULL);
	MINE_LOG(SANY_LOG_ERROR, "STOP, wait for pthread_join!!!\n");
}

void InitTask::InitStart(){
    m_Cockpit = new Protocol_Cockpit();
	m_Cockpit->InitProtocol();
	sleep(2);

    ReadConfig();

	/*驾驶舱IP+PORT订阅和确认发布*/
	pDataTypeCockpit = new CmdtoRemoteCtrlPubSubType();
	pSubCockpit = new ddsSubscriber<CmdtoRemoteCtrl>(pDataTypeCockpit, std::bind(&InitTask::cockpit_addressCallback, this, std::placeholders::_1));
	MINE_LOG(SANY_LOG_INFO, "CREATE IP_PORT SUBSCRIBER!!\n");

	pDataTypeACK = new CmdCtrlACKPubSubType();
	pPubACK = new ddsPublisher<CmdCtrlACK>(pDataTypeACK);
	MINE_LOG(SANY_LOG_INFO, "CREATE IP_PORT ACK PUBLISHER!!!\n");

	if (pthread_create(&t_recv_id, NULL, threadRecvData, this) == -1)
	{
		MINE_LOG(SANY_LOG_ERROR, "create threadRecvData thread error!\n");
	}

	if (pthread_create(&t_send_id, NULL, threadSendData, this) == -1)
	{
		MINE_LOG(SANY_LOG_ERROR, "create threadSendData thread error!\n");
	}
}

/*IP+PORT确认发布*/
void InitTask::cockpit_ctrlACK(CmdtoRemoteCtrl &addr)
{
	MINE_LOG(SANY_LOG_INFO, "publish IP+PORT ack \n");
	ctrlACK.Seq_Num() = addr.Seq_Num();
	pPubACK->publish(ctrlACK);
	return;
}

/*IP+PORT订阅回调*/
void InitTask::cockpit_addressCallback(CmdtoRemoteCtrl &addr)
{
	MINE_LOG(SANY_LOG_INFO, "receive cockpit address,  g_u8Ipflag is %d\n", g_u8IpFlag);
	if (0 == g_u8IpFlag)
	{	
		g_u8IpFlag = 1;
		cockpit_ctrlACK(addr);
		ProtoCockpit_setServerName(addr.ipaddr().c_str());
		
		if(InitSocket_Cockpit() == RET_OK){
			m_Cockpit->Task_start();
		}else{
			g_u8IpFlag = 0;
			StopSocket_Cockpit();
			m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 3, 0);
		}
	}
	MINE_LOG(SANY_LOG_INFO, "setServerName,  g_u8IpFlag is %d\n", g_u8IpFlag);

	return;
}

/*读取配置文件*/
void InitTask::ReadConfig(){

    YAML::Node config = YAML::LoadFile(CONFIG_PATH);
	if(! config)
	{
		MINE_LOG(SANY_LOG_ERROR, "Read Config-runSetup.yaml fail!!!");
	}
	else
	{
		m_Cockpit->vehicle_vin = config["vehicle"]["vehicle_VIN"].as<std::string>();
		m_Cockpit->vehicle_iccid = config["vehicle"]["vehicle_ICCID"].as<std::string>();
		SANY_U32 config_v_type = config["vehicle"]["vehicle_type"].as<int>();
	//	brake_delay = config["vehicle"]["vehicle_delay"].as<int>();
	//	read_brakeing = config["vehicle"]["vehicle_brake"].as<int>();
		
		// 1 为油车
		if(config_v_type == 1)
		{
			m_Cockpit->vehicle_type = "SKT-I";
		}
		// 2 为电车
		else if(config_v_type == 2)
		{
			m_Cockpit->vehicle_type = "SKT-E";
		}
		/* 20250306 liuss31 新增车型 3 为 130H混型-------- start*/ 
		else if(config_v_type == 3)
		{
			m_Cockpit->vehicle_type = "SKT-H";
		}
		else{
			// 默认油车
			MINE_LOG(SANY_LOG_ERROR, "Read Config-runSetup.yaml vehicle_type is %d, SET 1 (SKT-I) !!!\n", config_v_type);
			config_v_type = 1;
			m_Cockpit->vehicle_type = "SKT-I";
		}
		/* 20250306 liuss31 新增车型 130H混型-------- end*/ 
		MINE_LOG(SANY_LOG_INFO, "Vehicle VIN is %s\n", m_Cockpit->vehicle_vin.c_str());
		MINE_LOG(SANY_LOG_INFO, "Vehicle ICCID is %s\n", m_Cockpit->vehicle_iccid.c_str());
		MINE_LOG(SANY_LOG_INFO, "Vehicle type is %s\n", m_Cockpit->vehicle_type.c_str());
	}
}

/*******************************************************************************
threadRecvData 数据接收线程，如有网络数据则将数据存储到接收缓冲区等待处理
********************************************************************************/
void InitTask::funcRecvData()
{
	SANY_U32 Tick = 0;
	SANY_U32 u32ReadLen = 0;
	SANY_U32 u32DateLen = 0;
	fd_set fdR;
	struct timeval timeout;
	CP_Header_st *pHeadST = NULL;
	SANY_U8 u8HaveRecvHead = 0;
	SANY_U8 flag = 0;
	MINE_LOG(SANY_LOG_INFO, "funcRecvData thread START!!\n");
	while (true)
	{
		if(1 == g_u8TaskStartFlag)
		{
			if(0 == flag){
				MINE_LOG(SANY_LOG_INFO, "funcRecvData thread restart!!\n");
				flag = 1;
			}
			u32ReadLen = 0;
			if (m_Socket_CockpitClient->b_connected)
			{
				// MINE_LOG(SANY_LOG_INFO, "funcRecvData thread m_Socket, g_u8IpFlag is %d, g_u8TaskStartFlag is %d, Tick is %d \n", g_u8IpFlag, g_u8TaskStartFlag, Tick);
				timeout.tv_sec = 1;
				timeout.tv_usec = 0;

				FD_ZERO(&fdR);
				FD_SET(m_Socket_CockpitClient->socket_fd, &fdR);

				switch (select(m_Socket_CockpitClient->socket_fd + 1, &fdR, NULL, NULL, &timeout))
				{
				case -1:{
					MINE_LOG(SANY_LOG_WARNING, "Read data ERROR fd(%d) \r\n",m_Socket_CockpitClient->socket_fd);
					break;
				}
				case 0:{
					MINE_LOG(SANY_LOG_WARNING, "Read data Timeout(%d) \r\n",m_Socket_CockpitClient->socket_fd);
					break;
				}
				default:
				{
					// MINE_LOG(SANY_LOG_ERROR, "funcRecvData thread default g_u8IpFlag is %d, g_u8TaskStartFlag is %d\n", g_u8IpFlag, g_u8TaskStartFlag);
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
									m_Cockpit->ProtoCockpit_SendAckCom(pHeadST->ucCommand[0], 0x2);
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
							 	m_Cockpit->ProtoCockpit_WriteRecvBuffer((SANY_U8 *)m_Socket_CockpitClient->recv_buffer, sizeof(CP_Header_st) + u32DateLen);
								// MINE_LOG(SANY_LOG_INFO, "receive data, write to recv buffer!!!!!!!!!!!\n");
							}
							u8HaveRecvHead = 0;
							u32DateLen = 0;
						}
					}
				}
				}
			}
		}else{
			u32ReadLen = 0;
			u32DateLen = 0;
			pHeadST = NULL;
			u8HaveRecvHead = 0;
			if(1 == flag){
				StopSocket_Cockpit();
				flag = 0;
				MINE_LOG(SANY_LOG_INFO, "funcRecvData thread stop!!\n");
			}
		}
		Tick++;
		if((Tick % 10000) == 0)
		{
			MINE_LOG(SANY_LOG_INFO, "funcRecvData thread is running\n");
			// Tick = 0;
		}
		usleep(1000);
		
	}
	MINE_LOG(SANY_LOG_INFO, "funcRecvData thread EXIT!!\n");
}

/*******************************************************************************
threadSendData 数据发送线程，检测环形缓冲区中有无数据，有数据则立即发送
********************************************************************************/
void InitTask::funcSendData()
{
	SANY_U32 Tick = 0;
	SANY_U8 u8Ret = RET_OK;
	SANY_U32 u32SendLen = 0;
	SANY_U32 u32NetStatus = 0;
	SANY_U8 szSendBuffer[PROTO_PACK_MAX_LEN] = {0};
	SANY_U8 flag = 0;
	SANY_U8 u8ConnectTimes = 0;
	MINE_LOG(SANY_LOG_INFO, "funcSendData thread START!!\n");
	while (true)
	{
		if(1 == g_u8TaskStartFlag)
		{
			if(0 == flag){
				MINE_LOG(SANY_LOG_INFO, "funcSendData thread restart!!\n");
				flag = 1;
			}
			if (m_Socket_CockpitClient->b_connected && g_u8NetReady == 1)
			{
				// MINE_LOG(SANY_LOG_INFO, "funcSendData thread connected\n");
				memset(szSendBuffer, 0, sizeof(szSendBuffer));

				u8Ret =	m_Cockpit->ProtoCockpit_ReadSendBuffer(szSendBuffer, &u32SendLen);
				if (RET_OK != u8Ret || 0 >= u32SendLen)
				{
					// MINE_LOG(SANY_LOG_ERROR, "ProtoCockpit_ReadSendBuffer error !!\n");
					continue;
				}
				if (!m_Terminal.sa_comm_socket_client_send(m_Socket_CockpitClient, (char *)szSendBuffer, (int)u32SendLen))
				{
					MINE_LOG(SANY_LOG_ERROR, "sa_comm_socket_client_send err, u32SendLen: %d,  g_u8NetReady set 0 !!\n", u32SendLen);
					/*发送失败，复位网络状态*/
					g_u8NetReady = 0;
					continue;
				}
			}
			else
			{
				if(m_Cockpit->g_u8LoginFlag != 2){
					/* 断开连接后进行重连 */
					m_Cockpit->g_u8LoginFlag = 0; // 修改登入状态
				}
				// m_Cockpit->g_BrakeStartCount = 0;
				// /*断开网络连接*/
				if(m_Socket_CockpitClient->b_connected){
					m_Terminal.comm_socket_client_do_disconnect(m_Socket_CockpitClient);
					MINE_LOG(SANY_LOG_ERROR, "STOP, sa_comm_socket_client_disconnect!!!\n");
				}
				if(m_Cockpit->g_u8LoginFlag != 2){
					/* 定时器到时后尝试重连 */
					MINE_LOG(SANY_LOG_INFO, "Reconnect TCP, current login flag: %d\n", m_Cockpit->g_u8LoginFlag);
					if(19 < u8ConnectTimes)
					{
						MINE_LOG(SANY_LOG_ERROR, "Reconnect 10 seconds, exit!!! \n");
						m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 3, 1);
						m_Cockpit->g_u8LoginFlag = 2;
						m_Cockpit->Task_stop();
					}
					else
					{
						u32NetStatus = m_Terminal.comm_socket_client_do_connect(m_Socket_CockpitClient);
						MINE_LOG(SANY_LOG_INFO, "comm_socket_client_do_connect, u32NetStatus: %d\n", u32NetStatus);
						if (1 != u32NetStatus)
						{
							m_Cockpit->ProtoCockpit_initCtrlData();
							u8ConnectTimes++;
							MINE_LOG(SANY_LOG_ERROR, "sa_comm_socket_client_send fail, u8ConnectTimes: %d\n", u8ConnectTimes);					
							/*延时500ms重连*/
							usleep(500 * 1000);
							continue;
						}
						/* 网络连接成功，更新网络状态 */
						g_u8NetReady = 1;
						u8ConnectTimes = 0;
						MINE_LOG(SANY_LOG_ERROR, "Reconnect TCP Success!!! m_Socket_CockpitClient->b_connected: %d, clear u8ConnectTimes: %d\n", m_Socket_CockpitClient->b_connected, u8ConnectTimes);
					}
				}else{
					MINE_LOG(SANY_LOG_INFO, "EXIT, Socket STOP, g_u8LoginFlag %d\n", m_Cockpit->g_u8LoginFlag);
				}
			}
		}else{
			u8Ret = RET_OK;
			u32SendLen = 0;
			u8ConnectTimes = 0;
			szSendBuffer[PROTO_PACK_MAX_LEN] = {0};
			if(1 == flag){
				StopSocket_Cockpit();
				flag = 0;
				MINE_LOG(SANY_LOG_INFO, "funcSendData thread stop!!\n");
			}
		}
		Tick++;
		if((Tick % 10000) == 0)
		{
			MINE_LOG(SANY_LOG_INFO, "funcSendData thread is running\n");
			// Tick = 0;
		}
		usleep(1000);
	}
	MINE_LOG(SANY_LOG_ERROR, "funcSendData thread EXIT\n");
}

/*数据接收线程*/
void *InitTask::threadRecvData(void *args)
{
	InitTask *pThread = static_cast<InitTask *>(args);
	pThread->funcRecvData();
}

/*数据发送线程*/
void *InitTask::threadSendData(void *args)
{
	InitTask *pThread = static_cast<InitTask *>(args);
	pThread->funcSendData();
}

/*初始化建立连接*/
SANY_U8 InitTask::InitSocket_Cockpit()
{
	g_u8NetReady = 0;
	m_Socket_CockpitClient = m_Terminal.sa_comm_socket_client_new((const char *)cockpitServerName, (const char *)HostclientName, false);
	
	/*死等网络连接成功*/
	SANY_U8 g_count = 0;
	while (!m_Terminal.comm_socket_client_do_connect(m_Socket_CockpitClient))
	{
		// sany_log_set_output(logHandleLocal, SANY_LOG_OUT_BOTH);
		MINE_LOG(SANY_LOG_ERROR, "Connect server fail %d times\n", g_count);
		if(g_count > 10)
		{
			MINE_LOG(SANY_LOG_ERROR, "Connect server fail 10 times, EXIT!!\n");
			m_pFailureSDK->report(FailureModule_Nebula_RemoteCtrl, 3201, 3, 1);
			// Task_Stop();
			return RET_ERROR;
		}
		g_count++;
		sleep(1);
	}
	g_u8NetReady = 1;
	
	if(m_Socket_CockpitClient->b_connected){
		g_u8TaskStartFlag = 1;
	}
	MINE_LOG(SANY_LOG_INFO, "Connect server success\n");
	return RET_OK;
}

/*******************************************************************************
 * 函数名称:      ProtoCockpit_setServerName
 * 描述:        设置驾舱服务端地址
 * parma in:  char * pServerName	驾舱IP+端口号  例："192.168.8.76:8080"
 * 返回值:       RET_OK/RET_ERROR
 *
 ********************************************************************************/
SANY_U8 InitTask::ProtoCockpit_setServerName(const char *pServerName)
{
	MINE_LOG(SANY_LOG_INFO, "ProtoCockpit_setServerName!! \n");
	if (!pServerName)
	{
		MINE_LOG(SANY_LOG_ERROR, "pServerName is NULL, %s\n", cockpitServerName);
		return RET_ERROR;
	}
	std::string _ser = pServerName;
	_ser.erase(std::remove_if(_ser.begin(), _ser.end(),
        [](char ch) { return !isdigit(ch) && ch != '.' && ch != ':'; }),
        _ser.end());
	const char *_ipresult = _ser.c_str();
	memset(cockpitServerName, 0, sizeof(cockpitServerName));
	memcpy(cockpitServerName, _ipresult, MIN(strlen(_ipresult), sizeof(cockpitServerName)));
	MINE_LOG(SANY_LOG_INFO, "ProtoCockpit_setServerName cockpitServerName %s\n", cockpitServerName);
	return RET_OK;
}

/*断开网络连接*/
void InitTask::StopSocket_Cockpit(){
	g_u8NetReady = 0;
	if(m_Socket_CockpitClient->b_connected){
		shutdown(m_Socket_CockpitClient->socket_fd, SHUT_RDWR);
		m_Terminal.comm_socket_client_do_disconnect(m_Socket_CockpitClient);
		MINE_LOG(SANY_LOG_INFO, "STOP, sa_comm_socket_client_disconnect!!!\n");
	}
	// if(m_Socket_CockpitClient != NULL){
	// 	m_Terminal.sa_comm_socket_client_release(m_Socket_CockpitClient);
	// 	MINE_LOG(SANY_LOG_INFO, "STOP, sa_comm_socket_client_release!!!\n");
	// }
}
