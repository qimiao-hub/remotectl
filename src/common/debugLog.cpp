#include "./debugLog.h"
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include<time.h>
#include "../protocol/protocol_cockpit.h"

#if 1

/*默认放开所有模块的错误打印*/
u8 g_u8DebugLogLevel = DEBUG_LOG_ERROR;
u32 g_u32DebugLogModule = 0xFFFFFFFF; 

extern Protocol_Cockpit* pCockpit; 

#define SERVER_PORT 8000

/*初始化服务端监控套接字*/
void Debug_Log::debugServerInit()
{	 
	struct sockaddr_in server_addr;
	
	if((socket_fd = socket(AF_INET,SOCK_STREAM,IPPROTO_TCP)) < 0)
	{
		perror("socket");
		return;
	}
	
	//初始化 server_addr
	memset(&server_addr,0, sizeof(server_addr));
	
	//初始化服务器端的套接字，并用htons和htonl将端口和地址转成网络字节序
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(SERVER_PORT);
	
	//ip可是是本服务器的ip，也可以用宏INADDR_ANY代替，代表0.0.0.0，表明所有地址
	server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	 
	//对于bind，accept之类的函数，里面套接字参数都是需要强制转换成(struct sockaddr *)
	//bind三个参数：服务器端的套接字的文件描述符，
	if(bind(socket_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
	{
		perror("connect");
		return;
	}
	 
	//设置服务器上的socket为监听状态
	if(listen(socket_fd, 1) < 0)
	{
		perror("listen");
		return;
	}
}

void Debug_Log::start()  //添加线程
{
	debugServerInit();
    if(pthread_create(&tid_log, NULL, run_agency, this) == -1)//!=0
    {
        printf("create sanyzk_log_thread error!\n");
    }
}

/*重定向程序标准IO到指定tty*/
void Debug_Log::redirectIOConsole(char         * ttyStr)
{
	int fd;
	printf("redirectIOConsole %s\n",ttyStr);
	fd = open(ttyStr,O_RDONLY | O_WRONLY);
	dup2(fd, 0);
	dup2(fd, 1);
	dup2(fd, 2);
	close(fd);
}

void* Debug_Log::run_agency(void *args)
{
    Debug_Log *pThread = static_cast<Debug_Log *>(args);
    pThread->run();
    return NULL;
}

/*串口打印处理线程 setDebugCtrl+模块号+打印等级        		"setDebugCtrl 0x01 0x01"*/
void Debug_Log::run()
{
	int size = 0;
	int iDebugLevel = 0;
	char szInStr[50] = {0};
	char szsendStr[50] = "cmd format err";
	char* tmpIndex1 = NULL;
	char* tmpIndex2 = NULL;

	char buffer[200]; //存储 发送和接收的信息 
	int iDataNum;
	struct sockaddr_in clientAddr;
	int addr_len = sizeof(clientAddr);
	
   	while (1)
    {		
    	cilent_fd = accept(socket_fd, (struct sockaddr*)&clientAddr, (socklen_t*)&addr_len);
		if(cilent_fd < 0)
		{
			perror("accept");
			continue;
		}

		bzero(szInStr, sizeof(szInStr));
		size = recv(cilent_fd, szInStr, sizeof(szInStr), 0);
		if((size > 0) && (size < sizeof(szInStr)))
		{
			//printf("----------------------------------------------recv msg: %s\n", szInStr);
			if(!memcmp(szInStr,"setLevel",strlen("setLevel")))
			{
				tmpIndex1 = NULL;
				tmpIndex2 = NULL;
				
				u32 u32DebugParam1 = 0;
				u32 u32DebugParam2 = 0;
					
				tmpIndex1 = strstr(szInStr, "0x");
				if(tmpIndex1)
				{
					tmpIndex2 = strstr(tmpIndex1 + 1, "0x");
					if(tmpIndex2)
					{
						u32DebugParam1 = (u32)strtol(tmpIndex1 + 2, NULL, 16);

						u32DebugParam2 = (u32)strtol(tmpIndex2 + 2, NULL, 16);

						if((u32DebugParam1 > 0xffffffff || u32DebugParam1 <= 0) || (u32DebugParam2 > 0xff || u32DebugParam2 <= 0))
						{
							printf("setDebugCtrl err,  Module %x Level %x invalid\n",u32DebugParam1, u32DebugParam2);
						}
						else
						{
							g_u32DebugLogModule = u32DebugParam1;
							g_u8DebugLogLevel = (u8)u32DebugParam2;
							printf("---------------------setDebugCtrl  Module %x Level %x success------------------------\n"\
								,g_u32DebugLogModule, g_u8DebugLogLevel);

							bzero(szsendStr, sizeof(szsendStr));
							memcpy(szsendStr, "setLevel success", MIN(strlen("setLevel success"), sizeof(szsendStr)));	
						}
					}
				}
			}
			/*重定向到指定终端*/
			else if(!memcmp(szInStr,"getConsole",strlen("getConsole")))
			{
				tmpIndex1 = NULL;
				tmpIndex2 = NULL;

				char ttyStr[20] = {0};
				
				tmpIndex1 = strstr(szInStr, "getConsole");
				memcpy(ttyStr, tmpIndex1 + strlen("getConsole") + 1, MIN(sizeof(ttyStr),(sizeof(szInStr)- strlen("getConsole") - 1)));
				//printf("---------------------getConsole %s ------------------------\n",ttyStr);
				redirectIOConsole(ttyStr);	
				
				bzero(szsendStr, sizeof(szsendStr));
				memcpy(szsendStr, "getConsole success", MIN(strlen("getConsole success"), sizeof(szsendStr)));	
			}
			/*重定向到log文件*/
			else if(!memcmp(szInStr,"logRedirect",strlen("logRedirect")))
			{
				int fdOut = 0;
				time_t rawtime;
				struct tm* timeinfo;
				char szLogName[50];
				time(&rawtime);
				timeinfo=localtime(&rawtime);
				strftime(szLogName,50,"%Y%m%d_%I%M%S\n",timeinfo);
				printf("%s",szLogName);
				
				memcpy(szLogName+strlen(szLogName)-1, ".sanyzktruck.log",sizeof(".sanyzktruck.log"));

				printf("%s\n",szLogName);

				fdOut = open(szLogName, O_WRONLY|O_CREAT, 0644);
				dup2(fdOut, 1);
				dup2(fdOut, 2);
				close(fdOut);
    			/*stdout是向程序的末尾的控制台重定向*/
    			printf("And now back to the console once again\n");
				
				bzero(szsendStr, sizeof(szsendStr));
				memcpy(szsendStr, "logRedirect success", MIN(strlen("logRedirect success"), sizeof(szsendStr)));	
			}
			else if(!memcmp(szInStr,"LoginOut",strlen("LoginOut")))
			{
				
				printf("---------------------LoginOut Cmd set ------------------------\n");
				
				/*如果存在对象则释放对象，否则报错*/
				if(pCockpit)
				{
					pCockpit->stop();
					delete pCockpit;
					pCockpit = NULL;
					
					bzero(szsendStr, sizeof(szsendStr));
					memcpy(szsendStr, "LoginOut set", MIN(strlen("LoginOut set"), sizeof(szsendStr)));	
				}
				else
				{
					printf("---------------------LoginOut err ------------------------\n");
				}
			}
			else if(!memcmp(szInStr,"Login",strlen("Login")))
			{
				char* p1 = NULL;
				char szServerName[50] = {0};

				p1 = strstr(szInStr, "Login");
				memcpy(szServerName, p1 + strlen("Login") + 1, MIN(sizeof(szServerName),(sizeof(szInStr)- strlen("Login") - 1)));

				printf("---------------------Login %s ------------------------\n",szServerName);
				if(strlen(szServerName) > 15)
				{	
					if(NULL == pCockpit)
					{
						pCockpit = new Protocol_Cockpit;
						pCockpit->ProtoCockpit_setServerName(szServerName);
						pCockpit->start();
					
						bzero(szsendStr, sizeof(szsendStr));
						memcpy(szsendStr, "Login set", MIN(strlen("Login set"), sizeof(szsendStr)));	
					}
					else
					{
						bzero(szsendStr, sizeof(szsendStr));
						memcpy(szsendStr, "Login Cockpit not null", MIN(strlen("Login Cockpit not null"), sizeof(szsendStr)));	
					}
					
				}
				else
				{
					bzero(szsendStr, sizeof(szsendStr));
					memcpy(szsendStr, "Login set err", MIN(strlen("Login set err"), sizeof(szsendStr)));
				}
				
			}
			else
			{
				bzero(szsendStr, sizeof(szsendStr));
				memcpy(szsendStr, "not sup cmd", MIN(strlen("not sup cmd"), sizeof(szsendStr)));	
			}
		}
		send(cilent_fd, szsendStr, strlen(szsendStr), 0); //服务端也向客户端发送消息 
		close(cilent_fd);
    }
}

#endif /*CONFIG_DEBUG_TOOL*/


