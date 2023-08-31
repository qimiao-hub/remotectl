#ifndef DEBUG_LOG_H
#define DEBUG_LOG_H

#include <stdio.h>
#include <ctime>
#include <fcntl.h>
#include <unistd.h>
#include <stdarg.h>
#include <pthread.h>
#include "common/type.h"


/* 打印模块定义 按位操作*/
#define DEBUG_LOG_PROTOCOL           		1
#define DEBUG_LOG_PROTOCOL_COCKPIT           2


/* 打印等级定义 */
#define DEBUG_LOG_OFF              		0
#define DEBUG_LOG_ERROR              	1
#define DEBUG_LOG_WARNING	        	2
#define DEBUG_LOG_INFO               	4


static const char* debugLevelDesc[4] = {"DEBUG_ERROR","DEBUG_WARNING","","DEBUG_INFO"};


#if 1

#define SANY_DEBUG(module, Level, fmt, ...)\
	if (g_u32DebugLogModule & (u32)module)		\
	{											\
		if (g_u8DebugLogLevel & (u8)Level)    \
		{                                   	\
			printf("[%s|%s@%s %d]" fmt , debugLevelDesc[(u8)Level - 1], __FUNCTION__ , __FILE__ , __LINE__ , ##__VA_ARGS__);\
		}\
	}
#else

#define SANY_DEBUG(module, Level, fmt, ...)\
			printf("[%s|%s@%s %d]" fmt , debugLevelDesc[(u8)Level - 1], __FUNCTION__ , __FILE__ , __LINE__ , ##__VA_ARGS__);\
	   
#endif /*CONFIG_DEBUG_TOOL*/


#if 1
class Debug_Log {
public:
    Debug_Log() {}
	void start();

	void debugServerInit();
	void redirectIOConsole(char          * ttyStr);

private:

	int socket_fd; 
	int cilent_fd;
    
	void run();
	pthread_t tid_log;
	static void* run_agency(void *);

};
#endif /*CONFIG_DEBUG_TOOL*/

#endif // DEBUG_LOG_H
