
#ifndef sany_TYPE_H_
#define sany_TYPE_H_

#include <pthread.h>
#include <semaphore.h>

#ifdef __cplusplus
extern "C" {
#endif


//--------------------------------------普通类型----------------------------------------

typedef unsigned char 			SANY_U8;
typedef unsigned short 			SANY_U16;
typedef unsigned int 			SANY_U32;
typedef char		 			SANY_S8;
typedef short 					SANY_S16;
typedef int 					SANY_S32;
typedef unsigned long long 		SANY_U64;
typedef long long 				SANY_S64;
typedef void 					SANY_VOID;
typedef float 					SANY_F32;
typedef double                  SANY_DOUBLE;



typedef enum
{
	SANY_FALSE = 0,
	SANY_TRUE ,
}SANY_BOOL;


#define SANY_FAILED          (-1)
#define SANY_SUCCESS         (0)
#define SANY_ALL_SUCCESS     (1)
#define SANY_ALL_FAILED      (-2)


typedef pthread_t           SANY_THREAD_T;
typedef pthread_mutex_t     SANY_MUTEX_T;
typedef sem_t               SANY_SEM_T;
typedef pthread_cond_t      SANY_COND_T;
typedef pthread_rwlock_t    SANY_RW_MUTEX_T;



typedef enum
{
	SANY_ERR_PARAM						=  -0x10002,			//参数错误	
	SANY_ERR_DISCONNECT 				=  -0x10003,			//连接失败	
	SANY_ERR_REPEATED 					=  -0x10004, 			//重复
	SANY_ERR_NO_FIND 					=  -0x10005, 			//找不到
	SANY_ERR_NO_DEVICE					=  -0x10006, 			//缺失设备
	SANY_ERR_SPACE_FULL					=  -0x10007, 			//空间已满
	SANY_ERR_SPACE_EMPTY				=  -0x10008, 			//空间已空
	SANY_ERR_NO_FILE					=  -0x10009, 			//缺失文件
	SANY_ERR_ALLOC						=  -0x1000a, 			//分配内存失败
	SANY_ERR_PTHREAD					=  -0x1000b, 			//创建线程失败
	SANY_ERR_BUSY						=  -0x1000c, 			//业务正忙
	SANY_ERR_INTERNAL					=  -0x1000d, 			//内部错误
	SANY_ERR_TIMEOUT					=  -0x1000e, 			//超时
	SANY_ERR_DATA_CONFUSED 				=  -0x1000f, 			//数据混乱					 			
}SANY_ERR_E;



#define SANY_1K (1024)
#define SANY_2K (2048)
#define SANY_4K (4096)
#define SANY_1M (1048576)



#ifdef __cplusplus
}
#endif


#endif //sany_TYPE_H_





