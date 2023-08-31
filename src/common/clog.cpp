#include "clog.h"
#include <iostream>

using namespace common;

FILE* CLog::file_stream = NULL;
FILE* CLog::data_stream[DEBUG_DATA_NUM];
int CLog::log_mode = 0;
void (*CLog::logPrintf)(const char *,...) = NULL;
void (*CLog::warningPrintf)(const char *,...) = NULL;
void (*CLog::errorPrintf)(const char *,...) = NULL;

void logC(const char *func, const char *file, const int line,
          const char *type, const char *format, ...)
{
    FILE *file_fp;
    time_t loacl_time;
    char time_str[128];

    // 获取本地时间
    time(&loacl_time);
    strftime(time_str, sizeof(time_str), "[%Y.%m.%d %X]", localtime(&loacl_time));

    // 日志内容格式转换
    va_list ap;
    va_start(ap, format);
    char fmt_str[2048];
    vsnprintf(fmt_str, sizeof(fmt_str), format, ap);
    va_end(ap);

    // 写入到日志文件中
    if(CLog::log_mode&LOG_NULL_MOD)
    {
        return;
    }

    if(CLog::log_mode&LOG_FILE_MOD)
    {
        fprintf(CLog::file_stream, "%s||%s@%s:%d||%s\n", type, func,
            file, line, fmt_str);
    }
    if(CLog::log_mode&LOG_STDOUT_MOD)
    {
        fprintf(stdout, "%s@%s:%d||%s\n", func,
            file, line, fmt_str);
    }

}

int CLog::logSwitch(bool bLog,bool bWarn,bool bError)
{
    if(!bLog)
        logPrintf = logPrintf_null;
    if(!bWarn)
        warningPrintf = warningPrintf_null;
    if(!bError)
        errorPrintf = errorPrintf_null;
}

int CLog::logStart(const char* log_file_name,int mode)
{
    log_mode = mode;
    switch (log_mode)
    {
    case LOG_STDOUT_MOD|LOG_FILE_MOD:
        logPrintf = logPrintf_vp_vfp;
        warningPrintf = warningPrintf_vp_vfp;
        errorPrintf = errorPrintf_vp_vfp;
        logOpen(log_file_name);
        break;
    case LOG_STDOUT_MOD:
        logPrintf = logPrintf_vp;
        warningPrintf = warningPrintf_vp;
        errorPrintf = errorPrintf_vp;
        break;
    case LOG_FILE_MOD:
        logPrintf = logPrintf_vfp;
        warningPrintf = warningPrintf_vfp;
        errorPrintf = errorPrintf_vfp;
        logOpen(log_file_name);
        break;
    case LOG_NULL_MOD:
        logPrintf = logPrintf_null;
        warningPrintf = warningPrintf_null;
        errorPrintf = errorPrintf_null;
        break;
    default:
        break;
    }
    int ret = 0;

#if LOG_DATA_WRITE
    system("rm ./log/data_dms_*");
    for(int i=0; i<DEBUG_DATA_NUM; i++)
    {
        char log_file[256];
        sprintf (log_file,"./log/data_dms_%drun", i);
        data_stream[i] = fopen(log_file,"a+");
        if(data_stream[i] == NULL)
        {
            printf("open data%d error\n",i);
            ret = -1;
        }
    }
#endif
    return ret;
}

void CLog::logEnd()
{
    switch (log_mode)
    {
    case LOG_STDOUT_MOD|LOG_FILE_MOD:
    case LOG_FILE_MOD:
        logclose();
        break;
    default:
        break;
    }
    for(int i=0; i<DEBUG_DATA_NUM; i++)
    {
        printf("fclose(data_stream[%d])\n",i);
        fclose(data_stream[i]);
        data_stream[i] = NULL;
    }
}


int CLog::logOpen(const char* log_file_name)
{
    int ret = 0;

    file_stream = fopen(log_file_name,"a+");
    if(file_stream == NULL)
    {
        ret = -1;
    }

    return ret;
}

void CLog::logclose()
{
    fclose(file_stream);
    file_stream = NULL; 
}

void CLog::logPrintf_null(const char *cmd, ...)
{

}
void CLog::warningPrintf_null(const char *cmd, ...)
{

}
void CLog::errorPrintf_null(const char *cmd, ...)
{

}

void CLog::saveData(int index,const char *cmd, ...)
{
    va_list args;       //定义一个va_list类型的变量，用来储存单个参数
    va_start(args,cmd); //使args指向可变参数的第一个参数
    vfprintf(data_stream[index],cmd,args);  //必须用vprintf等带V的
    va_end(args);       //结束可变参数的获取
}

void CLog::logPrintf_vp(const char *cmd, ...)
{
    //        if( !CGlobal::GetInstance()->m_bOpenLogP )
    //            return;
    printf("log: ");
    va_list args;       //定义一个va_list类型的变量，用来储存单个参数
    va_start(args,cmd); //使args指向可变参数的第一个参数
    vprintf(cmd,args);  //必须用vprintf等带V的
    va_end(args);       //结束可变参数的获取
}

void CLog::logPrintf_vfp(const char *cmd, ...)
{
    //        if( !CGlobal::GetInstance()->m_bOpenLogP )
//                return;
    fprintf(file_stream,"log: ");
    va_list args;       //定义一个va_list类型的变量，用来储存单个参数
    va_start(args,cmd); //使args指向可变参数的第一个参数
    vfprintf(file_stream,cmd,args);  //必须用vprintf等带V的
    va_end(args);       //结束可变参数的获取
}

void CLog::logPrintf_vp_vfp(const char *cmd, ...)
{
    //        if( !CGlobal::GetInstance()->m_bOpenLogP )
    //            return;
    printf("log: ");
    fprintf(file_stream,"log: ");
    va_list args;       //定义一个va_list类型的变量，用来储存单个参数
    va_start(args,cmd); //使args指向可变参数的第一个参数
    vprintf(cmd,args);  //必须用vprintf等带V的
    vfprintf(file_stream,cmd,args);  //必须用vprintf等带V的
    va_end(args);       //结束可变参数的获取
}

void CLog::warningPrintf_vp(const char *cmd, ...)
{
    //        if( !CGlobal::GetInstance()->m_bOpenLogP )
    //            return;
    printf("warning: ");
    va_list args;       //定义一个va_list类型的变量，用来储存单个参数
    va_start(args,cmd); //使args指向可变参数的第一个参数
    vprintf(cmd,args);  //必须用vprintf等带V的
    va_end(args);       //结束可变参数的获取
}

void CLog::warningPrintf_vfp(const char *cmd, ...)
{
    //        if( !CGlobal::GetInstance()->m_bOpenLogP )
    //            return;
    fprintf(file_stream,"warning: ");
    va_list args;       //定义一个va_list类型的变量，用来储存单个参数
    va_start(args,cmd); //使args指向可变参数的第一个参数
    vfprintf(file_stream,cmd,args);  //必须用vprintf等带V的
    va_end(args);       //结束可变参数的获取
}

void CLog::warningPrintf_vp_vfp(const char *cmd, ...)
{
    //        if( !CGlobal::GetInstance()->m_bOpenLogP )
    //            return;
    printf("warning: ");
    fprintf(file_stream,"warning: ");
    va_list args;       //定义一个va_list类型的变量，用来储存单个参数
    va_start(args,cmd); //使args指向可变参数的第一个参数
    vprintf(cmd,args);  //必须用vprintf等带V的
    vfprintf(file_stream,cmd,args);  //必须用vprintf等带V的
    va_end(args);       //结束可变参数的获取
}


void CLog::errorPrintf_vp(const char *cmd, ...)
{
    //        if( !CGlobal::GetInstance()->m_bOpenLogP )
    //            return;
    printf("error: ");
    va_list args;       //定义一个va_list类型的变量，用来储存单个参数
    va_start(args,cmd); //使args指向可变参数的第一个参数
    vprintf(cmd,args);  //必须用vprintf等带V的
    va_end(args);       //结束可变参数的获取
}

void CLog::errorPrintf_vfp(const char *cmd, ...)
{
    //        if( !CGlobal::GetInstance()->m_bOpenLogP )
    //            return;
    fprintf(file_stream,"error: ");
    va_list args;       //定义一个va_list类型的变量，用来储存单个参数
    va_start(args,cmd); //使args指向可变参数的第一个参数
    vfprintf(file_stream,cmd,args);  //必须用vprintf等带V的
    va_end(args);       //结束可变参数的获取
}

void CLog::errorPrintf_vp_vfp(const char *cmd, ...)
{
    //        if( !CGlobal::GetInstance()->m_bOpenLogP )
    //            return;
    printf("error: ");
    fprintf(file_stream,"error: ");
    va_list args;       //定义一个va_list类型的变量，用来储存单个参数
    va_start(args,cmd); //使args指向可变参数的第一个参数
    vprintf(cmd,args);  //必须用vprintf等带V的
    vfprintf(file_stream,cmd,args);  //必须用vprintf等带V的
    va_end(args);       //结束可变参数的获取
}
