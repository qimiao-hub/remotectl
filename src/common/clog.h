#ifndef CLOG_H
#define CLOG_H

#include <stdio.h>
#include <ctime>
#include <fcntl.h>
#include <unistd.h>
#include <stdarg.h>

namespace common {

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

void logC(const char *func, const char *file, const int line,
          const char *type, const char *format, ...);
#define LOGC(type, format, ...) logC(__func__, __FILENAME__, __LINE__, type, format, ##__VA_ARGS__)

#define LOG_NULL_MOD  0x0
#define LOG_STDOUT_MOD  0x1
#define LOG_FILE_MOD    0x2

#define LOG_MSG_TYPE            "MSG"
#define LOG_DEBUG_TYPE          "DEBUG"
#define LOG_WARNING_TYPE        "WARNING"
#define LOG_ERROR_TYPE          "ERROR"

#define DEBUG_DATA_NUM          0xf

#define LOG_DATA_WRITE          0

class CLog {
public:
    CLog() {}


    static void (*logPrintf)(const char *,...);
    static void (*warningPrintf)(const char *,...);
    static void (*errorPrintf)(const char *,...);

    static int logStart(const char* log_file_name,int mode);
    static int logSwitch(bool bLog,bool bWarn,bool bError);
    static void logEnd();

    static void saveData(int index,const char *cmd, ...);
private:
    static int logOpen(const char* log_file_name);
    static void logclose();

    static void logPrintf_null(const char *cmd, ...);
    static void logPrintf_vp(const char *cmd, ...);
    static void logPrintf_vfp(const char *cmd, ...);
    static void logPrintf_vp_vfp(const char *cmd, ...);

    static void warningPrintf_null(const char *cmd, ...);
    static void warningPrintf_vp(const char *cmd, ...);
    static void warningPrintf_vfp(const char *cmd, ...);
    static void warningPrintf_vp_vfp(const char *cmd, ...);

    static void errorPrintf_null(const char *cmd, ...);
    static void errorPrintf_vp(const char *cmd, ...);
    static void errorPrintf_vfp(const char *cmd, ...);
    static void errorPrintf_vp_vfp(const char *cmd, ...);
public:
    static FILE* file_stream;
    static FILE* data_stream[DEBUG_DATA_NUM];
    static int log_mode;
private:



};
}

#endif // CLOG_H
