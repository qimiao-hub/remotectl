#ifndef CTIMER
#define CTIMER

#include <ctime>
#include <sys/time.h>
#include <stdio.h>
#include <iostream>
#include <fstream>

using namespace std;


namespace common {
class CTimer
{
public:
    double duration;
    CTimer(string str)
    {
        duration = 0.0;
        str_name = str;
    }

    void start()
    {
        gettimeofday(&tv,NULL);
        duration_start = tv.tv_sec*1000+tv.tv_usec*0.001;
    }
    void end()
    {
        gettimeofday(&tv,NULL);
        duration_end = tv.tv_sec*1000+tv.tv_usec*0.001;
        duration = duration_end - duration_start;
//        CLog::logPrintf("TIME: %s:%f ms\n",str_name.data(),duration);
//        printf("TIME: %s:%f ms\n",str_name.data(),duration);
    }

    static void delay(int sec)
    {
        time_t start_time, cur_time; // 变量声明
        time(&start_time);
        do {
            time(&cur_time);
        } while((cur_time - start_time) < sec );
    }

    static void Delay(int mtime)//time*1000为秒数
    {
        clock_t now = clock();

        while(clock() - now < mtime);
    }

private:
    struct timeval tv;
    string str_name;
    double duration_start;//ms
    double duration_end;
};
}

#endif // CTIME
