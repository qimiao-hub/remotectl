/*!
 * (C) 2017
 */
#ifndef _Sem_chiv_h
#define _Sem_chiv_h
#include <semaphore.h>
#include <time.h>


namespace common {

class Sem {
public:
    Sem(int init_value = 0)
    {
        sem_init(&mSem, 0, init_value);
    }
    
    bool Wait()
    {
        return sem_wait(&mSem) == 0;
    }
    
    void setZero()
    {
        sem_init(&mSem, 0, 0);
    }

    long getSemNum()
    {
        return mSem.__align;
    }

    bool Post()
    {
        return sem_post(&mSem) == 0;
    }

    int sem_timedwait_millsecs(long msecs)
    {
        struct timespec ts;

        clock_gettime(CLOCK_REALTIME, &ts);
        long secs = msecs/1000;
        msecs = msecs%1000;

        long add = 0;
        msecs = msecs*1000*1000 + ts.tv_nsec;
        add = msecs / (1000*1000*1000);
        ts.tv_sec += (add + secs);
        ts.tv_nsec = msecs%(1000*1000*1000);
        return sem_timedwait(&mSem, &ts);
    }


    
private:
    sem_t mSem;
};

}

#endif
