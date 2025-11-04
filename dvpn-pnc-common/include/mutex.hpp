/*!
 * (C) 2017 CHIV
 *
 * Mutex with posix API
 */
#ifndef _Mutex_chiv_h
#define _Mutex_chiv_h

#include <semaphore.h>

namespace common {

class Mutex {
public:
    Mutex()
    {
        sem_init(&mSem, 0, 1); /** binary semaphore */
    }
    
    bool Lock()
    {
        return sem_wait(&mSem) == 0;
    }
    
    bool Unlock()
    {
        return sem_post(&mSem) == 0;
    }
    
private:
    
    sem_t mSem;
};

class AutoLocker {
public:
    AutoLocker(Mutex &lock) : mLock(lock)
    {
        lock.Lock();
    }
    
    ~AutoLocker()
    {
        mLock.Unlock();
    }
    
    Mutex &mLock;
};

}

#endif
