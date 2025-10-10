/*!
 * (C) 2017 CHIV
 */
#ifndef _SimpleThread_chiv_h
#define _SimpleThread_chiv_h

#include <pthread.h>
#include <sys/prctl.h>
#include <stdio.h>
#include <assert.h>

namespace common {

class SimpleThread {
public:
    SimpleThread() : mShouldExit(false), mThr(0)
    {
        pthread_attr_init(&mAttr);
    }
    
    virtual ~SimpleThread()
    {
        Stop();
    }
    
    virtual bool Start()
    {
//        int ret = pthread_create(&mThr, NULL, thread, this);
        int ret = pthread_create(&mThr, &mAttr, thread, this);
        get_thread_policy(&mAttr);
        get_thread_priority(&mAttr);
        show_thread_priority(&mAttr,SCHED_RR);
        return ret == 0;
    }
    
    virtual void Stop()
    {
        void *retv;
        mShouldExit = true;
        if (mThr) {
            pthread_join(mThr, &retv);
            mThr = 0;
        }
    }
    
    virtual int Run() = 0;
    
    bool ShouldExit() const
    {
        return mShouldExit;
    }
    
    static void *thread(void *usr)
    {
        SimpleThread *t = static_cast<SimpleThread *>(usr);
        unsigned long r = t->Run();
        
        return (void *)r;
    }
    
    static int get_thread_policy(pthread_attr_t *attr)
    {
      int policy;
      int rs = pthread_attr_getschedpolicy(attr,&policy);
      assert(rs==0);
      switch(policy)
      {
      case SCHED_FIFO:
        printf("policy= SCHED_FIFO\n");
        break;
      case SCHED_RR:
        printf("policy= SCHED_RR\n");
        break;
      case SCHED_OTHER:
        printf("policy=SCHED_OTHER\n");
        break;
      default:
        printf("policy=UNKNOWN\n");
        break;
      }
      return policy;
    }

    static void show_thread_priority(pthread_attr_t *attr,int policy)
    {
      int priority = sched_get_priority_max(policy);
      assert(priority!=-1);
      printf("max_priority=%d\n",priority);
      priority= sched_get_priority_min(policy);
      assert(priority!=-1);
      printf("min_priority=%d\n",priority);
    }

    static int get_thread_priority(pthread_attr_t *attr)
    {
      struct sched_param param;
      int rs = pthread_attr_getschedparam(attr,&param);
      assert(rs==0);
      printf("priority=%d\n",param.__sched_priority);
      return param.__sched_priority;
    }

    static void set_thread_policy(pthread_attr_t *attr,int policy)
    {
      int rs = pthread_attr_setschedpolicy(attr,policy);
      assert(rs==0);
      get_thread_policy(attr);
    }
protected:
    
    bool mShouldExit;
    pthread_t mThr;
    pthread_attr_t mAttr;
    struct sched_param mParam;
};

}

#endif
