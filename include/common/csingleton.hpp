#ifndef CSINGLETON
#define CSINGLETON

namespace common {

#define VNAME(name) (#name)


template <class T>

class CSingleton
{
public:
    CSingleton<T>()
    {
    }
    static T* GetInstance()
    {
        if(mInstance == 0)
        {
            mInstance = new T();
        }
        return mInstance;
    }
    static T* getInstance()
    {
        return GetInstance();
    }
private:
    static T* mInstance;
};

template <class T> T* CSingleton<T>::mInstance = 0;
}


#endif // CSINGLETON
