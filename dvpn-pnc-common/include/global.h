#ifndef CGOLBAL_H
#define CGOLBAL_H

#include "csingleton.hpp"
#include "type.h"
#include "pubapi.h"

using namespace common;


enum E_FAULT_LEVEL
{
    E_0_FAULT_LEVEL,
    E_1_FAULT_LEVEL,
    E_2_FAULT_LEVEL,
    E_3_FAULT_LEVEL,

    E_FAULT_LEVEL_END,
};

class CGlobal:public CSingleton<CGlobal>
{
public:
    CGlobal() {}
    void swapStreamEndian(u8 *pData, s32 len);

    float getGearFactor(s32 trans_current_gear);
    u8 protoCalcXOR( u8 *pu8DataBuf , s32 s32auhXorSize);
private:
    void swapEndian(u8 *pData, s32 start_index, s32 length);
};

#endif
