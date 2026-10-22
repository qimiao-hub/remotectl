#include "global.h"


void CGlobal::swapStreamEndian(u8 *pData, s32 len)
{
    for(int i=0; i<len; i+=4)
    {
        if(len > i+4)
            swapEndian(pData,i,4);
        else
            swapEndian(pData,i,len-i);
    }
}

void CGlobal::swapEndian(u8 *pData, s32 start_index, s32 length)
{
    s32 i,cnt,end,start;
    cnt = length / 2;
    start = start_index;
    end  = start_index + length - 1;
    u8 tmp;
    for (i = 0; i < cnt; i++)
    {
        tmp            = pData[start+i];
        pData[start+i] = pData[end-i];
        pData[end-i]   = tmp;
    }
}

/**************************************************************************
* 函数名称:      异或校验和
* 描述:        该函数用于对数据进行异或校验求和
* 参数:         u8 *pu8DataBuf          字符串的地址
              s32 s32auhXorSize       字符的长度
*
* 返回值:       u8XorByte
*
***************************************************************************/
u8 CGlobal::protoCalcXOR( u8 *pu8DataBuf , s32 s32auhXorSize)
{
    u8 u8XorByte = 0;;
    s32	s32Index = 0;;
    u8XorByte = pu8DataBuf[0];


    for(s32Index = 1; s32Index < s32auhXorSize; s32Index++)
    {
        u8XorByte ^= pu8DataBuf[s32Index];
    }

    return u8XorByte;
}

float CGlobal::getGearFactor(s32 trans_current_gear)
{
    float gearFactor = 0;

    switch(trans_current_gear)
    {
    case 0:
        gearFactor = 0;
        break;
    case 1:
        gearFactor = PubApi::vehicle_trans_ratio_D1;
        break;
    case 2:
        gearFactor = PubApi::vehicle_trans_ratio_D2;
        break;
    case 3:
        gearFactor = PubApi::vehicle_trans_ratio_D3;
        break;
    case 4:
        gearFactor = PubApi::vehicle_trans_ratio_D4;
        break;
    case 5:
        gearFactor = PubApi::vehicle_trans_ratio_D5;
        break;
    case 6:
        gearFactor = PubApi::vehicle_trans_ratio_D6;
        break;
    case -1:
        gearFactor = PubApi::vehicle_trans_ratio_R;
        break;
    }
    return gearFactor;
}
