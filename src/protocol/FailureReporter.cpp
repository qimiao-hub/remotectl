/*
 * FailureReporter.cpp
 *
 *  Created on: 2023/01/12
 *      Author: ZhangYue
 *  20230202  zhangl1490  增加直接用32位FailureCode上报的接口
 */

#include <pthread.h>
#include <thread>
#include "FailureReporter.h"


FailureReporter::FailureReporter()
{
    // TODO Auto-generated constructor stub
}

FailureReporter::~FailureReporter()
{
    // TODO Auto-generated destructor stub
}

bool FailureReporter::init()
{
    printf("[FailureReporter] init.\n");
    m_status = SelfInspectionStatus_Initializing;

    //初始化故障状态表保护锁
    pthread_mutex_init(&m_FailureStatusLocker, NULL);

    //故障收集，topic创建
    TopicDataType * TopicFailureCollection = new FailureCollectionPubSubType();
    printf("debug-01\n");
    //创建发布者
    m_pubFailureCollection = new ddsPublisher<FailureCollection>(TopicFailureCollection);
    printf("debug-02\n");
    //开机自检响应，topic创建
    TopicDataType * TopicSelfInspectionResp = new SelfInspectionRespPubSubType();
    //创建发布者
    m_subSelfInspectionResp = new ddsPublisher<SelfInspectionResp>(TopicSelfInspectionResp);

    m_status = SelfInspectionStatus_initialized;

    return true;
}

void FailureReporter::deinit()
{

}

bool FailureReporter::start()
{
    printf("[FailureReporter] start.\n");
    //开机自检请求，topic创建
    TopicDataType * TopicSelfInspectionReq = new SelfInspectionReqPubSubType();
    //创建订阅者
    ddsSubscriber<SelfInspectionReq>* subSelfInspectionReq = new ddsSubscriber<SelfInspectionReq>(TopicSelfInspectionReq, std::bind(&FailureReporter::receiveSelfInspectionRequest, this, std::placeholders::_1));

    m_SelfInspectionSn = 0xFFFFFFFF;
    m_status = SelfInspectionStatus_ReadyToWork;
}

void FailureReporter::stop()
{
    m_status = SelfInspectionStatus_initialized;
    m_SelfInspectionSn = 0xFFFFFFFF;
}

void FailureReporter::receiveSelfInspectionRequest(SelfInspectionReq &data)
{
    printf("[FailureReporter] receiveSelfInspectionRequest: sn(%d).\n",data.sn());
    //正常工作后才回复
    if (SelfInspectionStatus_ReadyToWork != m_status)
    {
        return;
    }
    //开机自检消息流水号，故障诊断从1开始，故障上报源从0xFFFFFFFF开始，当故障上报源收到不连续值时，需要回应
    if (data.sn() == (m_SelfInspectionSn + 1))
    {
        //更新流水号
        m_SelfInspectionSn = data.sn();
        return;
    }
    m_SelfInspectionSn = data.sn();

    //通过“开机自检响应”topic上报
    SelfInspectionResp selfInspResp;
    selfInspResp.failuremodule(m_module);
    selfInspResp.status(m_status);
    //上报开机自检响应
    m_subSelfInspectionResp->publish(selfInspResp);
    printf("[FailureReporter] receiveSelfInspectionRequest: response module(%d) status(%d).\n", m_module, m_status);


    pthread_mutex_lock(&m_FailureStatusLocker);

    //开机自检结束，开始正常即时上报故障
    m_sendMsgFlag = true;
    //遍历故障，上报所有存在的故障
    for (auto iter=m_FailureStatus.begin();  iter!=m_FailureStatus.end(); iter++)
    {
        if (false == iter->second)
        {
            continue;
        }
        else
        {
            printf("[FailureReporter] receiveSelfInspectionRequest: failurecode(%d) true.\n", iter->first);
            sendFailureReport(iter->first, true);
            continue;
        }
    }

    pthread_mutex_unlock(&m_FailureStatusLocker);

    return;
}


bool FailureReporter::report(uint32_t module, uint32_t spn, uint32_t fmi, bool active)
{
    printf("[FailureReporter] report: (%d, %d, %d, %d).\n", module, spn, fmi, active);
    uint32_t failureCode = 0;
    //获取failureCode
    if (false == failurecodeCombination(module, spn, fmi, failureCode))
    {
        return false;
    }
    /*20230202  zhangl1490  增加直接用32位FailureCode上报的接口, Start*/
    return reportByFailurecode(failureCode, active);
    /*20230202  zhangl1490  增加直接用32位FailureCode上报的接口, End*/
}

/*20230202  zhangl1490  增加直接用32位FailureCode上报的接口, Start*/
bool FailureReporter::reportByFailurecode(uint32_t failureCode, bool active)
{
    printf("[FailureReporter] reportByFailurecode: rcv(%d, %d).\n", failureCode, active);

    pthread_mutex_lock(&m_FailureStatusLocker);

    /*检查：该故障是否上报过*/
    std::map<uint32_t, bool>::iterator iterFailureStatus;
    iterFailureStatus = m_FailureStatus.find(failureCode);
    /*没有记录，没上报过*/
    if (iterFailureStatus == m_FailureStatus.end())
    {
        if (true == active)
        {
            //添加记录
            m_FailureStatus[failureCode] = active;

            printf("[FailureReporter] reportByFailurecode: report(%d, %d).\n", failureCode, active);
            //发送消息通知故障管理模块
            sendFailureReport(failureCode, active);

            pthread_mutex_unlock(&m_FailureStatusLocker);
            return true;
        }
        //如果没有记录，当前收到的是清除，认为是无效信息，抛弃掉，结束流程

        pthread_mutex_unlock(&m_FailureStatusLocker);
        return true;
    }
    //有过记录，但可能已经清除了
    else
    {
        //与当前记录一致，无需上报
        if (active == iterFailureStatus->second)
        {
            return true;
        }
        else
        {
            //更新本地维护的状态
            iterFailureStatus->second = active;

            printf("[FailureReporter] reportByFailurecode: report(%d, %d).\n", failureCode, active);
            //发送消息通知故障管理模块
            sendFailureReport(failureCode, active);

            pthread_mutex_unlock(&m_FailureStatusLocker);
            return true;
        }
    }

    //不会走到这里

    pthread_mutex_unlock(&m_FailureStatusLocker);

    return false;
}
/*20230202  zhangl1490  增加直接用32位FailureCode上报的接口, End*/

bool FailureReporter::setModule(uint32_t module)
{
    printf("[FailureReporter] setModule: (%d).\n", module);
    if ((uint32_t)FailureModule_MAX <= module)
    {
        return false;
    }
    //保存模块名称
    m_module = (FailureModule_Enum)module;

    return true;
}

bool FailureReporter::failurecodeCombination(uint32_t module, uint32_t spn, uint32_t fmi, uint32_t &failureCode)
{
    //module最多8位，SPN最多19位，FMI最多5位
    if ((0xFF < module) || (0x7FFFF < spn) || (0x1F < fmi))
    {
        return false;
    }
    failureCode = 0;
    uint32_t temp = 0;

    //module占用0~7位
    temp = (module << 24) & 0xFF000000;
    failureCode = failureCode | temp;

    //spn占用8-26位
    temp = (spn << 5) & 0x00FFFFE0;
    failureCode = failureCode | temp;

    //fmi占用27-31位
    temp = fmi & 0x0000001F;
    failureCode = failureCode | temp;

    return true;
}

bool FailureReporter::sendFailureReport(uint32_t failureCode, bool active)
{
    //如果尚未完成开机自检，无需发送消息
    if (false == m_sendMsgFlag)
    {
        return true;
    }

    uint32_t u32Active;
    //通过“故障上报”topic上报
    FailureCollection failureToReport;
    failureToReport.failureCode(failureCode);
    if (true == active)
    {
        u32Active = 0x01;
    }
    else
    {
        u32Active = 0x00;
    }
    failureToReport.activeFlag(u32Active);
   
    //上报启动超时故障
    m_pubFailureCollection->publish(failureToReport);
    printf("[FailureReporter] sendFailureReport: send(%d %d).\n", failureCode, u32Active);
    return true;
}


