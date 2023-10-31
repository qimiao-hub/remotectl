/*
 * FailureReporter.h
 *
 *  Created on: 2023/01/12
 *      Author: ZhangYue
 *  20230202  zhangl1490  增加直接用32位FailureCode上报的接口
 *  20230323  zhangl1490  拆分idl，减少SDK更新
 */

#ifndef FAILUREREPORTER_H_
#define FAILUREREPORTER_H_

/*20230323  zhangl1490  拆分idl，减少SDK更新, Start*/
#include "./FailureReportType/FailureReporterInterface.h"
#include "./FailureReportType/FailureReporterInterfacePubSubTypes.h"
/*20230323  zhangl1490  拆分idl，减少SDK更新, End*/
#include <zkbs/ddsBase/ddsPubBase.h>
#include <zkbs/ddsBase/ddsSubBase.h>

#include <stdint.h>
#include <bits/stl_map.h>

class FailureReporter
{

public:
    FailureReporter();
    virtual ~FailureReporter();
    bool init();
    void deinit();
    bool start();
    void stop();

public:
    //对外接口，调用该接口上报故障
    bool report(uint32_t module, uint32_t spn, uint32_t fmi, bool active);
    /*20230202  zhangl1490  增加直接用32位FailureCode上报的接口, Start*/
    bool reportByFailurecode(uint32_t failurecode, bool active);
    /*20230202  zhangl1490  增加直接用32位FailureCode上报的接口, End*/
    //调试接口，设置模块名称，用于回应开机自检响应
    bool setModule(uint32_t module);

private:
    //开机自检请求
    void receiveSelfInspectionRequest(SelfInspectionReq &data);
    //拼接故障编号
    bool failurecodeCombination(uint32_t module, uint32_t spn, uint32_t fmi, uint32_t &failureCode);
    //发送故障，需保证m_FailureStatusLocker已锁  ！  ！  ！
    bool sendFailureReport(uint32_t failureCode, bool active);

private:
    //模块名称，各模块需修改该值  ！  ！  ！
    FailureModule_Enum m_module = FailureModule_Unknow;

    //当前模块状态
    SelfInspectionStatus_Enum m_status = SelfInspectionStatus_Unknow;
    //开机自检消息流水号，故障诊断从1开始，故障上报源从0xFFFFFFFF开始，当故障上报源收到不连续值时，需要回应
    uint32_t m_SelfInspectionSn = 0xFFFFFFFF;

    //故障状态涉及多线程操作，主线程和回调函数可能调用，需要加锁保护
    pthread_mutex_t m_FailureStatusLocker;
//m_FailureStatusLocker保护范围，Start
    //本地维护的故障状态表，key是SPN和FMI通过getKeyFromSpnAndFmi转换而来，value代表故障 存在/ 已清除
    std::map<uint32_t, bool> m_FailureStatus;
    //故障即时上报标志位，发送开机自检响应之后更新为true
    bool m_sendMsgFlag = false;
//m_FailureStatusLocker保护范围，End

    //故障收集Topic，发布者
    ddsPublisher<FailureCollection> *m_pubFailureCollection = NULL;
    //开机自检响应Topic，发布者
    ddsPublisher<SelfInspectionResp> *m_subSelfInspectionResp = NULL;
};


#endif /* FAILUREREPORTER_H_ */
