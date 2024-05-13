/**
************************************************************
* @file         PROTODEF.h
* @brief        协议结构头文件
* @author       sany-Yanhao
* @date         2020-12-07
* @version      V0.0.1
* @copyright    sany
***********************************************************/
#ifndef PROTODEF_H
#define PROTODEF_H
/*********************************************************************
 * INCLUDES
 */ 
#include "../common/sany_type.h"

/*********************************************************************
 * MACROS
 */
#define WP_VIN_SIZE 17

#define SYS2NET(ray,val)            (ray)[0] = ((val) / 256); (ray)[1] = ((val) & 0xFF)
#define SYS2NET4(ray)               ((((SANY_U32)(ray)[0]) * 256 * 256 * 256) + (((SANY_U32)(ray)[1]) * 256*256) + (((SANY_U32)(ray)[2]) * 256) + (ray)[3])
#define ARRAY2SHORT(ray)            ((((SANY_U16)(ray)[1]) * 256) + (ray)[0])
#define ARRAY2INT(ray)              ((((SANY_U32)(ray)[3]) * 256 * 256 *256) + (((SANY_U32)(ray)[2]) * 256 *256) + (((SANY_U32)(ray)[1]) * 256) + (ray)[0])

#define NET2SYS(ray)                ((((SANY_U16)(ray)[0]) * 256) + (ray)[1])

//#define OFFSET_VEHICLE_ANGLE        400

#define OFFSET_VEHICLE_ANGLE_1      800
#define OFFSET_VEHICLE_ANGLE_2      40

#define OFFSET_VEHICLE_HEADING      180
#define OFFSET_VEHICLE_PITCH        90
#define OFFSET_VEHICLE_POLL         180


/******************************三一智矿云端控制协议命令标识汇总*********************************/
enum{
    EV_WP_VEHICLE_REGISTER = 1,             //Vehicle register
    EV_WP_VEHICLE_SIGNOUT,                  //Vehicle sign out
    EV_WP_INFO_REALTIME,                    //Real-time information reporting
    EV_WP_INFO_BIGDATA,                     //Infomation of big data reporting
    EV_WP_INFO_RECORDS,                     //Historical information reporting
    EV_WP_INFO_FAULT,                       //Fault information reporting
    EV_WP_INFO_ALARM,                       //Alarm information reporting
    EV_WP_INFO_VEHICLETERMINAL,             //Vehicle terminaml information reporting
    EV_WP_INFO_WORKING,                     //Working information reporting
    EV_WP_INFO_NORMALDATA,                  //Normal Data reporting   20200128 update
    EV_WP_INFO_VARDATA,                     //Variable Data reporting 20200128 update

    EV_WP_COR_REQ = 0x10,                   //Vehicle collaboration request
    EV_WP_EXCAVATOR_POS_REQ,                //Excavator position request
    EV_WP_DEVICE_MONITOR,                   //Other device information monitor

    EV_WP_HEARTBEAT = 0x20,                 //heart beat
    EV_WP_TERMINALTIMECAL,                  //Terminal time calibration

    EV_WP_DOWN_PARAQUERY = 0x30,            //Parameter query cmd
    EV_WP_DOWN_PARASETCMD,                  //Parameter seting cmd
    EV_WP_UP_PARASETACK,                    //Parameter seting finish ack
    EV_WP_DOWN_ALARMPARAQUERY,              //Alarm parameter query cmd
    EV_WP_DOWN_ALARMPARASETCMD,             //Alarm parameter seting cmd
    EV_WP_UP_ALARMPARASETACK,               //Alarm parameter seting finish ack
    EV_WP_DOWN_OTA_BEGIN,                   //OTA begin message
    EV_WP_DOWN_OTA_FINISH,                  //OTA finish message

    EV_WP_SEND_TASK = 0x40,                 //Cloud send a task
    EV_WP_SEND_SURROUNDINFO,                //Cloud send surrounding information
    EV_WP_SEND_EMERGENCYINFO,               //Cloud send Emergency information
    EV_WP_REMOTE_VEHICLE,                   //Vehicle remote

    //EV_WP_V2S_REGISTER = 0x56,            //Vehicle register to simulator
    //EV_WP_V2S_SIGNOUT,                    //Vehicle sign out from simulator
    //EV_WP_V2S_HEARTBEAT,                  //Heart beat(V2S)
    //EV_WP_S2V_CONTROL,                    //Simulator control vehicle
    //EV_WP_V2S_STATEINFO,                  //Upload vehicle state to simulator
    EV_WP_REQ_REMOTE_CTRL = 0x5B,               //Vehicle require to be control
    EV_WP_COCKPIT_INFO,                      //Simulator server info

    EV_WP_CTRL_REQUESTRUN = 0x60,           //Cloud request vehicle to run
    EV_WP_CTRL_DIAGNOSIS,                   //Remote diagnosis vehicle
    EV_WP_ABNORMAL_WAKEUP,                  //Abnormal wake-up feedback
    
    EV_WP_CUSTOMDATA_UPLINK = 0x70,         //Transparent transmission of custom uplink data
    EV_WP_CUSTOMDATA_DOWNLINK,              //Transparent transmission of custom downlink data

    EV_WP_DATASYSTEM_UPLINK = 0x80,         //Uplink data system

    EV_WP_DATASYSTEM_DOWNLINK = 0x90,       //Downlink data system

    EV_WLAPP_MAX                           
}Protocol_Cmd_Enum;


/******************************三一智矿驾驶舱控制协议命令标识汇总*********************************/
enum{
    EV_CP_VEHICLE_REGISTER = 0x01,          //车端登入驾驶舱端
    EV_CP_INFO_REALTIME_REP,              	//实时上报（高频数据）
    EV_CP_REQ_CTRL,							//控车请求
    EV_CP_REALTIME_CTRL,					//实时控制
	// EV_CP_STATUS_REALTIME_REP,				//实时上报（状态变更）
    EV_CP_HEART_BEAT,                       //心跳
	EV_CP_SEND_EMERGENCY_STOP = 0x06,     	//紧急停车
    EV_CP_VEHICLE_SIGNOUT = 0x08,          	//登出驾驶舱端（主动、被动）
    
    EV_CLAPP_MAX                           
}Protocol_Cockpit_Cmd_Enum;

/*************************通用协议消息头结构体**************************/
typedef struct  {
    SANY_U8	ucHeader[2];                    //Start char
    SANY_U8	ucCommand[1];                   //Command Flag
    SANY_U8	ucAckflg[1];                    //Ack Flag;
    SANY_U8	ucVIN[WP_VIN_SIZE];             //Vehicle VIN;
    SANY_U8	ucEncryption[1];				//0x01 NO ; 0x02 RSA ; 0x03 AES128; 0xFF invalid ;
    SANY_U8	ucDataByteSize[2];              //Data unit size; 
    SANY_U8  ucData[0];
}CP_Header_st;

/***********************车-舱协议*********登入命令*********消息结构体************************/
typedef struct {
    SANY_U8	ucTime[8];				//指令时间
    SANY_U8	ucSequenceNum[1];		//登入流水号
    SANY_U8	ucIccid[20];			//ICCID
    SANY_U8	ucHardVer[4];			//硬件版本
    SANY_U8	ucSoftVer[4];			//软件版本
    SANY_U8 ucWorkStatus[1];        //车辆工作状态
    SANY_U8 ucCurrentGear[1];       //当前档位
} CP_Register_Cmd_st;


/***********************车-舱协议*********登入响应*********消息结构体************************/
typedef struct {
    SANY_U8	ucTime[8];				//指令时间
    SANY_U8	ucSequenceNum[1];		//响应流水号
    /*Bit7：0：ICCID匹配	1：ICCID不匹配
	Bit6：0：硬件类型匹配  1：硬件类型不匹配
	Bit5：0：硬件版本号匹配  1：硬件版本号不匹配
	Bit4：0：软件类型匹配  1：软件类型不匹配
	Bit3：0：软件版本号匹配  1：软件版本号不匹配*/
	SANY_U8	ucLoginStatus[1];		//登入状态
    SANY_U8	ucSecretKey[915];		//公/私钥
} CP_Register_Ack_st;

/***********************车-舱协议*********登出命令*********消息结构体************************/
typedef struct {
    SANY_U8	ucTime[8];				//指令时间
    SANY_U8	ucSequenceNum[1];		//流水号（与登入流水号一致）
    SANY_U8	ucloginOutFlag[1];		//登出标志 0x01主动登出；0x02被动登出
} CP_LoginOut_Cmd_st;

/***********************车-舱协议*********登出响应*********消息结构体************************/
typedef struct {
    SANY_U8	ucTime[8];				//指令时间
} CP_LoginOut_Ack_st;


/***********************车-舱协议*********控车请求命令*********消息结构体************************/
typedef struct {
    SANY_U8	ucTime[8];				//指令时间
    SANY_U8	ucSequenceNum[1];		//指令流水号
    SANY_U8	ucCtrlSwitch[1];		//遥控接管开关
} CP_Req_Ctrl_Cmd_st;

/***********************车-舱协议*********控车请求响应*********消息结构体************************/
typedef struct {
	SANY_U8	ucTime[8];				//指令时间
	SANY_U8	ucSequenceNum[1];		//响应流水号
	SANY_U8	ucCtrlStatus[1];		//控车状态
} CP_Req_Ctrl_Ack_st;

/***********************车-舱协议*********实时控制命令*********消息结构体************************/
typedef struct {
    SANY_U8	ucTime[8];				//指令时间
    SANY_U8	ucSequenceNum[1];		//响应流水号
    SANY_U8	ucWheelAngle[2];		//车轮转角
    SANY_U8	ucThrottleOpening[1];	//油门开度
    SANY_U8	ucBrakeOpening[1];		//制动开度
    SANY_U8	ucGearOpening[1];		//档位操作
    SANY_U8	ucCombinedGearL[1];		//组合开关左
    SANY_U8	ucCombinedGearR[1];		//组合开关右
    SANY_U8	ucKeyOperation1[1];		//按键操作1
    SANY_U8	ucKeyOperation2[1];		//按键操作2
} CP_Realtime_Ctrl_Cmd_st;

/***********************车-舱协议*********紧急停车命令*********消息结构体************************/
typedef struct {
    SANY_U8	ucTime[8];				//指令时间
    SANY_U8	ucSequenceNum[1];		//指令流水号
    SANY_U8	ucStopFlag[1];			//0x00解除急停；0x01紧急停车
} CP_Emergency_Stop_Cmd_st;

/***********************车-舱协议*********紧急停车响应*********消息结构体************************/
typedef struct {
	SANY_U8	ucTime[8];				//指令时间
	SANY_U8	ucSequenceNum[1];		//响应流水号
} CP_Emergency_Stop_Ack_st;

//V2.2.0新增车舱心跳
/***********************车-舱协议*********车舱心跳命令*********消息结构体************************/
typedef struct {
	SANY_U8	ucTime[8];				//指令时间
	SANY_U8	ucSequenceNum[2];		//流水号
} CP_HeartBeat_Cmd_st;

/***********************车-舱协议*********车舱心跳响应*********消息结构体************************/
typedef struct {
	SANY_U8	ucTime[8];				//指令时间
	SANY_U8	ucSequenceNum[2];		//响应流水号
} CP_HeartBeat_Ack_st;

/***********************车-舱协议*********实时上报(高频)命令*********消息结构体************************/
typedef struct {
    SANY_U8	ucTime[8];					//指令时间
    SANY_U8	ucCarSpeed[2];				//车速
    SANY_U8	ucHeadingAngle[2];			//航向角
    SANY_U8	ucWheelAngleL[2];			//左轮转角
    SANY_U8	ucThrottleOpening[1];		//油门开度
    SANY_U8	ucBrakeOpening[1];			//制动开度
    SANY_U8	ucEngineSpeed[2];			//发动机转速
    SANY_U8	ucAccumulatedMileage[4];	//累计里程
    SANY_U8	ucSubtotalMileage[2];		//小计里程
    SANY_U8 ucPitchAngle[2];            //俯仰角
    // Version 2.2.0  取消状态变更及实时控制响应，合并至实时上报
    SANY_U8	ucGearOper[1];				//档位操作
    SANY_U8	ucRemainingOil[1];			//剩余油、电量
    SANY_U8	ucWaterTemp[1];				//水温
    SANY_U8	ucBrakeAirPre1[1];			//制动气压1
    SANY_U8	ucBrakeAirPre2[1];			//制动气压2
    SANY_U8	ucCombinedGearL[1];			//组合开关左
    SANY_U8	ucCombinedGearR[1];			//组合开关右
    SANY_U8	ucKeyOperation1[1];			//按键操作1
    SANY_U8	ucKeyOperation2[1];			//按键操作2 
    SANY_U8	ucWorkStatus[1];			//车辆驾驶状态
    SANY_U8 ucLiftUpStatus[1];         //举升到位状态
    SANY_U8 ucLiftDownStatus[1];        //下降到位状态
} CP_Realtime_Rep_Cmd_st;

/******************************协议通用枚举*********************************/
enum{
    EV_WP_ACK = 0x01,
    EV_WP_NACK,
    EV_WP_VIN_REP,

    EV_WP_CMD = 0xFE
}Protocol_Ack_Flg_Enum;

enum{
    EV_WP_NO_ENCRYPT = 0x01,
    EV_WP_RSA,
    EV_WP_AES128,

    EV_WP_ABNORMAL = 0xFE,
    EV_WP_INVAILD
}Protocol_Encrypt_Enum;

typedef enum
{
    COMM_ANSWER_OK = 0X1,
    COMM_ANSWER_ERROR,

    COMM_ANSWER_INVALID = 0xFF
}Msg_Comm_Answer_Enum;



/***************************协议8.4 实时信息上报枚举*****************************/
enum{
    EV_MI_VEHICLE_HIFREQ_DATA = 0x1,       //整车高频数据
    EV_MI_LOCATION_DATA,                   //定位数据
    EV_MI_SENSOR_DATA,                     //传感器感知数据
    EV_MI_CANBUS_DATA,                     //总线报文数据
    EV_MI_CCU_DATA,                        //自动驾驶控制器数据
    EV_MI_MAX
}RealTime_CmdID_Enum;

/***************************协议8.4.2.1 自动驾驶状态*****************************/
typedef enum
{
    Auto_NormalRoadRun = 1,                //正常道路行走
    Auto_NormalParkRun,                    //正常泊车行走
    Auto_NormalStop,                       //正常停车
    Auto_ObstacleStop,                     //遇障停车
    Auto_ObstacleRun,                      //遇障行车
    Auto_MalfunctionStop,                  //故障停车
    Auto_WaitUpload,                       //等待装料
    Auto_Uploading,                        //装料执行中
    Auto_WaitDump,                         //等待卸料
    Auto_Dumping,                          //卸料执行中
    Auto_WaitTask,                         //等待任务
    Auto_WaitUploadEnter,                  //等待装料入场
    Auto_WaitDumpEnter,                    //等待卸料入场
    
    Auto_Others = 0xFE,
    Auto_Invaild = 0xFF
}AutoStaus_Enum;

/***************************协议8.4.2.1 装载状态*****************************/
typedef enum
{
    Load_Empty = 1,                        //空载
    Load_Full,                             //满载

    Load_Abnormal = 0xFE,
    Load_Invaild = 0xFF
}LoadState_Enum;

/***************************协议8.4.2.1 举升状态*****************************/
typedef enum
{
    Lift_Uping = 1,                        //升起执行中
    Lift_Downing,                          //降落执行中
    Lift_UpFinish,                         //完全升起状态
    Lift_DownFinish,                       //恢复正常状态

    Lift_Abnormal = 0xFE,
    Lift_Invaild = 0xFF
}LiftState_Enum;

/***************************协议8.4.2.1 档位*****************************/
typedef enum 
{
    Gear_P = 1,
    Gear_R,
    Gear_D1,
    Gear_N,
    Gear_C,
    Gear_ECO,
    Gear_D2,
    Gear_D3,
    Gear_D4,
    Gear_D5,
    Gear_D6,

    Gear_Abnormal = 0xFE,
    Gear_Invaild = 0xFF
}CarGear_Enum;

/***************************协议8.9 作业信息上报枚举*****************************/
typedef enum
{
	OPERATION_DATA_FLAG = 0x01,
} Operation_Data_Flag_Enum;


/***************************协议8.10 常态信息上报枚举*****************************/
typedef enum
{
	LOWFREQ_DATA = 0x01,                   //整车低频数据
} Normal_Info_Data_Flag_Enum;



/***************************协议8.11 变量信息上报枚举*****************************/
typedef enum
{
	VARIABLE_DATA = 1,                     //整车变化数据
	CHARGE_DATA                            //充电数据
} Variable_Data_Flag_Enum;
    
/***************************协议8.11.2.1 车辆状态枚举*****************************/
typedef enum
{
    CAR_READY = 1,                         //车辆启动状态
    CAR_ACCON,                             //车辆上电
    CAR_KEYOFF,                            //熄火
    CAR_BATTPWROFF,                        //蓄电池下电

    CAR_ABNORMAL = 0xFE,
    CAR_INVALID = 0xFF
}CarStatus_Enum;

/***************************协议8.11.2.1 控制状态枚举*****************************/
typedef enum
{
    CTRL_USROPERATION = 1,                 //人工操作状态
    CTRL_AUTODRIVING,                      //自动驾驶状态
    CTRL_REMOTE,                           //远程控制
    CTRL_MAINTENANCE,                      //运维状态

    Ctrl_Abnormal = 0xFE,
    Ctrl_Invaild = 0xFF
}ControlStatus_Enum;



/***************************协议8.12设备协同枚举*****************************/
/**************************协议8.12.1 矿车协同请求****************************/
enum{
	RE_OF_PER_COM = 0x01,                   //Personnel confirmation request
    RE_CHA_ADM,                             //Charging admission request
	RE_DIS_ADM,                             //Discharge admission request
	RE_CHA,                                 //Charging request

	RE_INVALID = 0xFF                       //INVALID
}Protocol_Equip_Tog_Instruction_Code_Enum;

/************************协议8.12.1 矿车协同请求回复**************************/
enum{
	REQ_NOT_ALLOW = 0x01,
	REQ_ALLOW,

	REQ_INVALID = 0xFF
}Protocol_har_Coll_Req_Enum;

/**************************协议8.12.2 泊车位置请求****************************/
enum{
	EXCA_CEN_POS_RE = 0x01,                 //Excavator center position request
    EXCA_BUC_POS_RE,                        //Excavator bucket position request
	UNLOAD_POS_RE,                          //卸料位置请求     
	
	RE_POS_INVALID = 0xFF                   //INVALID

}Protocol_Pos_Instruction_Code_Enum;



/***************************协议8.15 任务下发枚举*****************************/
enum{
	//Task_Set_Dri_Path = 0x01,                    //Set the driving path
    //Task_Veh_Are_Not,                            //Vehicle Area notice
    //Task_Syn_Sur_Vel,                            //Synchronize surrounding vehicle information

    TASK_EMERG_STOP = 0x01,                      //emergency stop
            
    TASK_DIS_VEH_CONTROL = 0x03,                 //distance between vehicles control
    TASK_SPE_CONTROL,                            //The speed control
    TASK_TA_UPDATE,                              //Task to update
    TASK_OPER_FUN,                               //Operations function
    TASK_CALI_LOA,                               //Calibration location
	
	TASK_INVALID = 0xFF                          //INVALID
}Protocol_Task_Instruction_Coed_Enum;

/***************************协议8.15.1 紧急停车*****************************/
typedef enum
{
	VEHI_EMERG_STOP = 0x01,                      // vehicle emergency stop
	VEHI_RECOV                                   //vehicle recovery
} Vehicle_Emerg_Stop_Enum;

/***************************协议8.15.4 任务更新*****************************/
enum{
	TASK_START = 0x01,                           //TASK_START
    TASK_SUS,                                    //The task to suspend
    TASK_RES,                                    //Task to restore
    TASK_PARK_LOT,                               //Return to parking lot
    TASK_TO_TERM,                                //Task to terminate
    TASK_LOAD_FIN                                //load finished  V2.2 update 
}Protocol_Task_Ache_Enum;

/***************************协议8.15.5 运维功能*****************************/
typedef enum
{
	ENTRY_OPERA_MODE = 0x01,                     //vehicle emergency stop
	EXIT_OPERA_MODE                              //vehicle recovery
} Operation_Function_Enum;

/*************************协议8.15.6 标定地点/路线**************************/
typedef enum
{
	CALI_SWITCH_PIONT = 0x01,                    //标定转换点
	CALI_LOAD_ENTRY_POINT,                       //标定装料入场点
	CALI_LOAD_POINT,                             //标定装料点
	CALI_DUMP_ENTRY_POINT,                       //标定卸料入场点
	CALI_DUMP_POINT,                             //标定卸料点
	CALI_ROUTE                                   //标定路线
} Cali_Point_Enum;

/***********************协议8.15.6 标定地点/路线回复*************************/
enum{
	CALIBRATION_COMPE = 0x01,
	CALIBRATION_FAIL,

    CALIBRATION_INVALID = 0xFF
}Protocol_Task_Send_Cali_Enum;



/***************************协议8.16 远程车控枚举*****************************/
enum{
    REMOTE_VEHICLE_POWERON = 1,                 //车辆上电
    REMOTE_VEHICLE_AUTOPILOT_ON,                //车辆进入自动驾驶模式
    REMOTE_VEHICLE_AUTOPILOT_OFF,               //车辆退出自动驾驶模式
    REMOTE_VEHICLE_POWEROFF,                    //车辆下电
    REMOTE_VEHICLE_AUTOPILOT_FAST,              //车辆快速自动驾驶模式
    REMOTE_VEHICLE_POWEROFF_FAST,               //车辆快速下电

    REMOTE_VEHICLE_INVALID = 0xFF               //无效
}Protocol_Remote_Vehicle_Status_Enum;



/***********************协议8.19 自定义下行数据透传枚举************************/
typedef enum
{
    Downlink_TaskStart = 1,                     //传输开始
    Downlink_Download,                          //数据下发
    Downlink_TaskEnd,                           //传输结束
    Downlink_Interrupt                          //传输中断
}DownlinkMsgId_Enum;


#pragma pack(1)
/*************************双发控制结构体**************************/
typedef struct {
    SANY_U8  u8SendFlag;
    SANY_U8  u8LoginData[80];
    SANY_U8  u8SendData[100];
}MultSend_st;

/*************************通用协议消息头结构体**************************/
typedef struct  {
    SANY_U8	ucHeader[2];                    //Start char
    SANY_U8  ucProtocolVersion[1];           //Protocol version, start from 0;
    SANY_U8	ucCommand[1];                   //Command Flag
    SANY_U8	ucAckflg[1];                    //Ack Flag;
    SANY_U8	ucVIN[WP_VIN_SIZE];             //Vehicle VIN;
    SANY_U8	ucEncryption[1];				//0x01 NO ; 0x02 RSA ; 0x03 AES128; 0xFF invalid ;
    SANY_U8	ucDataByteSize[2];              //Data unit size; 
    SANY_U8  ucData[0];
}WP_Header_st;

/***********************协议6.7连接保持结构体*************************/
typedef struct{
    SANY_U8  ucTime[6];
} WP_HeartBeat_st;

/***********************协议8.1通用应答结构体************************/
typedef  struct{
   SANY_U8   ucTime[6];
}WP_ProAck_st;

/***********************协议8.2 登入消息结构体************************/
typedef struct {
    SANY_U8	ucTime[6];
    SANY_U8	ucSequenceNum[2];
    SANY_U8	ucIccid[20];
    SANY_U8  ucMiningId[2];
    SANY_U8	ucHardVer[4];
    SANY_U8	ucSoftVer[4];
    SANY_U8	ucTerminalNum[10];
    SANY_U8  ucMapVer[2];
    SANY_U8  ucRechargeSubSYSNum[1];
    SANY_U8  ucRechargeSubSYSLen[1];
    SANY_U8  ucRechargeCode[0];
} WP_Register_st;

/***********************协议8.3 登出消息结构体************************/
typedef struct {

    SANY_U8	uctime[6];			    
    SANY_U8  ucSequenceNum[2];
} WP_LOGOUT_st;


/******************协议8.4 实时信息上报消息头结构体********************/
typedef struct {
    SANY_U8	ucTime[6];
    SANY_U8	ucDataBodyNum[1];
    SANY_U8	next[0];
} WP_RT_Header_st;

/********************协议8.4.2.1 整车高频数据结构体********************/
typedef struct {
    SANY_U8	ucFlg[1];			            //message cmd ID, 0x1
    //SANY_U8	ucCarState[1];				    //car status, 1:ready; 2:ACC ON; 3:Key Off; 4:Battery power off; FE:Abnormal; FF:invaild
    //SANY_U8	ucCtrlState[1];                 //control status, 1:Manual; 2:Auto; 3.remote; 4.maintain; FE:Abnormal; FF:invaild
    SANY_U8  ucAutopilotState[1];            //Autopilot status，1：normal drive 2:normal park 3:normal stop 4:obstracl stop 5:obstracl drive 6:fault stop 
                                        //7: wait load 8:loading 9:wait dump 0x0A:dumping 0x0B:wait task 0x0C:wait entry load area 0x0D:wait entry dump area
                                        //0xFE: other status   0xFF: invalid data
    SANY_U8  ucLoadState[1];                 //Load status, 1:Unload; 2:Load; FE:Abnormal; FF:invaild
    //SANY_U8  ucLiftPwrState[1];              //Lift power status, 1:Open; 2:Close; FE:Abnormal; FF:invaild
    SANY_U8  ucLiftState[1];                 //Lift status, 1:Up(running); 2:Down(running); 3:Up(Finish); 4:Down(Finish); FE:Abnormal; FF:invaild
    SANY_U8	ucSpeed[2];                     //Car speed, range 0~5000, mean 0KM/h~500KM/H, Accuracy:0.1
    SANY_U8  ucGear[1];                      //Gear, 1: P; 2: R; 3: D1; 4: N; 5: C; 6: ECO; 7: D2; 8: D3; 9: D4; A: D5; B: D6 FE:Abnormal; FF:invaild
    SANY_U8	ucWheelAngle[2];                //Car wheel angle, range 0~800, Accuracy 1, means-40~40,   <0 means left, >0 means right
    SANY_U8	ucCarHeadAngle[2];              //Car head angle, range 0~360, Accuracy 1, means -180~180, North = 0, < 0 means left, > 0 means right
    //SANY_U8	ucBatteryPW[1];                 //Battery, rang 0~200, mean(0~40V) Accuracy 0.2V, FE: Abnormal, FF:invaild
    //SANY_U8	ucTirePressue[1];               //Tire pressue, range 0~90, mean 210KPa~300KPa, Accuracy 1KPa, 0xFE: Abnormal, 0xFF:invaild
    SANY_U8	ucMileage[4];                   //Mileage, range 0~9999999, means 0KM~999999.9KM, Accuracy 0.KM, "0xFF 0xFF 0xFF 0xFE": Abnormal, "0xFF 0xFF 0xFF 0xFF ":invaild
    SANY_U8	ucSOC[1];                       //SOC, range0~100, Accuracy 1, FE:Abnormal; FF:invaild
    SANY_U8  ucInstantaneousPower[2];        //Instantaneous Power, range 0~10000,means -500KW~500KW, Accuarcy 0.1KW
    SANY_U8  ucAveragePower[2];              //Average Power, range 0~5000, means 0KWH~500KWH, Accuracy 0.KWH
    SANY_U8  ucEngineSpeed[2];               //Engine Speed,range 0~40000, means 0 ~5000rpm, Accuracy 0x125rpm
    SANY_U8  ucEngineConsumption[4];         //oil comsumption, range0~200000000 means 0~100000000L, Accuarcy 0.5L 
    SANY_U8  ucReserved[6];                 //Reserved data 
    SANY_U8  next[0];
} WP_RT_HighFreq_VehicleData_st;;

/*********************协议8.4.2.2 定位数据结构体*********************/
typedef struct {
    SANY_U8	ucFlg[1];			        //message cmd ID, 0x2
    SANY_U8	ucLocationState[1];         //Location state confidence, range 0~10
    SANY_U8  ucLocationArea[1];          //Location Area, bit 0: 0 East longitude 1 West longitude; bit 1: 0 north latitude 1 south latitude
    SANY_U8  ucLocationTime[7];          //Location Time, Accuracy 10ms, Format: YY-MM-DD HH:MM:SS.SS
    SANY_U8  ucHeading[4];               //Yaw angle, range 0~359999,Accuracy 0.001
    SANY_U8  ucPitchAngle[4];            //Pitch angle, range 0~180000, Accuracy 0.01
    SANY_U8  ucRollAngle[4];             //Roll angle, range 0~36000, Accuracy 0.001
    SANY_U8  ucLongitude[4];             //Accuary 1 / 10000000
    SANY_U8  ucLatitude[4];              //Accuary 1 / 10000000
    SANY_U8  next[0];
} WP_RT_Locationnew_st;

/******************协议8.4.2.3 传感器感知数据头结构体******************/
typedef struct {
    SANY_U8	ucFlg[1];			        //message cmd ID, 0x3
    SANY_U8  ucObstacleNum[1];           //Obstacle number, range: 0~250
    SANY_U8	next[0];
} WP_RT_Sensor_Header_st;

/*****************协议8.4.2.3 传感器感知数据类型结构体*****************/
typedef struct {
    SANY_U8   ucPerceptionType[1];            //Perception type, 1:Fusion data; 2:Front view camera; 3: Lidar; 4:Millimeter wave radar; 5:Ultrasonic radar
    SANY_U8   ucObstacleType[1];              //Obstacle Type, 1: Obstacle(No distinction); 2: Car; 3:Truck; 4:Specialcar; 5: Stone; 6: Pit; 7: Person; 0xFF: invaild
    SANY_U8   ucConfidence[1];                //Confidence, range 0~10
    SANY_U8   ucTime[4];                      //Get time, accuracy 10ms, Fomart:HH:MM:SS.SS
    SANY_U8   ucObXcoordinate[2];             //range(Based on the longest distance sensor) -250~250
    SANY_U8   ucObYcoordinate[2];             //range(Based on the longest distance sensor) -250~250
    SANY_U8   ucObHeading[2];                 //range 0~35999, accuracy 0.01
    SANY_U8   ucObLength[2];                  //range 0m~10m
    SANY_U8   ucObWidth[2];                   //range 0m~8m
    SANY_U8   ucObHeight[2];                  //range 0m~6m
    SANY_U8   ucObXSpeed[2];                  //Obstacle X direction speed, range 0KM/H~220KM/H
    SANY_U8   ucObYSpeed[2];                  //Obstacle Y direction speed, range 0KM/H~220KM/H
    SANY_U8   ucObXAcceleration[2];           //Obstacle X direction acceleration, range 0KM/H^2~5KM/H^5
    SANY_U8   ucObYAcceleration[2];           //Obstacle Y direction acceleration, range 0KM/H^2~5KM/H^5
    SANY_U8   next[0];
}WP_RT_Obstacle_st;


/**********************协议8.9 作业信息上报头结构体***********************/
typedef struct{
	SANY_U8 uctime[6];
	//SANY_U8 ucCount_infor[1];
	//SANY_U8 ucSybol_infor[1];
	SANY_U8 ucDataBodyNum[1];	 
	SANY_U8 ucData[0];
}OPERATION_INFOR_REPORT_ST;

/*********************协议8.9.2.1 作业数据上报结构体**********************/
typedef struct{
	SANY_U8 ucflg[1];     
	SANY_U8 ucTaskID[4];         //unique ID 
	SANY_U8 ucCount_com[1];      //completed cycle count
	SANY_U8 ucCount_Remi[1];     //remained discompleted cycle count
	SANY_U8 ucTask_statue[1];    // task status, 0x00 no task  0x01:load complete, 0x02:dump complete
	                        //0x03:task unfinished 0xFF: invalid value
	SANY_U8 ucReserved[10];      //resevered
	SANY_U8 next[0];
}OPERATION_DATA_REPORT_ST;


/***********************协议8.10 常态信息头结构体************************/
typedef struct {
    SANY_U8	ucTime[6];
    SANY_U8	ucDataBodyNum[1];
    SANY_U8	next[0];
} WP_Normal_Header_st;

/**********************协议8.10.2.1 整车低频数据结构体*********************/
typedef struct {
	SANY_U8	ucFlg[1];
	SANY_U8  ucSignalPower[1];   			//valid range:0-170 (equal:-120~50dBm),Accuracy:1 dBm, 0xFE abnarmal 0xFF invalid
	SANY_U8  ucBatteryPW[1];                 //valid range:0~200 (equal: 0~40V), Accuracy: 0.2V,   0xFE abnarmal 0xFF invalid
	SANY_U8  ucWaterTemp[1];					//valid range:0~200 (-50~150℃), Accurary :1℃,        0xFE abnarmal 0xFF invalid
	SANY_U8  ucEngineTime[4]; 				//valid range: 0~4220000000 (0~211000000h) Accurary :1 hour,0xFE abnarmal 0xFF invalid
	SANY_U8	ucTirePressue[10];              //BYTE[0]：tyre 1 Pressure BYTE[2]：tyre 2   Pressure ...BYTE[9]：tyre 10 Pressure the order of the
	                                    //tyre number is from left to right and from front to back, valid range: 0~90(210kPa~ 300kPa), 
	                                    //Accuracy 1kPa,  0xFE abnarmal 0xFF invalid
 	SANY_U8  ucTireTemp[10];                 //BYTE[0]：tyre 1 temperature BYTE[2]：tyre 2   temperature ...BYTE[9]：tyre 10 temperature,the order 
 	                                    //of the tyre number is from left to right and from front to back, valid range:0~100(0~100℃),
 	                                    //Accuracy 1℃,  0xFE abnarmal 0xFF invalid
	SANY_U8  ucReserved[10]; 
	SANY_U8  next[0];
} WP_Normal_LowFreq_VehicleData_st;


/***********************协议8.11 变量信息头结构体************************/
typedef struct {
    SANY_U8	ucTime[6];
    SANY_U8	ucDataBodyNum[1];
    SANY_U8	next[0];
} WP_Variable_Header_st;

/**********************协议8.11.2.1 整车变化数据结构体*********************/
typedef struct {
	SANY_U8	ucFlg[1];
	SANY_U8 	ucCarstate[1];         			//car status, 1:ready; 2:ACC ON; 3:Key Off; 4:Battery power off; FE:Abnormal; FF:invaild
	SANY_U8 	ucCtlstate[1]; 					//control status, 1:Manual; 2:Auto; 3.remote; 4.maintain, FE:Abnormal; FF:invaild
	SANY_U8 	ucWorkstate[1];					//Work status, 0x01:Ready, 0x02:Delay, 0x03: Faulty 0x04: repair, FE:Abnormal; FF:invaild
	SANY_U8  ucReserved[10]; 
	SANY_U8 	next[0];
} WP_Variable_VehicleData_st; 

/**********************协议8.11.2.1 整车变化数据结构体*********************/
typedef struct {
	SANY_U8 	ucFlg[1];
	SANY_U8 	ucChargeplugstate[1];         	//Charge plug state:0x01 disconnect, 0x02: connect, FE:Abnormal; FF:invaild  
	SANY_U8 	ucChargestate[1];               //charge state:1:discharged, 2:charge finish, 3:Charge ready, 4:charging, 5:charge faulty FE:Abnormal; FF:invaild
	SANY_U8 	ucReserved[5];
	SANY_U8 	next[0];
} WP_charge_Data_st;


/**********************协议8.12.1 矿车协同请求结构体**********************/
typedef struct{
    SANY_U8 uctime[6];
    SANY_U8 ucInstr_Code[1];
    SANY_U8 ucInstr_Flow[1];
}WP_Equ_To_st;

/*********************协议8.12.1 矿车协同请求回复结构体********************/
typedef struct{
	SANY_U8 uctime[6];
	SANY_U8 Feedback_instr_Pip[1];
	SANY_U8 Feedback_instr_Code[1];
	SANY_U8 Feedback_Result[1];
}FEEDBACK_MINE_COLL_ST;

/**********************协议8.12.2 泊车位置请求结构体**********************/
typedef struct{
    SANY_U8 uctime[6];
    SANY_U8 ucInstr_Code[1];
    SANY_U8 ucInstr_Flow[1];
}WP_Exca_Po_st;

/********************协议8.12.2 泊车位置请求回复结构体********************/
typedef struct{
	SANY_U8 uctime[6];
    SANY_U8 Feedback_instr_Pip[1];
	SANY_U8 Feedback_instr_Code[1];
    SANY_U8 Pos_Type[1];
	SANY_U8 longitude[4];
	SANY_U8 latitude[4];
	SANY_U8 Yaw_angle[4];
}FEEDBACK_LOCA_EXCA_ST;


/************************协议8.13 设备监控结构体*************************/
typedef struct{
    SANY_U8  ucTime[6];
    SANY_U8  ucBatteryVol[2];
    SANY_U8  ucReserved[10];
}WP_Device_Monitor_st;


/************************协议8.15 任务下发头结构体***********************/
typedef struct{
    SANY_U8 uctime[6];
    SANY_U8 ucInstr_Code[1];
    SANY_U8 ucInstr_Flow[1];
	SANY_U8 ucInstr_len[2];
	SANY_U8 ucData[0];
}WP_task_send_st;

/**********************协议8.15 任务下发回复结构体***********************/
typedef struct{
	SANY_U8 uctime[6];
	SANY_U8 Feedback_instr_Pip[1];
	SANY_U8 Feedback_instr_Code[1];
    SANY_U8 Feedback_result[1];          //  update  20210204  V2.2
	SANY_U8 Task_status[1];
}FEEDBACK_TASK_SEND_ST;


/***********************协议8.15.1 紧急停车结构体***********************/
typedef struct
{
	SANY_U8 Emerg_stop[1];
}WP_TAKS_emerg_stop_st;

/*********************协议8.15.2 车辆间距控制结构体*********************/
typedef struct
{
	SANY_U8 Dis_Vehi_control[2];
}WP_TASK_distance_vehi_ctl;

/***********************协议8.15.3 车速控制结构体***********************/
typedef struct
{
	SANY_U8 Target_Speed[1];
} WP_task_target_speed;

/***********************协议8.15.4 任务更新结构体***********************/
typedef struct{
    SANY_U8 Clotask_ID[4];
    SANY_U8 Clotask_Sche[1];
    SANY_U8 Clotask_Count[1];
    SANY_U8 Reverse[5];       //invalid segment
	SANY_U8 ucData[0];
}WP_task_cloud_updata_st;


/***********************协议8.15.5 运维功能结构体***********************/
typedef struct
{
	SANY_U8 opera_func[1];
} WP_task_opera_func;

/*********************协议8.15.6 标定地点/路线结构体********************/
typedef struct
{	
	SANY_U8 Calib_location[1];
	SANY_U8 ucData[0];
} WP_task_cali_loc_rou;


/************************协议8.16 远程车控结构体*************************/
typedef struct{
    SANY_U8      ucTime[6];
    SANY_U8      ucCmd[1];
    SANY_U8      ucSn[1];
    SANY_U8      ucReserved[2];
}WP_VehicleRemote_st;

/************************协议8.16 远程车控指令代码 枚举*************************/
typedef  enum{
         Remote_Vehicle_PowerON = 0x01,
         Remote_Vehicle_Autopilot_ON,   
         Remote_Vehicle_Autopilot_OFF,
         Remote_Vehilce_PowerOFF,
         Remote_Vehicle_Autopilot_Fast,
         Remote_Vehicle_PowerON_Fast,
         Remote_Vehilce_Invaild 
} WP_VehicleCMD;
         



/**********************协议8.16 远程车控回复结构体***********************/
typedef struct{
    SANY_U8      ucTime[6];
    SANY_U8      ucSn[1];
    SANY_U8      ucCmd[1];
    SANY_U8      ucRet[1];
    SANY_U8      ucReserved[2];
}WP_VeihcleRemote_Ack_st;


/**********************协议8.17 遥控驾驶请求结构体**********************/
typedef struct {
    SANY_U8		uctime[6];			   
    SANY_U8      ucSimStatus[1];
    SANY_U8      ucCMDSn[1];
    SANY_U8		ucCMDID[1];
    SANY_U8      ucWheelAngle[2];	    //steeling wheel angle, 0x0~0xE189
    SANY_U8      ucWheelSpeed[2];		//steeling wheel acceleration, 0~500
    SANY_U8      ucSpeedup[1];			//accelerator, 0~200
    SANY_U8      ucSpeedDown[2];			//brake, 0~400
    SANY_U8      ucGear[1];				//gear
    SANY_U8      ucHandbrake[1];			//handbrake
    SANY_U8      ucPowerStatus[1];       //power to lift or car
    SANY_U8      ucLiftState[1];			//pull up/down
    SANY_U8      ucCarStatus[1];         //require to control car
} WP_REMOTE_CTL_st;

/********************协议8.17 遥控驾驶请求回复结构体********************/
typedef struct {

    SANY_U8		uctime[6];			    
    SANY_U8		ucCMDID[1];
    SANY_U8		ucRes[1];
    SANY_U8		ucVehicleSpeed[2];
    SANY_U8		ucLocation[20];
    SANY_U8		ucHeadAngle[2];
    SANY_U8		ucLiftState[1];
    SANY_U8		ucHandBrake[1];
    SANY_U8		ucGear[1];
} WP_REMOTE_ACK_st;

/**********************协议8.17.1 请求遥控驾驶结构体**********************/
typedef struct {
    SANY_U8		uctime[6];			   
    SANY_U8      ucVehicleName[10];
	SANY_U8		ucVehicleVIN[17];
	SANY_U8      ucVehicleICCID[10];
	SANY_U8		ucReqUrgency[6];	/*请求紧急度*/
} WP_REQ_REMOTE_CTL_st;

/********************协议8.17 遥控驾驶驾驶舱信息结构体********************/
typedef struct {
    SANY_U8		uctime[6];			    
    SANY_U8		ucCockpitIp[4];
	SANY_U8		ucCockpitPort[2];
} WP_REMOTE_COCKPIT_INFO_st;


/*****************协议8.19 自定义下行数据透传头结构体*****************/
typedef struct{
    SANY_U8 uctime[6];
    SANY_U8 ucMsgFlg[1];    //0x01 传输开始 0x02数据下发        0x03传输结束        0x04  传输中断
    SANY_U8 ucData[0];
}WP_Customize_Downlink_Header_st;

/**********************协议8.19.2 传输开始结构体*********************/
typedef struct{
    SANY_U8 ucFileType[1];
    SANY_U8 ucWindowSize[2];                 //一个窗口内发送分片的数量
    SANY_U8 ucDataLen[4];                    //发送完整文件的数据总长度
    SANY_U8 ucSegmentNum[2];                 //发送完整文件的总分片数量
    SANY_U8 ucSegmentLen[2];                 //分片的数据长度 自定义长度
}WP_Downlink_TaskStart_st;

/********************协议8.19.2 传输开始回复结构体********************/
typedef struct{
    SANY_U8 uctime[6];
    SANY_U8 ucRecFlag[1];
}WP_Downlink_StartAck_st;

/**********************协议8.19.3 数据下发结构体*********************/
typedef struct{
    SANY_U8 ucWindowsNo[2];          //协议2.2增加
    SANY_U8 ucSegmentNo[2];
    SANY_U8 ucData[0];
}WP_Downlink_Download_st;

/********************协议8.19.3 数据下发回复结构体*******************/
typedef struct{
    SANY_U8 uctime[6];
    SANY_U8 ucWindowsNo[2];              //接收的窗口序号
    SANY_U8 ucRecFlag[1];               // 数据接收状态       0x01：数据接收异常,0x02：数据接收正常,0xFF：无效数据
    SANY_U8 ucSegmentNum[2];             //分片接收异常的数量
    SANY_U8 ucData[0];
  //  SANY_U8 ucAbnormalNo[2];             //接收异常分片序号
}WP_Downlink_DownloadAck_st;

/**********************协议8.19.4 传输结束结构体*********************/
typedef struct{
    SANY_U8 ucTaskEnd[1];
}WP_Downlink_End_st;

/********************协议8.19.4 传输结束回复结构体*******************/
typedef struct{
    SANY_U8 uctime[6];
    SANY_U8 ucRecFlag[1];
}WP_Downlink_EndAck_st;

/**********************协议8.19.5 传输中断结构体*********************/
typedef struct{
    SANY_U8 u8InterruptFlg[1];
}WP_Downlink_Interrupt_st;
/**********************协议8.19.5 传输中断回复结构体*****************/
typedef struct{
    SANY_U8 uctime[6];
}WP_Downlink_InterruptACK_st;





#pragma pack()

/*********************************************************************
 * FUNCTIONS
 */

#endif // PROTODEF_H
