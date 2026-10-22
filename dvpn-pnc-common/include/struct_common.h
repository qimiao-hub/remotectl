#ifndef STRUCT_COMMON_H
#define STRUCT_COMMON_H
//#include "state_define.h"

#include "zkbs/ddsBase/ddsSubBase.h"
#include "zkbs/ddsBase/ddsPubBase.h"

#include <vector>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <string>
#include "type.h"
#include "sany_utils.h"

#include "stdio.h"

//#include "syzk_msgs/ctl_msgPubSubTypes.h"
//#include "syzk_msgs/decide_msgPubSubTypes.h"
//#include "syzk_msgs/gps_imu_msgPubSubTypes.h"
//#include "syzk_msgs/gps_msgPubSubTypes.h"
//#include "syzk_msgs/imple_msgPubSubTypes.h"
//#include "syzk_msgs/obst_msgPubSubTypes.h"
//#include "syzk_msgs/plan_msgPubSubTypes.h"
//#include "syzk_msgs/plat_msgPubSubTypes.h"
//#include "syzk_msgs/wire_msgPubSubTypes.h"

#define R_GEAR -1
#define N_GEAR 0
#define D_GEAR 1
//#define P_GEAR 2

#define P_GEAR_PROTO        7
#define R_GEAR_PROTO        -1
#define N_GEAR_PROTO        0
#define D1_GEAR_PROTO       1
#define D2_GEAR_PROTO       2
#define D3_GEAR_PROTO       3
#define D4_GEAR_PROTO       4
#define D5_GEAR_PROTO       5
#define D6_GEAR_PROTO       6

#define P_GEAR_PROTO_ELEC 1
#define R_GEAR_PROTO_ELEC 2
#define N_GEAR_PROTO_ELEC 3
#define D_GEAR_PROTO_ELEC 4

#define R_GEAR_SKT90E 13

#define D_GEAR_PROTO        D1_GEAR_PROTO



#define ZAN_TEST 0
#define TRANS_INFO_TO_CLOUD 0

struct S_KalmanInfo
{
    SANY_F32 filterValue;//滤波后的值,这个初值不重要,因为会不停更新
    SANY_F32 kalmanGain;//Kalamn增益
    SANY_F32 A;//状态矩阵,在一阶滤波器里面就是一个常数
    SANY_F32 H;//观测矩阵,在一阶滤波器里面就是一个常数
    SANY_F32 Q;//状态矩阵的方差,表示预测模型的噪声,是对预测模型的信任程度;
    SANY_F32 R;//观测矩阵的方差,表示测量模型的噪声,是对测量系统的信任程度;
    SANY_F32 P;//状态协方差,这个初值不重要,因为会不停更新
};

struct S_ActiveDiagnosticTroubleCodes
{
    SANY_U32 protect_lamp_status; // 00:OFF 01:ON
    SANY_U32 amber_warning_lamp_status; // 00:OFF 01:ON
    SANY_U32 red_stop_lamp_status; // 00:OFF 01:ON
    SANY_U32 malfunction_indicator_lamp_status; // 00:OFF 01:ON
    SANY_U32 flash_protect_lamp; //
                            // 00 Slow Flash -1 Hz
                            // 01 Fast Flash-2 Hz or faster
                            // 11 Uinavailable / Do Not Flash
    SANY_U32 flash_amber_warn_lamp; //
                            // 00 Slow Flash -1 Hz
                            // 01 Fast Flash-2 Hz or faster
                            // 11 Uinavailable / Do Not Flash
    SANY_U32 flash_red_stop_lamp;
                            // 00 Slow Flash -1 Hz
                            // 01 Fast Flash-2 Hz or faster
                            // 11 Uinavailable / Do Not Flash
    SANY_U32 flash_malfunction_indicator_lamp;
                            // 00 Slow Flash -1 Hz
                            // 01 Fast Flash-2 Hz or faster
                            // 11 Uinavailable / Do Not Flash
    SANY_U32 SPN_LSB; // suspect parameter number
    SANY_U32 SPN_MSB;
    SANY_U32 FMI; // failure mode identifier
            //     0 Data Valid but above Normal Operating Range
            //     1 Data Valid but below Normal Operating Range
            //     2 Data Erratic, SANY_S32ermittent or Incorrect
            //     3 Voltage above Normal or Shorted to High Source
            //     4 Voltage below Normal or Shorted to Low Source
            //     5 Current below Normal or Open Circuit
            //     6 Current above Normal or Grounded Circuit
            //     7 Mechanical System Not Responding or out of adjustment
            //     8 Abnormal frequency or pulse width or period
            //     9 Abnormal Update Rate
            // 10 Abnormal Rate of Change
            // 11 Root Cause Not Know
            // 12 Bad SANY_S32elligent Device or Component
            // 13 Out of Calibration
            // 14  Special Instructions
            // 15 Data Valid But Above Normal Operating Range (Least Severe Level)
            // 16 Data Valid But Above Normal Operating Range (Moderate Sever Level)
            // 17 Data Valid But Below Normal Operating Range (Least Severe Level)
            // 18 Data Valid But Below Normal Operating Range (Moderate Severe Level)
    SANY_U32 occurence_count;
    SANY_BOOL conversion_method; // 0:低字节在前 1：低字节在后
};

struct S_TransportProtocolConnectionManage
{
    SANY_U32 control_byte; // 控制字节
    SANY_U32 total_message_size; // 总信息字节数
    SANY_U32 total_number_of_packets; // 总包数
    SANY_U32 PGN;
};

struct S_TransportProtocolDataTransfer
{
    SANY_U32 sequence_number;
    SANY_U32 packetized_data;
};

struct S_ImpleResultInfo
{
    // Torque/Speed Control 1
    SANY_U32 engine_override_ctrl_mode; //　控制模式，00:Override disabled
                                                        // 01:Speed control
                                                        // 10:Torque control
                                                        // 11:Speed/torque limit control
    SANY_U32 engine_req_speed_ctrl_condition; // 请求速度控制状态
                                                    // 00 Transient Optimized for driveline disengaged and non-lockup conditions
                                                    // 01 Stability Optimized for driveline disengaged and non-lockup conditions
                                                    // 10 Stability Optimized for driveline engaged and/or in lockup condition 1 (e.g., vehicle driveline)
                                                    // 11 Stability Optimized for driveline engaged and/or in lockup condition 2 (e.g., PTO driveline)
    SANY_U32 override_ctrl_mode_priority; // 控制模式优先级
                                                // 00 Highest priority
                                                // 01 High priority
                                                // 10 Medium priority
                                                // 11 Low priority
    SANY_F32 engine_req_speed; // 请求速度，单位：rpm
    SANY_F32 engine_req_torque; //　请求转矩,单位：%

    // Shift  Control
    SANY_U32 trans_req_gear; //档位
                                // 0xE0  无操作
                                // 0x7D  N
                                // 0xDF  R
                                // 0xFC  D
                                // 0xF9  UP
                                // 0xF7  DOWN
    SANY_U32 trans_gearbox_highest_gear_setting;

    // Electronic Retarder Controller 1
    SANY_F32 driver_demand_retarder_CCU;

    // Unmanned Request
    SANY_BOOL unmanned_enable; //　无人驾驶就绪状态，0:未就绪　1:就绪
    SANY_BOOL on_start_enable; // ON档电启动使能，０:不使能　1: 使能
    SANY_BOOL vehicle_startup_ctrl; //　车辆启动控制，0:断开 1:闭合
    SANY_BOOL flash_ctrl; // 爆闪灯控制，0: 断开 1: 闭合
    SANY_U32 container_ctrl; // 货箱控制
                                    // 0: 无动作
                                    // 1: 举升
                                    // 2: 下降
                                    // 3：浮动降落
    SANY_U8 Remote_enable;//遥控驾驶使能位 非5:启动  5：未启动
    SANY_BOOL left_turn_signal_ctrl; // 左转向灯控制，0:断开 1:闭合
    SANY_BOOL right_turn_signal_ctrl; // 右转向灯控制，0:断开 1:闭合
    SANY_BOOL fog_lamp_ctrl; // 雾灯控制，0:断开 1:闭合
    SANY_BOOL outline_lamp_ctrl; // 示廓灯控制，0：断开 1：闭合
    SANY_BOOL low_light_ctrl; // 近光灯控制，0：断开 1：闭合
    SANY_BOOL high_beam_ctrl; // 远光灯控制，0：断开 1：闭合
    SANY_BOOL wiper_ctrl; // 雨刮喷水控制，0：断开 1：闭合
    SANY_BOOL wiper_intermittent_ctrl; // 雨刮间歇控制，0：断开 1：闭合
    SANY_BOOL wiper_low_speed_ctrl; // 雨刮低速控制，0：断开 1：闭合
    SANY_BOOL wiper_high_speed_ctrl; // 雨刮高速控制，0：断开 1：闭合
    SANY_BOOL hazard_warn_lamp_ctrl; // 危险警报灯控制，0：断开 1：闭合
    SANY_BOOL rear_light_ctrl; // 后照灯控制，0：断开 1：闭合
    SANY_BOOL horn_start; // 喇叭控制，0：断开 1：闭合
    SANY_BOOL parking_req; // 驻车请求，0：驻车 1：解除
    SANY_BOOL brake_req_status; // 刹车请求状态，0：未请求 1：已请求
    SANY_BOOL hydraulic_retarder_ctrl; // 液压缓行控制，0：断开 1：闭合
    SANY_BOOL axial_ctrl; // 轴差控制，0：断开 1：闭合
    SANY_F32 traffic_brake_req; // 行车制动请求
    SANY_BOOL exhaust_brake_req; // 排气制动控制，0：断开 1：闭合
    SANY_BOOL brake_light_ctrl; // 制动灯控制，0：断开 1：闭合
    SANY_BOOL slow_light_ctrl; // 液压缓行灯控制，0：断开 1：闭合
    SANY_BOOL head_light_ctrl; // 前照灯控制，0：断开 1：闭合

    // Steering angle request
    SANY_F32 steering_angle_req; // Factor:0.1; Offset: -50

    SANY_F32 active_spd;   //车速
    SANY_F32 yaw_rate;     //横摆角速度
    SANY_S32 drive_direction; // 车辆行驶方向
    SANY_F32 desire_speed;
    SANY_BOOL write_speaker_control_ctr;
    SANY_S32 set_max_gear;
    SANY_U8 vilid_count;
    SANY_BOOL rolling_flag;

    //130下发新增
    SANY_U8 Electric_braking;//电制动
    SANY_BOOL Parking_airSup_state;//停车补气状态
    SANY_BOOL Engine_cylinder_braking;//发动机缸内制动
    SANY_BOOL clutchControlCmd; //离合控制
    SANY_U8 Bucket_pitch; // 下发给底盘的货箱俯仰角
};

struct S_WirectlFeedbackInfo
{
    //Engine information
    SANY_U32 engine_torque_mode; // 发动机扭矩模式
    //电车
    SANY_S16 main_DriveMotor_Tem_raw;//主电机温度 offset -40
    SANY_S16 main_DriveMotor_speed_raw;//主电机转速 offset -6000
    SANY_F32 main_DriveMotor_Current_raw;//主电机电流 offset -3000
    SANY_S16 mainDriveMotorControllerTemp_raw;//主控制器温度 offset -40
    SANY_F32 Powertrain_outputTorque;//动力总称输出扭矩 offset -3000
    //油车
    SANY_F32 actual_engine_fractional_torque; // 发动机附加扭矩，单位：%
    SANY_S16 driver_demand_engine_percent_torque; // 驾驶指令百分比扭矩，单位：%
    SANY_S16 actual_engine_torque; // 发动机实际扭矩百分比，单位：%
    SANY_U32 engine_speed; // 发动机转速，单位：rpm
    SANY_U32 src_address_ctrl_dev_for_engine; // 控制发动机的设备地址
    //
    SANY_S16 accel_pedal_position; // 油门踏板位置，单位：%
    SANY_BOOL accelerator_requestStatus;//油门请求状态
    SANY_S16 engine_demand_percent_torque; // 发动机需求扭矩百分比,单位：%

    //Brake infomation
    SANY_S16 brake_pedal_SYMC; // SYMC制动深度，单位：%
    SANY_S16 actual_retarder_percent_torque; // 缓速器档位，单位：%
    SANY_F32 foot_valve_info; // 脚阀传感器信号，单位：Mpa
    SANY_BOOL parking_brake_signal_status; // 驻车制动信号状态，0：驻车 1：不驻车(车辆直接反馈)
    SANY_BOOL exhaust_brake_status; // 排气制动状态，0：断开 1：闭合
    SANY_BOOL hydraulic_startup_status; // 液压缓行启动状态，0：OFF 1:ON
    // SANY_BOOL ABSflg ;//ABS触发状态
    // SANY_BOOL ASRflg ;//ASR触发状态
    SANY_U8 Engine_wait_to_lamp;//发动机进气预热指示

    //Steer infomation
    SANY_F32 steering_angle_request; // 单位：度
    SANY_F32 steering_left_angle_info;
    SANY_F32 steering_right_angle_info;


    //Transmission information
    SANY_S32 trans_current_gear; // 变速箱当前档位信息
                                // N：0
                                // R：-1
                                // D1：1
                                // D2：2
                                // D3：3
                                // D4：4
                                // D5：5
                                // D6：6
    // ECM Hours  &&  Battery information1
    SANY_F32 Accumulated_charged_electric_quanlity;//累计充电电量
    SANY_F32 Accumulated_discharged_electric_quanlity;//累计放电电量
    SANY_S16 Single_charged_electric_quanlity;//单次充电电量
    SANY_F32 Battery_power_raw; // 电池功率
    SANY_F32 engine_total_hours; // 发动机工作总时长
    SANY_F32 on_voltage; // On档电压
    SANY_F32 battery_voltage; //蓄电池电压
    SANY_U32 Acc_Power_On_Status;//Acc电源状态
    SANY_U32 Charge_Gun_Status;//充电枪状态
    SANY_U32 Force_State;//取力状态
    SANY_U32 Vehicle_Remote_Start_Error;//车辆远程启动错误
    SANY_BOOL vehicle_loss_ctrl;//车辆失控


    //Body electrical information1
    SANY_U32 transmission_requested_range;//变速器请求换档范围
    SANY_U32 shifting_gears;//是否正在换档
    SANY_U32 Locking_gearboxTorqueConverter;//缓速器锁止状态
    SANY_U32 lift_condition; // 货箱升降状态，
                            // 0x0无动作
                            // 0x1正在举升
                            // 0x2正在下降
    SANY_BOOL lift_longo_position_status; // 举升到位接近开光状态，0：OFF 1：ON
    SANY_BOOL drop_longo_position_status; // 下降到位接近开光状态，0：OFF 1：ON
    SANY_BOOL left_turn_start_status; // 左转向灯启动状态，0：OFF 1：ON
    SANY_BOOL right_turn_start_status; // 右转向灯启动状态，0：OFF 1：ON
    SANY_BOOL hazard_start_status; // 危险警报灯启动状态，0：OFF 1：ON
    SANY_BOOL flash_start_status; // 爆闪灯启动状态，0：OFF 1：ON
    SANY_BOOL fog_lamp_start_status; // 雾灯启动状态，0：OFF 1：ON
    SANY_BOOL outline_lamp_start_status; // 示廓灯启动状态，0：OFF 1：ON
    SANY_BOOL low_light_start_status; // 近光灯启动状态，0：OFF 1：ON
    SANY_BOOL high_beam_start_status; // 远光灯启动状态，0：OFF 1：ON
    SANY_BOOL head_light_status; // 前照灯状态，0：OFF 1：ON
    SANY_BOOL rear_light_start_status; // 后照灯启动状态，0：OFF 1：ON
    SANY_BOOL wiper_start_status; // 雨刮喷水启动状态，0：OFF 1：ON
    SANY_BOOL wiper_longermit_start_status; // 雨刮间歇启动状态，0：OFF 1：ON
    SANY_BOOL wiper_slow_start_status; // 雨刮低速启动状态，0：OFF 1：ON
    SANY_BOOL wiper_high_start_status; // 雨刮高速启动状态，0：OFF 1：ON
    SANY_BOOL horn_start_status; // 喇叭启动状态，0：OFF 1：ON
    SANY_BOOL inter_axle_differential; // 轴差，0：OFF 1：ON
    SANY_BOOL go_to_unmanned_status; // 进入无人驾驶状态，0：不允许 1：允许
    SANY_BOOL slow_light_start_status; // 缓行灯启动状态，0：OFF 1：ON
    SANY_BOOL brake_light_start_status; // 制动灯启动状态，0：OFF 1：ON
    SANY_BOOL revers_light_start_status; // 倒车灯启动状态，0：OFF 1：ON
    SANY_BOOL charge_indicator; // 充电指示，0：OFF 1：ON
    SANY_U32 vehicle_status; // 车辆状态 0： On档 1：启动 2：启动失败

    //tire pressure
    SANY_U32 Front_L_tire_pressure;//前左轮胎压
    SANY_U32 Front_R_tire_pressure;//前右轮胎压
    SANY_U32 MID_LL_tire_pressure;//中左左轮胎压
    SANY_U32 MID_LR_tire_pressure;//中左右轮胎压
    SANY_U32 MID_RL_tire_pressure;//中右左轮胎压
    SANY_U32 MID_RR_tire_pressure;//中右右轮胎压
    SANY_U32 BACK_LL_tire_pressure;//后左左轮胎压
    SANY_U32 BACK_LR_tire_pressure;//后左右轮胎压
    SANY_U32 BACK_RL_tire_pressure;//后右左轮胎压
    SANY_U32 BACK_RR_tire_pressure;//后右右轮胎压

    //轮边压力
    SANY_U32 Brake_Pressure_FA_Left;//制动压力前桥左
    SANY_U32 Brake_Pressure_FA_Right;//制动压力前桥右
    SANY_U32 Brake_Pressure_RA1_Left;//制动压力后桥左
    SANY_U32 Brake_Pressure_RA1_Right;//制动压力前桥右

    //tire temperature
    SANY_S16 Front_L_tire_temperature;//前左轮胎温
    SANY_S16 Front_R_tire_temperature;//前右轮胎温
    SANY_S16 MID_LL_tire_temperature;//中左左轮胎温
    SANY_S16 MID_LR_tire_temperature;//中左右轮胎温
    SANY_S16 MID_RL_tire_temperature;//中右左轮胎温
    SANY_S16 MID_RR_tire_temperature;//中右右轮胎温
    SANY_S16 BACK_LL_tire_temperature;//后左左轮胎温
    SANY_S16 BACK_LR_tire_temperature;//后左右轮胎温
    SANY_S16 BACK_RL_tire_temperature;//后右左轮胎温
    SANY_S16 BACK_RR_tire_temperature;//后右右轮胎温


    //tire error code
    SANY_U8 Front_L_tire_code;//前左轮胎压故障码
    SANY_U8 Front_R_tire_code;//前右轮胎压故障码
    SANY_U8 MID_LL_tire_code;//中左左轮胎压故障码
    SANY_U8 MID_LR_tire_code;//中左右轮胎压故障码
    SANY_U8 MID_RL_tire_code;//中右左轮胎压故障码
    SANY_U8 MID_RR_tire_code;//中右右轮胎压故障码
    SANY_U8 BACK_LL_tire_code;//后左左轮胎压故障码
    SANY_U8 BACK_LR_tire_code;//后左右轮胎压故障码
    SANY_U8 BACK_RL_tire_code;//后右左轮胎压故障码
    SANY_U8 BACK_RR_tire_code;//后右右轮胎压故障码

    //air conditioner
    SANY_BOOL air_power_status;//空调开关状态
    SANY_BOOL air_auto_status;//空调自动状态
    SANY_BOOL air_ac_status;//空调AC制冷状态
    SANY_BOOL air_defrost_status;//空调除霜状态
    SANY_BOOL air_cyclemode_status;//循环模式状态
    SANY_U8 air_outletmode_status;//吹风模式状态
    SANY_U8 air_level_status;//空调风量状态
    SANY_U8 air_temperature_statusR;//设定温度
    SANY_S8 cab_temperature_status;//驾驶舱温度

    //130HDI新增
    SANY_S16 TM_Torque;//TM电机实际扭矩 -3200-3200
    SANY_S16 IM_Torque;//IM电机实际扭矩 -3200-3225
    SANY_S16 IM_Speed;//IM电机实际转速 -8000-8063
    SANY_U8 Parking_generation_status:2;//停车发电状态 0：不使能 1：使能 2：故障
    SANY_F32 Bucket_pitch;//货箱俯仰角度;-90~90 0.01
    SANY_S16 friction_torque;//摩擦扭矩 -125-125
    SANY_BOOL chassis_rolling;//底盘判断溜车反馈
    SANY_S16 L_brake_temperature;//左刹车温度
    SANY_S16 R_brake_temperature;//右刹车温度
    SANY_S16 Maximum_remaining_electric_braking;//最大剩余电制动
    SANY_BOOL is_arrving_wall;//是否抵墙
    SANY_S16 OutputTorqueGearbox;//变速箱输出扭矩
    SANY_S16 symcReqElecBrake;//SYMC请求的电制动0-100%
    SANY_S16 symcElecTorqueResp;;//底盘反馈的SYMC请求的电制动0-100%
    SANY_S16 autoElecTorqueResp;;//底盘反馈的无人驾驶请求的电制动0-100%
    SANY_U8 gear_state;//档位状态0--N档 1--D档 2--R档 3--M档4--S档
    SANY_U8 clutchState; //离合状态 1结合 0分离 2半联动




    //EBS light
    SANY_U8 EBS_red_alarmlight;
    SANY_U8 EBS_yellow_alarmlight;
    
    //brake hold state
    SANY_U8 Brake_Hold_state;

    //VCU版本号
    std::string VCU_Version;
    //电车
    SANY_S16 ramp_sensor;//坡度传感器
    // Aux motor information
    SANY_S16 rxauxiliary_DriveMotor_Tem_raw;//辅电机温度 offset -40
    SANY_F32 rxauxiliary_driveMotor_speed_raw;//辅电机转速 offset -6000
    SANY_F32 rxauxiliary_DriveMotor_Current_raw;//辅电机电流 offset -3000
    SANY_S16 rxauxiliarDriveMotorControllerTemp_raw;//辅电机控制器温度 offset -40

    //Battery information2
    SANY_U32 endurance_mileage;//剩余里程
    SANY_U32 power_consumption;//瞬时电耗
    SANY_U32 SOC;//荷电状态（剩余电量）
    SANY_U32 charging_state;//充电状态
    SANY_U32 Battery_State;//电池状态
    SANY_S16 Battery_Current;//电池电流 offset -1000
    SANY_S16 Battery_Temperature_raw;//电池温度 offset -40
    SANY_U32 Battery_Fault_Status;//电池故障状态
    SANY_U32 energy_recover;//能量回收状态
    // Engine Temperature
    SANY_F32 engine_coolant_temperature; // 发动机冷却水温度，单位：摄氏度
    SANY_F32 fuel_temperature; // 燃油温度，单位：摄氏度
    SANY_F32 engine_oil_temperature; // 机油温度，单位：摄氏度

    // Engine Fluid Level/Pressure
    SANY_F32 engine_oil_level; // 机油液位，单位：%
    SANY_F32 engine_oil_pressure; // 机油压力，单位：kpa

    // Fuel Economy (Liquid)
    SANY_F32 engine_fuel_rate; // 油耗，单位：L/h
    SANY_F32 engine_instant_fuel_economy; // 瞬时油耗，单位：km/L
    SANY_F32 engine_average_fuel_economy; // 平均燃油经济性，单位：km/L
    SANY_S16 fuel_level; // 油量，单位：%

    //Body electrical information2
    SANY_F32 air_pressure_1; // 单位：MPa
    SANY_F32 air_pressure_2; // 单位：MPa
    SANY_F32 front_axle_running_brake_pressure_input;//前桥行车制动压力输入0~2.55
    SANY_F32 brake_pressure_input_in_middle_axle;//中桥行车制动压力输入0~2.55
    SANY_F32 rear_axle_running_brake_pressure_input;//后桥行车制动压力输入0~2.55
    SANY_U32 weighing_cell_1;//称重传感器10~255
    SANY_U32 weighing_cell_2;//称重传感器20~255
    SANY_U32 num_297;//297与控制的心跳
    // Wheel speed information
    SANY_F32 front_left_wheel_speed; // 前左轮车速，单位：km/h
    SANY_F32 front_right_wheel_speed; // 前右轮车速，单位：km/h
    SANY_F32 rear_left_wheel_speed; // 后左轮车速，单位：km/h
    SANY_F32 rear_right_wheel_speed; // 后右轮车速，单位：km/h

    
    // Tachograph
    SANY_F32 tachograph_output_shaft_speed; // 变速箱输出轴转速，单位： rpm
    SANY_F32 tachograph_vehicle_speed; // 车速，单位：km/h
    SANY_F32 total_vehicle_distance; // 单位：m


    //fuel consumption
    SANY_F32 engine_totalFuel_used;//发动机总油耗 单位：L
    SANY_F32 trip_fuel_consumption;//发动机旅程油耗 单位：L

    // Wheel angle sensor information
    SANY_F32 steering_left_angle_info_mv; // 单位：mv
    SANY_F32 steering_right_angle_info_mv; // 单位：mv

    SANY_U32 vcu_state;
    SANY_U32 udpindex;

    // DM1_ECM -- Active Diagnostic Trouble Codes
    S_ActiveDiagnosticTroubleCodes dm1_vehicle_diag_trouble;

    // ECM -- Transport Protocol-Connection Management
    S_TransportProtocolConnectionManage ecm_tpcm;

    // ECM -- Transport Protocol-Data Transfer
    S_TransportProtocolDataTransfer ecm_tpdt;

    // DM1_TCU -- Active Diagnostic Trouble Codes
    S_ActiveDiagnosticTroubleCodes dm1_tcu_diag_trouble;

    // TCU -- Transport Protocol-Connection Management
    S_TransportProtocolConnectionManage tcu_tpcm;

    // TCU -- Transport Protocol-Data Transfer
    S_TransportProtocolDataTransfer tcu_tpdt;

    // SCA(Safety_Controller_Alarm) -- Active Diagnostic Trouble Codes
    S_ActiveDiagnosticTroubleCodes sca_diag_trouble;
    S_ActiveDiagnosticTroubleCodes sca_diag_trouble1;
    S_ActiveDiagnosticTroubleCodes sca_diag_trouble2;
    // MCA(Main_Controller_Alarm) -- Active Diagnostic Trouble Codes
    S_ActiveDiagnosticTroubleCodes mca_diag_trouble;
    S_ActiveDiagnosticTroubleCodes mca_diag_trouble1;
    S_ActiveDiagnosticTroubleCodes mca_diag_trouble2;

    // not clear
    SANY_U32 CAN_A_State;
    SANY_U32 CAN_B_State;
    SANY_BOOL parking; // 驻车，0：OFF 1：ON
    SANY_U32 low_air_pressure_alarm; // 气压过低警报，01：ON 10：OFF

    /*VCU反馈控制信息及电车故障码*/
    SANY_BOOL Autopilot_enable_bit;//自动驾驶使能位
    SANY_BOOL Power_control_enable_bit;//动力控制使能位
    SANY_BOOL Shift_request_status;//挂挡请求状态
    SANY_BOOL accelerator_request_status;//油门请求状态
    SANY_BOOL Parking_request;//驻车请求
    SANY_BOOL Brake_request_status;//刹车请求状态
    SANY_BOOL No15_power_supply_control;//15号电源控制
    SANY_BOOL Brake_light_control;//制动灯控制

    SANY_U8 Gear_request;//挡位请求
    SANY_BOOL ACC_power_control;//ACC电源控制
    SANY_U8 Container_control;//货箱控制
    SANY_BOOL Left_turn_signal;//左转向灯
    SANY_BOOL Right_turn_signal;//右转向灯

    SANY_U8 accelerator_request;//油门请求

    SANY_U8 Brake_request;//制动请求

    SANY_BOOL backup_light;//倒车灯控制
    SANY_BOOL Fog_lamp_control;//雾灯控制
    SANY_BOOL Small_lights;//小灯控制
    SANY_BOOL lower_beam_lamb;//近光灯控制
    SANY_BOOL higher_beam_lamb;//远光灯控制
    SANY_BOOL Wipers_spray_water;//雨刮喷水控制
    SANY_BOOL Wiper_intermission;//雨刮间歇控制
    SANY_BOOL Wiper_low_speed;//雨刮低速控制

    SANY_BOOL Wiper_high_speed;//雨刮高速控制
    SANY_BOOL Axle_lock_control;//轴锁控制
    SANY_BOOL Power_takeoff_start;//取力启动
    SANY_BOOL Flash_lamp;//爆闪灯控制
    SANY_BOOL Headlights;//前照灯控制
    SANY_BOOL Rear_light;//后照灯控制
    SANY_BOOL Horn_on;//喇叭启动
    SANY_BOOL S_gear_control;//S档控制

    SANY_BOOL Emergency_Stop_Switch_Status60;//急停开关状态
    SANY_BOOL Hazard_Warning_Lamp_Control54;//危险警报灯控制
    SANY_U8 Remote_or_Unman;// 反馈8为遥控   6为无人

    SANY_U16 Steer_1_Curvature_control;//VCU外发转向系统1控制曲率

    SANY_U16 Steer_2_Curvature_control;//VCU外发转向系统2控制曲率
};

struct S_GpsImuInfo
{
    SANY_S32 gps_week;            //自 1980-1-6 至当前的星期数（格林尼治时间）     wwww                1451
    SANY_F32 gps_time;         //自本周日 0:00:00 至当前的秒数（格林尼治时间）  ssssss.sss           368123.300
    SANY_F32 time_year;
    SANY_U32 time_month;
    SANY_U32 time_day;
    SANY_U32 time_hour;
    SANY_U32 time_minute;
    SANY_U32 time_second;
    struct tm time_record;
    SANY_S64 gps_msec;
    SANY_S64 gps_msec_total;
    SANY_S32 gps_rtk_status;    //差分状态
    SANY_DOUBLE gps_lat;        //纬度
    SANY_DOUBLE gps_lng;        //经度
    SANY_F32 gps_heading;     //航向角
    SANY_F32 gps_ve;          //东向速度
    SANY_F32 gps_vn;          //北向速度
    SANY_F32 gps_vd;          //天向速度
    SANY_F32 gps_days_to_speed;//天向速度
    SANY_F32 gps_speed;       //合速度 km/h
    SANY_F32 gps_height;      //海拔
    SANY_F32 imu_acc_y;       //纵向加速度
    SANY_F32 imu_acc_x;       //侧向加速度
    SANY_F32 imu_yaw_z;       //横摆角速度
    SANY_F32 imu_v_y;         //y轴角速度
    SANY_F32 imu_v_x;         //x轴角速度
    SANY_F32 imu_v_z;         //z轴角速度
    SANY_S32 gps_satellite_front;     //前天线卫星数
    SANY_S32 gps_satellite_rear;      //后天线卫星数

    SANY_F32 gps_yaw;      //偏航
    SANY_F32 gps_roll;      //横滚
    SANY_F32 gps_pitch;      //俯仰
};


struct S_FaultDiagnosisFrom297Info
{
    //VCU fault diagnosis
    SANY_BOOL tbox_on_line;//Tbox在线状态  1在线状态
    SANY_BOOL acu_on_line;//ACU在线状态
    SANY_BOOL symc_on_line;//SYMC在线状态
    SANY_BOOL ebs_on_line;//EBS在线状态
    SANY_BOOL ic_on_line;//IC在线状态
    SANY_BOOL ecm_on_line;//ECM在线状态
    SANY_BOOL tcu_on_line;//TCU在线状态
    SANY_BOOL steering_controller_1_on_line;//转向控制器1在线状态
    SANY_BOOL sas_sensor_on_line;//SAS传感器在线状态
    SANY_BOOL wheel_angle_sensor_on_line;//车轮角度传感器在线状态
    SANY_BOOL steering_controller_2_on_line;//转向控制器2在线状态
    SANY_BOOL wheel_angle_sensor_2_on_line;//车轮角度传感器2在线状态
    SANY_BOOL vcu_on_line;//VCU在线状态
    SANY_BOOL inertial_navigation_on_line;//惯导在线状态
    SANY_BOOL electromagnetic_switch_state;//电磁开关状态
    SANY_BOOL electric_start_enable_state;//On档电启动使能状态
    SANY_BOOL vehicle_start_control_state;//车辆启动控制状态
    SANY_BOOL enters_unmanned_driving_state;//进入无人驾驶状态
    SANY_BOOL vehicle_speed;//车速
    SANY_BOOL vehicle_start_state;//车辆启动状态
    SANY_BOOL vehicle_gear_state;//车辆档位状态
    SANY_BOOL parking; // 驻车状态，0：OFF 1：ON
    SANY_BOOL request_switch_reset_state;//请求开关复位状态
    SANY_BOOL car_reset_state;//车厢复位状态
    SANY_BOOL can_bus_communication_state;//CAN总线通信状态
    SANY_BOOL unmanned_driving_system_state;//无人驾驶系统状态
    SANY_BOOL steering_system_state;//转向系统状态
    SANY_BOOL symc_agrees_state;//SYMC同意状态
    SANY_BOOL tbox_request_state;//TBOX请求状态
    SANY_BOOL shift_switching_state;//档位切换状态
    SANY_BOOL cloud_exit;//(无人驾驶系统退出)
    SANY_BOOL traffic_brake_take_over;//行车制动接管
    SANY_BOOL unmanned_warping_switch_Reset;//无人驾驶翘板开关复位
    SANY_BOOL symc_offline;//SYMC掉线
    SANY_BOOL power_CAN_offline;//动力CAN掉线
    SANY_BOOL steering_system_Abnormal;//转向系统异常
    SANY_BOOL car_end_Level_3_fault;//车端3级故障
    SANY_BOOL unmanned_driving_system_Level_3_fault;//无人驾驶系统3级故障
    SANY_BOOL SYMC_abnormally_exits;//SYMC异常退出
    SANY_BOOL emergency_stop_switch;//急停开关
    SANY_BOOL vehicle_end_control_signal_does_not_respond_substandard;//车端控制信号不响应不达标
    SANY_BOOL vehicle_control_deviates_from_the_route;//车控偏离路线
    SANY_BOOL throttle_take_over_state;//油门接管状态

    //Vehicle fault diagnosis
    SANY_U32 vehicle_failure_grade;//车辆故障等级
    SANY_U32 vehicle_error_source;//车辆故障来源
    SANY_U32 gearbox_fault_level;//变速箱故障等级
    SANY_U32 gearbox_SPN_LSN;//变速箱SPN_LSN
    SANY_U32 gearbox_SPN_MSB;//变速箱SPN_MSB
    SANY_U32 gearbox_FMI;//变速箱FMI
    SANY_U32 steering_1_safety_controller_failure_level;//转向1安全控制器故障等级
    SANY_U32 turn_to_1_safety_controller;//转向1安全控制器SPN_LSN
    SANY_U32 turn_to_1_safety_controller_SPN_MSB;//转向1安全控制器SPN_MSB
    SANY_U32 steering_1_safety_controller_FMI;//转向1安全控制器FMI
    SANY_U32 steering_1_main_controller_failure_level;//转向1主控制器故障等级
    SANY_U32 turn_to_1_master_controller_SPN_LSN;//转向1主控制器SPN_LSN
    SANY_U32 go_to_1_master_controller_SPN_MSB;//转向1主控制器SPN_MSB
    SANY_U32 turn_to_1_main_controller_FMI;//转向1主控制器FMI
    SANY_U32 steering_2_safety_controller_failure_level;//转向2安全控制器故障等级
    SANY_U32 steering_2_safety_controller_SPN_LSN;//转向2安全控制器SPN_LSN
    SANY_U32 steering_2_Safety_Controller_SPN_MSB;//转向2安全控制器SPN_MSB
    SANY_U32 steering_2_Safety_Controller_FMI;//转向2安全控制器FMI
    SANY_U32 steering_2_main_controller_failure_level;//转向2主控制器故障等级
    SANY_U32 turn_to_2_master_controller_SPN_LSN;//转向2主控制器SPN_LSN
    SANY_U32 go_to_2_master_controller_SPN_MSB;//转向2主控制器SPN_MSB
    SANY_U32 turn_to_2_main_controller_FMI;//转向2主控制器FMI

    SANY_U32 ControlByte;//控制字节
    SANY_F32 TotalNumberByte;//总字节包数
    SANY_U32 TotalNumberPacket;//总包数
    SANY_F32 PGN;
    SANY_U32 SequenceNumber;//多包序号
    //多包信息
    SANY_U32 TP_DT_Byte1;//1-5bit:FMI  6-8bit:SPN_MSB
    SANY_F32 TP_DT_Byte2_3;//SPN_LSB
    SANY_U32 TP_DT_Byte4;//故障等级
    SANY_U32 TP_DT_Byte5;//0：无故障 1：VCU故障 2：动力系统故障 3：车身系统故障 4：电池系统故障 5:转向系统故障
    SANY_U32 TP_DT_Byte6;//FF
    SANY_U32 TP_DT_Byte7;//FF

    //电车
    //Vehicle fault diagnosis
    SANY_F32 Auxiliary_driveMotor_Error_list;//辅助电机故障列表
    SANY_F32 Main_driveMotor_Error_list;//主电机故障列表
    SANY_U32 Mauxiliary_DriveMotor_Fault;//辅助电机故障码
    SANY_U32 Main_DriveMotor_Fault_Code;//主电机故障码
    SANY_U32 TCU_SystemFault_Code_Table;//TCU系统故障码
    SANY_U32 TCU_OtherFault_Code_Table;//TCU其他故障码
    //7
    SANY_U32 occurrence_Count1Copy2;
    SANY_U32 Conversion_MethodCopy_2;
    SANY_U32 steering_1_safety_controller_failure_level_elec;//转向1安全控制器故障等级
    SANY_U32 turn_to_1_safety_controller_elec;//转向1安全控制器SPN_LSN
    SANY_U32 turn_to_1_safety_controller_SPN_MSB_elec;//转向1安全控制器SPN_MSB
    SANY_U32 steering_1_safety_controller_FMI_elec;//转向1安全控制器FMI
    SANY_U32 Protect_LampStatusc_Copy_2;
    SANY_U32 Amber_WarningLamp_Status_Copy_2;
    SANY_U32 Red_StopLamp_Status_Copy_2;
    SANY_U32 Malfunction_IndicatorLamp3;
    //7
    SANY_U32 occurrence_Count1Copy3;
    SANY_U32 Conversion_MethodCopy_3;
    SANY_U32 steering_1_main_controller_failure_level_elec;//转向1主控制器故障等级
    SANY_U32 turn_to_1_master_controller_SPN_LSN_elec;//转向1主控制器SPN_LSN
    SANY_U32 go_to_1_master_controller_SPN_MSB_elec;//转向1主控制器SPN_MSB
    SANY_U32 turn_to_1_main_controller_FMI_elec;//转向1主控制器FMI
    SANY_U32 Protect_LampStatusc_Copy_3;
    SANY_U32 Amber_WarningLamp_Status_Copy_3;
    SANY_U32 Red_StopLamp_Status_Copy_3;
    SANY_U32 Malfunction_IndicatorLamp4;
    //7
    SANY_U32 occurrence_Count1Copy4;
    SANY_U32 Conversion_MethodCopy_4;
    SANY_U32 steering_2_safety_controller_failure_level_elec;//转向2安全控制器故障等级
    SANY_U32 steering_2_safety_controller_SPN_LSN_elec;//转向2安全控制器SPN_LSN
    SANY_U32 steering_2_Safety_Controller_SPN_MSB_elec;//转向2安全控制器SPN_MSB
    SANY_U32 steering_2_Safety_Controller_FMI_elec;//转向2安全控制器FMI
    SANY_U32 Protect_LampStatusc_Copy_4;
    SANY_U32 Amber_WarningLamp_Status_Copy_4;
    SANY_U32 Red_StopLamp_Status_Copy_4;
    SANY_U32 Malfunction_IndicatorLamp5;
    //7
    SANY_U32 occurrence_Count1Copy5;
    SANY_U32 Conversion_MethodCopy_5;
    SANY_U32 steering_2_main_controller_failure_level_elec;//转向2主控制器故障等级
    SANY_U32 turn_to_2_master_controller_SPN_LSN_elec;//转向2主控制器SPN_LSN
    SANY_U32 go_to_2_master_controller_SPN_MSB_elec;//转向2主控制器SPN_MSB
    SANY_U32 turn_to_2_main_controller_FMI_elec;//转向2主控制器FMI
    SANY_U32 Protect_LampStatusc_Copy_5;
    SANY_U32 Amber_WarningLamp_Status_Copy_5;
    SANY_U32 Red_StopLamp_Status_Copy_5;
    SANY_U32 Malfunction_IndicatorLamp6;

    //GPS fault diagnosis
    SANY_U32 course_Angle_condition;//航向角状态(GPS航向初始化标志位)
    SANY_U32 vehicle_status_marker;//车辆状态标志位
    SANY_U32 GpsNumSatsUsed;
    SANY_U32 GpsNumSats2Used;
    SANY_U32 integrated_navigation_overall_status_flag_bits;//组合导航总体状态标志位

    //电车故障
    SANY_U8 DTC_Sequence_number;//电车故障包序号

    SANY_U8 FMI;//DTC FMI
    SANY_U8 SPN_MSB;//DTC SPN_MSB

    SANY_U16 SPN_LSB;//DTC SPN_LSB

    SANY_U8 Error_level;//故障等级
    SANY_BOOL Multi_Send_Enable;//故障码多包发送使能位
    SANY_U8 Fault_resource;//0：无故障 1：VCU故障 2：动力系统故障 3：车身系统故障 4：电池系统故障 5:转向系统故障

    SANY_U8 Multi_Send_Num;//多包发送包数
};


typedef struct
{
  uint  object_type;          //目标障碍物类型
  uint  detection_type;       //检测类型：0：激光雷达；1：毫米波雷达；2：图像；3：激光与图像融合；4：激光与毫米波融合；5：图像与毫米波融合；6：激光、毫米波、图像融合
  float object_dist_long;     //目标障碍物与本车的纵向相对距离，单位m
  float object_dist_lat;      //目标障碍物与本车的横向相对距离，单位m，左侧为负
  float object_vrel_long;     //前车与本车的纵向相对速度，单位m/s
  float object_vrel_lat;      //前车与本车的横向相对速度，单位m/s
  float object_width;         //目标障碍物宽度
  float object_height;        //目标障碍物高度

}SensorFuseResult;

union UPara
{
    f64 f64Para;
    f32 f32Para;
    s32 s32Para;
    u8  u8Para;
    s8  s8Para;
    u32 u32Para;
};

#pragma pack()

typedef union
{
    //int   gps_info;//是否GPS故障，1:无数据 2：丢差分 4：惯导故障 3:无数据+丢差分 5：无数据+惯导故障 6：丢差分+惯导故障 7：无数据+丢差分+惯导故障

    struct
    {
        //故障标志 0：无故障 1 2 3
        unsigned int gps_rtk_err_lev:2;
        unsigned int gps_signal_err_lev:2;
        unsigned int gps_unline_err_lev:2;

        unsigned int driver_err_lev:2;
        unsigned int engine_fail_lev:2;

        unsigned int break_err_lev:2;
        unsigned int break_pre_err_lev:2;
        unsigned int air_pre_err_lev:2;

        unsigned int retarder_err_lev:2;//16bit
        unsigned int preak_err_lev:2;

        unsigned int turn1_ctl_unline_lev:2;
        unsigned int turn2_ctl_unline_lev:2;
        unsigned int turn_self_err:2;
        unsigned int turn_res_err_lev:2;
        unsigned int turn_feat_err_lev:2;

        unsigned int lift_drop_err_lev:2;

        unsigned int shift_err_lev:2;//32bit
        unsigned int slide_err_lev:2;
        unsigned int track_offset_lev:2;

        //doc undefine
        unsigned int TCU_unline_lev:2;
        unsigned int ECM_unline_lev:2;
        unsigned int IC_unline_lev:2;
        unsigned int EBS_unline_lev:2;
        unsigned int SYMC_unline_lev:2;

        unsigned int ACU_unline_lev:2;//48bit
        unsigned int Tbox_unline_lev:2;
        unsigned int VCU_unline_lev:2;
        unsigned int turn1_angle_unline_lev:2;
        unsigned int turn2_angle_unline_lev:2;
        unsigned int SAS_unline_lev:2;

        //故障标志 1：故障
        unsigned int lidar_front_mid_lev:2;
        unsigned int lidar_front_mid_data_lev:2;
        unsigned int lidar_front_right_lev:2;//64bit
        unsigned int lidar_front_right_data_lev:2;
        unsigned int lidar_front_left_lev:2;
        unsigned int lidar_front_left_data_lev:2;
        unsigned int lidar_back_lev:2;
        unsigned int lidar_back_data_lev:2;

        unsigned int radar_front_mid_lev:2;
        unsigned int radar_front_mid_data_lev:2;
        unsigned int radar_front_right_lev:2;//80bit
        unsigned int radar_front_right_data_lev:2;
        unsigned int radar_front_left_lev:2;
        unsigned int radar_front_left_data_lev:2;
        unsigned int radar_back_lev:2;
        unsigned int radar_back_data_lev:2;

        unsigned int camera_online_lev:2;
        unsigned int camera_image_quality_lev:2;
        unsigned int camera_image_dim_lev:2;//96bit
        unsigned int camera_image_cover_lev:2;

        unsigned int final_lev:2;
    };
    unsigned int alldata[4];
}FaultDiagnosisResultInfo;

//typedef struct
//{
//    unsigned char low_air_pressure_alarm:1; //储气罐压力低故障 = 0

//    unsigned char unmanned_driving_system_Level_3_fault:1;//无人驾驶系统3级故障
//    unsigned char car_end_Level_3_fault:1;//车端3级故障
//    unsigned char steering_system_Abnormal:1;//转向系统异常
//    unsigned char SYMC_abnormally_exits:1;//SYMC异常退出

//    unsigned char engine_failure_grade:2;//发动机故障等级
//    unsigned char gearbox_fault_level:2;//变速箱故障等级
//    unsigned char steering_1_safety_controller_failure_level:2;//转向1安全控制器故障等级
//    unsigned char steering_1_main_controller_failure_level:2;//转向1主控制器故障等级
//    unsigned char steering_2_safety_controller_failure_level:2;//转向2安全控制器故障等级
//    unsigned char steering_2_main_controller_failure_level:2;//转向2主控制器故障等级

//    unsigned char tbox_on_line:1;//Tbox在线状态  1在线状态
//    unsigned char acu_on_line:1;//ACU在线状态
//    unsigned char symc_on_line:1;//SYMC在线状态
//    unsigned char ebs_on_line:1;//EBS在线状态
//    unsigned char ic_on_line:1;//IC在线状态
//    unsigned char ecm_on_line:1;//ECM在线状态
//    unsigned char tcu_on_line:1;//TCU在线状态
//    unsigned char steering_controller_1_on_line:1;//转向控制器1在线状态

//    unsigned char sas_sensor_on_line:1;//SAS传感器在线状态
//    unsigned char wheel_angle_sensor_on_line:1;//车轮角度传感器在线状态
//    unsigned char steering_controller_2_on_line:1;//转向控制器2在线状态
//    unsigned char wheel_angle_sensor_2_on_line:1;//车轮角度传感器2在线状态
//    unsigned char vcu_on_line:1;//VCU在线状态
//    unsigned char inertial_navigation_on_line:1;//惯导在线状态

//    unsigned char course_Angle_condition;//航向角状态(GPS航向初始化标志位)
//    unsigned char vehicle_status_marker;//车辆状态标志位
//    unsigned char integrated_navigation_overall_status_flag_bits;//组合导航总体状态标志位

//}FaultDiagnosisFrom297Info;

typedef struct
{
	int iCtrlStatus; 			// 0自动驾驶模式， 1遥控驾驶模式////////////////////

	int	iWheelAngle;			//车轮转角/////////////
    int	iThrottleOpening;		//油门开度//////////////
    int	iBrakeOpening;			//制动开度//////////////
    int	iGearOpening;			//档位操作///////////////
    int	iCombinedGearL;			//组合开关左
    int	iCombinedGearR;			//组合开关右
    int	iKeyOperation1;			//按键操作1
    int	iKeyOperation2;			//按键操作2
}RemoteDrivingWirectlInInfo;

typedef struct
{
	int iVehicle_speed; 			// 车速，单位：km/h////////////////
    int	iHeadingAngle;				//航向角
    int	iWheelAngleL;				//车轮转角//////////////
    int	iThrottleOpening;			//油门开度//////////////
    int	iBrakeOpening;				//制动开度/////////////////////
    int	iEngineSpeed;				//发动机转速
    int	iAccumulatedMileage;		//累计里程
    int	iSubtotalMileage;			//小计里程
}RemoteDrivingWirectlOutInfo;

typedef struct
{
    int remote_cmd;
}RemoteCtlInfo;

typedef struct{
    float filterValue;//滤波后的值,这个初值不重要,因为会不停更新
    float kalmanGain;//Kalamn增益
    float A;//状态矩阵,在一阶滤波器里面就是一个常数
    float H;//观测矩阵,在一阶滤波器里面就是一个常数
    float Q;//状态矩阵的方差,表示预测模型的噪声,是对预测模型的信任程度;
    float R;//观测矩阵的方差,表示测量模型的噪声,是对测量系统的信任程度;
    float P;//状态协方差,这个初值不重要,因为会不停更新
}KalmanInfo;

// typedef struct
// {
//     float front_wheel_angle;//前轮转角
//     float omega;//车辆运动角速度
//     float velocity;//车辆速度
//     float radii;//车辆质心转弯半径
//     float lat;//车辆纬度
//     float lon;//车辆经度
//     float head;//车头航向
//     int key;//车辆在当前路径中的关键点
// }VehicleInfo;


typedef struct
{
    char product_type;//产品类型
    char module_type;//产品类型
    char vehicle_type;//车辆类型
    char big_version;//大版本号
    char small_version;//小版本号
    unsigned short int vehicle_num;//车辆标号
    char reverse;
}SoftVersion;
typedef struct
{
    // fp path
    double fp_x;
    double fp_y;
    double fp_threat;
    double fp_k;
    int fp_size;
    int fp_index;
    int fp_index_add;
    // WGS84
    double lng;
    double lat;
    float head;
    float preview_curve;
    int wgs84_size;
    int wgs84_pointkey;
    // lqr
    int num_loop_lqr;
    double P_max_coef;
    double k[4];
    double forword_angle;
    double current_x;
    double current_y;
    double head_rad;
    double next_T_x;
    double next_T_y;
    double next_T_head;
    double K_angel;
    float pure_suit_angel;
    float stanley_angel;
    double ed_org;
    double ephi_org;
    double ed_d_org;
    void init()
    {
        fp_x = 0;
        fp_y = 0;
        fp_threat = 0;
        fp_k = 0;
        fp_size = 0;
        fp_index = 0;
        fp_index_add = 0;
        lng = 0;
        lat = 0;
        head = 0;
        wgs84_size = 0;
        wgs84_pointkey = 0;
        num_loop_lqr = 0;
        P_max_coef = 0;
        
        for (size_t i = 0; i < 4; i++)
        {
            k[i] = 0;
        }
        
        forword_angle = 0;
        current_x = 0;
        current_y = 0;
        head_rad = 0;
        next_T_x = 0;
        next_T_y = 0;
        next_T_head = 0;
        K_angel = 0;
        preview_curve = 0;
        pure_suit_angel = 0;
        stanley_angel = 0;
        ed_org = 0;
        ephi_org = 0;
        ed_d_org = 0;
    }
} ControlLogInfo;

//横向决策
typedef struct
{
    float heading_angle_deviation_at_prepoint;//当前点和预瞄点航向角偏差
    float angle_deviation_at_prepoint;//当前点和预瞄点方位角偏差
    float bia_distance;//横向距离偏差
    float preview_dis;//预瞄距离
    float preview_curve;   //预瞄曲率
    int preview_point;    //预瞄点索引
    float current_slope;     //预瞄坡度
    float current_slope1;   //预瞄
    float preview_velocity;  //预瞄速度
    float angle_deviation;
    double mLngs;
    double mLats;
    double tLng; // 实时经度
    double tLat; // 实时纬度
    int point_key;
    float cur_advpoint;//超前点曲率
    float cur_keypoint;//当前点曲率
} BasicInfo, PreviewInfo;
//横向控制
typedef struct
{
    float front_wheel_angle{0.0f};//期望前轮转角
}SteerControl;

//纵向控制
typedef struct
{
    float throttle;//油门开度
    float brake_pedal;//制动开度（缓速器或电制动）
    float mechanical_brake;//机械制动开度
    unsigned char park_ctrl;//驻车制动 1:驻车 0:解除
    bool Brake_Hode;//1 使能 0解除

    void init()
    {
        throttle = 0;
        brake_pedal = 0;
        mechanical_brake = 0;
        park_ctrl = 1;
        Brake_Hode = 0;
    };
    void runInit()
    {
        throttle = 0;
        brake_pedal = 0;
        mechanical_brake = 0;
        park_ctrl = 0;
        Brake_Hode = 0;
    };
} SpeedControl;

typedef struct
{
    // 轮胎侧偏刚度
    double cf;
    double cr;

    // 前后悬架载荷
    double mass_fl;
    double mass_fr;
    double mass_rl;
    double mass_rr;
    double mass_front;
    double mass_rear;
    double m;

    // 最大纵向加速度
    double max_lateral_acceleration;
    // 最大制动减速度
    double standstill_acceleration;
    // 轴距
    double wheel_base;
    // 前轴中心到质心的距离
    double a;
    // 后轴中心到质心的距离
    double b;

    // 车辆绕z轴转动的转动惯量
    double Iz;

    void init()
    {
        cf = 0;
        cr = 0;
        mass_front = 0;
        mass_rear = 0;
        m = 0;
        wheel_base = 0;
        a = 0;
        b = 0;
    }
} Vehicle_para;

// 故障诊断控制
typedef struct
{
    float desired_speed;     // 期望速度
    float throttle;          // 油门开度
    float brake_pedal;       // 制动开度（缓速器或电制动）
    float mechanical_brake;  // 机械制动开度
    unsigned char park_ctrl; // 驻车制动 1:驻车 0:解除
    float steer_ctrl;        // 转向控制
    int shift_ctrl;          // 档位控制
    int bucket_ctrl;         // 举升控制
    int remote_enable;       // 驾驶模式
    bool req_extern_assist;  // 请求平台控制 0:不请求  1:请求平台调度
    bool req_remote_assist;  // 请求遥控接管 0:不请求  1:请求遥控接管
    bool vcu_unman_enable;   // 请求VCU退出无人 0:退出
    bool need_acu_recov;     // ACU退出后申请恢复使能位

    void init()
    {
        vcu_unman_enable = true;
        remote_enable = 5;
        desired_speed = 0;
        throttle = 0;
        brake_pedal = 0;
        mechanical_brake = 0;
        steer_ctrl = 0;
        shift_ctrl = 0;
        bucket_ctrl = 0;
        req_extern_assist = false;
        req_remote_assist = false;
        park_ctrl = 0;
        need_acu_recov = false;
    };
    void runInit()
    {
        throttle = 0;
        brake_pedal = 0;
        mechanical_brake = 0;
        shift_ctrl = 0;
        park_ctrl = 0;
    };
} DiagMeasureControl;

// 附属动作决策
typedef struct
{
    short shift_ctrl;  // 档位 1:D 0:N -1:R
    short bucket_ctrl; // 货箱 0:保持 1:上升 2:下降
    bool whistle;      // 鸣笛
    float bucket_thr;  // 举升油门
    bool prepare_to_lift;
    unsigned char bucket_park_ctrl; // 驻车制动 1:驻车 0:解除
    bool Axle_difference;           // 轴差 1：开启 0：关闭
    void init()
    {
        shift_ctrl = 0;
        bucket_ctrl = 0;
        whistle = false;
        bucket_thr = 0;
        prepare_to_lift = false;
        bucket_park_ctrl = 0;
        Axle_difference = false;
    };
} ActDecision;
typedef struct
{
    int nodeNumber;//节点序号
    double endLatitude;//位置终止条件的纬度
    double endLongitude;//位置终止条件的经度
    std::string pathFileName;//路径文件名
    std::string boundaryFileName;//边界文件名
    float desiredSpeed;//期望车速
    int taskType;//任务类型

}Task_Info;
//增加任务类型枚举 == 2020.10.12 by ZhangR
// typedef enum
// {
//     UNKNOWN_TASK,//0未知任务
//     LOAD_PATH,//1载入路径
//     ONEWAY_PATH_EMPTY_LOAD,//2单向路径空载行驶
//     ONEWAY_PATH_FULL_LOAD,//3单向路径满载行驶
//     LOOPBACK_PATH,//4回环路径行驶
//     DUMP_OPERATIONS,//5排土作业
//     LOAD_OPERATIONS,//6装载作业
//     PARKING_SWITCH,//7停车切换
//     STOP_AND_WAIT,//8停车等待
//     NO_LOAD_REVERSE,//9空载倒车行驶
//     FULL_LOAD_REVERSE//10满载倒车行驶
// }Task_type;

typedef struct
{
    double x;
    double y;
    float head;
    float width;
    float left_width;
    float right_width;
    float velocity;
}Posture;

typedef struct
{
    float x;
    float y;
    float head;
    float length;
    float width;
}PostureAbb;

typedef struct
{
    float desired_speed;
    float x;
    float y;
    float head;
    float length;
    float width;
    float near_x;
    float near_y;
}SpeedJudgeInfoOut;

typedef struct
{
    float x;
    float y;
}Point2D;

struct LocalPoseInfo
{
    double x;
    double y;
    double z;
    float head;
    float speed;
    float yaw;
    float roll;
    float pitch;
    float slope;
    int point_key;
    bool isValid;
    bool isSlide;
};

struct VelInfo
{
    float desird_speed;
    float current_speed;
};

enum LoadStatus
{
    EMPTY_LOAD,
    FULL_LOAD,
    NONE_LOAD
};

enum class LaneDirection : int
{
  Backward = -1,
  Forward = 1,
  Error = 0
};

typedef struct{
    u8  u8VIN[17];      /*车辆VIN*/
    u16 u16Speed;       /*车速*/
    u8  u8Gear;         /*档位*/
    f64 u32Distance;    /*距离，精度cm*/
    f32 f32Heading;     /*偏航角，精度 0.001*/
    f32 f32Ptich;       /*俯仰角 0.001*/
    f32 f32Roll;        /*横滚角，精度 0.001*/
    f64 f64Longitude;   /*精度，精度 10-7*/
    f64 f64Latitude;    /*纬度，精度 10-7*/
    u32 dis;
}OtherVehicleInfo;

enum ParkType
{
    NONE_POINT,
    LOAD_POINT,
    DUMP_POINT,
    WAIT_POINT
};

enum TaskType
{
    LOAD_TASK,
    DUMP_TASK,
    WAIT_TASK,
    FUEL_TASK,
    CHARGE_TASK,
    MAINTAIN_TASK,
    PARK_TASK,
    STOP_TASK,
    CRUSH_TASK
};

#endif // STRUCT_COMMON_H
