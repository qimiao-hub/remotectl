#ifndef PUBAPI_H
#define PUBAPI_H

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <map>
#include <math.h>
#include <iostream>
#include <ostream>
#include <algorithm>
#include <fstream>
#include <dirent.h>
#include "struct_common.h"
#include "mutex.hpp"
#include "csingleton.hpp"

#include "zkbs/ddsBase/ddsPubBase.h"
#include "zkbs/ddsBase/ddsSubBase.h"

//#include "syzk_msgs/plan_msgPubSubTypes.h"
//#include "syzk_msgs/decide_msgPubSubTypes.h"
//#include "syzk_msgs/gps_msgPubSubTypes.h"
#include "plan_msgPubSubTypes.h"
#include "decide_msgPubSubTypes.h"
#include "gps_msgPubSubTypes.h"

#include "Geocentric/LocalCartesian.hpp"

using namespace GeographicLib;

#define DRIVE_UNMANNED_STATUS 	0 	/*无人驾驶状态*/
#define DRIVE_REMOTE_STATUS 	1	/*遥控驾驶状态*/

using namespace std;
using namespace common;

#define PREDICT_DATA_SAVE 1
#define V2N_DATA_RECORD 1
#define SENSE_DATA_RECORD 0
#define ZHONGCHUANG 1
#define ERROR_NUMBER 0.123456

#define setbit(x,y)  x|=(1<<y)  // 将x中的第y位至1
#define clrbit(x,y)  x&=~(1<<y)  // 将x中的第y位清0
#define reversebit(x,y)  x^=(1<<y)  // 将x中的第y位取反
#define getbit(x,y)   ((x) >> (y)&1) // 将x中的第y位取出

#define TEST_RUN 0
/*read config.ini file*/
#define INI_LINE_LEN 256 //config file line character length greater than 256 is invalid

#define ELEC_CAR 0
#define SKT90I   1
#define A04_PLAT   2
#define SKT90EI   3
#define SKT105E   4
#define SKT105I   5
#define SKT130HD   6

#define FAULT_TIMER 50 //50ms
#define DATA_FROM_297_TIME //50ms

enum INI_RESULT
{
    INI_SUCCESS, //成功
    INI_ERROR, //普通错误
    INI_OPENFILE_ERROR, //失败
    INI_NO_ATTR //无对应的键值
};
typedef map<std::string,std::string> INI_KEYMAP;//子键索引 子键值
typedef map<std::string,INI_KEYMAP> INI_MAINKEYMAP;//主键索引 主键值
/*read config.ini file*/


typedef enum {
    ERGODIC = 1,//遍历
    CURRENT_ERGODIC,//从当前点遍历
    CIRCLE_PARTIAL,//环形路径检索前方若干个点
    NONCIRCLE_PARTIAL//非环形路径检索前方若干个点
} KEY_SEARCH_METHOD;

typedef struct _GPSPos
{
  double iLatitude;//纬度
  double iLongitude;//经度
}GPSPos;

typedef struct _TaskInfo
{
  int tNum;        //序号
  float tSpeed;
  int tType;
  double tLng_end;     //经度
  double tLat_end;     //纬度
  string tPath;
}TaskInfo;


class PubApi:public CSingleton<PubApi>
{
public:
    PubApi();
    ~PubApi();
public:
    /*read config.ini file*/
    float getIniNum(const char *main_key, const char *key);
    float getIniNum(const char *main_key, const char *key, float org_val);
    double getIniNum(const char *main_key, const char *key, double org_val);
    int getIniNum(const char *main_key, const char *key, int org_val);
    string getIniStr(const char *main_key, const char *key);
    INI_RESULT getIniKey(const char *main_key, const char *key, char *value);
    INI_RESULT readIniFile(const char *path_name);
    INI_RESULT writeIniLog(INI_MAINKEYMAP &ini_map);
    float getDistance(double lat1,double lng1,double lat2,double lng2);
    float calXyDis(float x1,float y1,float x2,float y2);
    float calVerticalDis(float des_x,float des_y,float line_x1,float line_y1,float line_x2,float line_y2);//1225
    GPSPos calculateGpsByAngle(double longitude,double latitude,float head,float distance);
    float getAngleByTwoGps(double lat1,double lng1,double lat2,double lng2);//求第一个点指向第二个点的航向角 0~360°,顺时针方向
    float getAngleByTwoXy(float x1, float y1, float x2, float y2);
    void transGpsIntoInertia(GPSInfo &my_gps, GPSInfo &target_point,double *inertia_x, double *inertia_y);
    void transGpsIntoInertiaPerception(GPSInfo &my_gps, GPSInfo &target_point,double *inertia_x, double *inertia_y);
    void transGpsIntoGlobalInertia(GPSInfo &my_gps, GPSInfo &target_point,double *inertia_x, double *inertia_y);
    inline float cal_value(GPSInfo p1, GPSInfo p2);
    int findKeyPoint(const vector<GPSInfo> &road_list,double iLongitude,double iLatitude,double iHead,int key_last,KEY_SEARCH_METHOD search_method);

    float calDisByKeyPoint(int target_key,int point_key,vector<GPSInfo>* qGpsPathList);
    float getXDistanceBetweenPoses(VehiclePos currentpos, VehiclePos comparedpos);
    float getYDistanceBetweenPoses(VehiclePos currentpos, VehiclePos comparedpos);
    char * getVIN(const char *main_key, const char *key);
    float ramp(float up_rate, float input, float down_rate, float last_value);
    float saturation(float up_bound, float input, float low_bound);

    /*read config.ini file*/
public:
    void initKalmanFilter(KalmanInfo *info,float Q,float R);
    float kalmanFilterFun(KalmanInfo *info,float new_value);
    bool checkGps(const char *gps_data);
    string getFiles(const string path);

    float filterChangeLimit(float new_value, float last_value, float limit);//变化量限幅函数
    float filterChangeLimit(float new_value, float last_value, float increase_limit, float decrease_limit);
    void clamp(int &raw_value, int low_limit, int high_limit); //整形值限幅
    void clamp(float &raw_value, float low_limit, float high_limit); //浮点数限幅
    int getVarSection(float &var_value, volatile float var_array[], int array_size);//计算变量所处的区间
    float lookupTable1D(float var_value, volatile float var_array[], int array_size, volatile float table_data[]);//一维查表
    float lookupTable2D(float column_value, volatile float column_array[], int column_size,
                        float row_value, volatile float row_array[], int row_size,
                        volatile float table_data[5][5]);//二维查表，一定注意行和列不要反了,维度为5*5
    float lookupTable2D(float column_value, volatile float column_array[], int column_size,
                        float row_value, volatile float row_array[], int row_size,
                        volatile float table_data[8][7]);//二维查表，一定注意行和列不要反了,维度为8*7
    float lookupTable2D(float column_value, volatile float column_array[], int column_size,
                        float row_value, volatile float row_array[], int row_size,
                        volatile float table_data[10][10]);//二维查表，一定注意行和列不要反了,维度为10*10

    float normRadAngle(float angle);
    float getDeltAngle(float park_head,float road_head);
    LocalCartesian geo_translation;

public:
    /*config parameter*/
    // vehicle
    static volatile float vehicle_mass;       // 车辆质量，kg
    static volatile float vehicle_length;     // 车辆长度,m
    static volatile float vehicle_width;      // 车辆宽度,m
    static volatile float vehicle_height;     // 车辆高度,m
    static volatile float vehicle_wheel_base; // 车辆轴距,m
    static volatile float vehicle_center_front_ratio;
    static volatile float vehicle_left_max_steer_wheel;  // 前轮左转最大转角,degree
    static volatile float vehicle_right_max_steer_wheel; // 前轮右转最大转角,degree
    static volatile float vehicle_wheel_radius;          // 轮胎半径,m
    static volatile float vehicle_main_ratio;            // 主传动比
    static volatile float vehicle_trans_ratio_D;
    static volatile float vehicle_trans_ratio_D1;  //挡位传动比
    static volatile float vehicle_trans_ratio_D2;  //
    static volatile float vehicle_trans_ratio_D3;  //
    static volatile float vehicle_trans_ratio_D4;  //
    static volatile float vehicle_trans_ratio_D5;  //
    static volatile float vehicle_trans_ratio_D6;  //
    static volatile float vehicle_trans_ratio_D7;  //    
    static volatile float vehicle_trans_ratio_R;  //
    static volatile int vehicle_type;  //车辆类型，0:电动小车，1:90i宽体车
    static volatile char vehicle_VIN[18];//车辆VIN
    static volatile float front_wheel_base;//前轴距
    static volatile float rear_wheel_base;//后轴距

    //net
    static volatile char net_server1name[22];
    static volatile bool b_fix_port_ip1;
    static volatile char net_client1name[22];

    static volatile char net_server2name[22];
    static volatile bool b_fix_port_ip2;
    static volatile char net_client2name[22];
    //deviation
    static volatile int is_use_front_radar;            //是否启用前毫米波雷达,1:是 0：不
    static volatile int is_use_back_radar;            //是否启用后毫米波雷达,1:是 0：不
    static volatile int is_use_front_lidar;            //是否启用前激光雷达,1:是 0：不
    static volatile int is_use_back_lidar;            //是否启用后激光雷达,1:是 0：不
    static volatile float radar_front_dist_long_deviation;//前毫米波雷达Y坐标值
    static volatile float radar_front_dist_lat_deviation;//前毫米波雷达X坐标值
    static volatile float radar_back_dist_long_deviation;//后毫米波雷达Y坐标值
    static volatile float radar_back_dist_lat_deviation;//后毫米波雷达X坐标值
    static volatile float rotate_lidar_dist_long_deviation;//转台雷达Y坐标值
    static volatile float rotate_lidar_dist_lat_deviation;//转台雷达X坐标值
    static volatile float lidar_back_dist_long_deviation;//后激光雷达Y坐标值
    static volatile float lidar_back_dist_lat_deviation;//后激光雷达X坐标值
    static volatile float front_axis_dist_deviation;//车辆前轮中心轴的Y坐标值
    static volatile float back_axis_dist_deviation;//车辆后轮中心轴的Y坐标值
    static volatile float back_aerial_dist_lat_deviation;//后天线的X坐标值
    static volatile float back_aerial_dist_long_deviation;//后天线的Y坐标值
    static volatile float back_aerial_dist_z_deviation;//后天线的z坐标值
    static volatile float imu_dist_long_deviation;//惯导的Y坐标值
    static volatile float imu_dist_lat_deviation;//惯导的X坐标值

    // decide
    static volatile bool decide_plat_mode;   //0单车模式，1平台模式
    static volatile bool decide_avoid_mode;   //0单车模式，1平台模式
    static volatile float decide_drive_speed_up_bound; // 正常行驶(空载和满载）速度限幅，km/h
    static volatile float decide_drive_speed_down_bound; // 正常行驶(空载和满载）速度限幅，km/h
    static volatile float decide_lift_forward_const_speed; // 卸载前行速度，km/h
    static volatile float decide_forward_const_speed; // 卸载前行速度，km/h
    static volatile float decide_back_const_speed; // 泊车速度，km/h
    static volatile float decide_dis_for_ask_pos;  // 提前申请停车点距离，m
    static volatile float decide_dis_for_ask_plan;     //提前请求规划距离，m
    static volatile float decide_dis_for_ask_planPre;     //提前请求规划距离，m

    static volatile float decide_dis_for_stop;     //等待区停车距离，m
    static volatile float decide_park_speed_up_bound; // 定点停车速度限幅，km/h
    static volatile float decide_park_k; // 定点停车刹车线性方程系数k
    static volatile float decide_park_b;   //定点停车刹车线性方程系数b
    static volatile double decide_load_pos_lng;   //装载点坐标
    static volatile double decide_load_pos_lat;   //
    static volatile float decide_load_pos_head;   //
    static volatile double decide_dump_pos_lng;   //卸载点坐标
    static volatile double decide_dump_pos_lat;   //
    static volatile float decide_dump_pos_head;   //

    static volatile float dumping_forward_dis; //
    static volatile float dumping_back_dis; //
    static volatile float loading_forward_dis;   //
    static volatile float loading_back_dis;   //
    static volatile float parking_forward_dis;   //
    static volatile float parking_back_dis;   //
    static volatile float chargeing_driving_dis;   //
    static volatile float parking_driving_dis;   //
    static volatile float dumping_forward_lifting_dis;
    static volatile float load_wait_time;
    static volatile float lifting_wait_time;
    static volatile float lifted_wait_time;
    static volatile float continue_angle_diff;
    static volatile float continue_dis_diff;

    //丢失前向感知判断次数
    static volatile int losePercpFrontIndex;
    //丢失后向感知判断次数
    static volatile int losePercpBackIndex;
    //丢失V2V的判断次数
    static volatile int loseV2VIndex;
    //基于V2V交叉路口让行，1为打开，0为关闭
    static volatile int v2v_cross_yield_mode;
    //基于V2V交叉路径让行预瞄路径距离
    static volatile int v2vCrossRoadPreDis;
    //判断为会车的距离差
    static volatile float mid_side_delt_dis;
    //判断为会车的角度差
    static volatile float mid_side_delt_head;
    //会车的期望车速
    static volatile float keep_mid_desire_spd;
    //行驶到主路径释放装料区距离
    static volatile float release_load_area_dis;
    //行驶到主路径释放卸料区距离
    static volatile float release_dump_area_dis;
    //二次泊车横向偏差
    static volatile float reparking_dis_x;
    //二次泊车纵向偏差
    static volatile float reparking_dis_y;
    //二次泊车角度差
    static volatile float reparking_dis_head;
    //无v2v的挖机通信方式，1为开启，0为关闭
    static volatile int no_v2v_load_mode;
    //绕障前的前车静止时间判断
    static volatile float avoid_obst_time;
    //举斗前进功能，1为开启，0为关闭
    static volatile int lifting_forward_dis;
    //屏蔽装料点驶出部分路段感知,不屏蔽改成-1
    static volatile float shield_out_load_area_dis;
    //屏蔽卸料点驶出部分路段感知，不屏蔽改成-1
    static volatile float shield_out_dump_area_dis;
    //感知遇障距离、系数5km/h
    static volatile float perception_stop_dis_down_5kmh;
    static volatile float perception_stop_index_5kmh;
    //感知遇障距离、系数10km/h
    static volatile float perception_stop_dis_down_10kmh;
    static volatile float perception_stop_index_10kmh;
    //感知遇障距离、系数15km/h
    static volatile float perception_stop_dis_down_15kmh; 
    static volatile float perception_stop_index_15kmh;
    //感知遇障距离、系数20km/h
    static volatile float perception_stop_dis_down_20kmh;
    static volatile float perception_stop_index_20kmh;
    //感知遇障距离、系数25km/h
    static volatile float perception_stop_dis_down_25kmh;
    static volatile float perception_stop_index_25kmh;
    //感知遇障距离、系数30km/h
    static volatile float perception_stop_dis_down_30kmh;
    static volatile float perception_stop_index_30kmh;
    //基于感知绕障距离，在遇障停车基础上增加检测距离
    static volatile float perception_avoid_add_dis;
    //v2v碰撞检测，障碍车尺寸
    static volatile float v2v_obst_vehicle_length;
    static volatile float v2v_obst_vehicle_width;
    //跟车限速在5-15km/h
    //基于v2v遇障停车距离、减速系数、跟车距离5km/h
    static volatile float v2v_stop_dis_down_5kmh;
    static volatile float v2v_stop_index_5kmh;
    static volatile float v2v_follow_dis_down_5kmh;
    //基于v2v遇障停车距离、减速系数、跟车距离10km/h
    static volatile float v2v_stop_dis_down_10kmh;
    static volatile float v2v_stop_index_10kmh;
    static volatile float v2v_follow_dis_down_10kmh;
    //基于v2v遇障停车距离、减速系数、跟车距离15km/h
    static volatile float v2v_stop_dis_down_15kmh;
    static volatile float v2v_stop_index_15kmh;
    static volatile float v2v_follow_dis_down_15kmh;
    //基于v2v遇障停车距离、减速系数、跟车距离20km/h
    static volatile float v2v_stop_dis_down_20kmh;
    static volatile float v2v_stop_index_20kmh;
    static volatile float v2v_follow_dis_down_20kmh;
    //基于v2v遇障停车距离、减速系数、跟车距离25km/h
    static volatile float v2v_stop_dis_down_25kmh;
    static volatile float v2v_stop_index_25kmh;
    static volatile float v2v_follow_dis_down_25kmh;
    //基于v2v遇障停车距离、减速系数、跟车距离30km/h
    static volatile float v2v_stop_dis_down_30kmh;
    static volatile float v2v_stop_index_30kmh;
    static volatile float v2v_follow_dis_down_30kmh;

    // obstavoid
    static volatile float obstavoid_slow_k_factor;   //遇障基于剩余路长的降速系数
    static volatile int obstavoid_filter_up_count;   //遇障滤波器计数器上限
    static volatile float obstavoid_filter_dis;   //遇障滤波距离,m
    static volatile float obstavoid_brake_bound;   //前行遇障刹车势场距离,m
    static volatile float obstavoid_brake_back_bound;   //倒车遇障刹车势场距离,m
    static volatile float obstavoid_one_side_width;   //遇障区域单侧宽度,m
    static volatile float obstavoid_obst_length;   //遇障区域长度,m
    static volatile float obstavoid_delta_length;   //遇障细分长度,m
    static volatile float obstavoid_default_ttc;   //遇障默认障碍ttc,s
    static volatile float obstavoid_default_y;   //遇障默认纵向距离,m
    static volatile float obstavoid_default_x;   //遇障默认横向距离,m
    static volatile float obstavoid_curve_default_x;
    static volatile float obstavoid_curve_default_y;
    static volatile float obstavoid_filter_z;
    static volatile float obstavoid_curve_default_angle;
    static volatile bool fifo_flag;
    static volatile bool percep_front_flag;
    static volatile bool percep_back_flag;

    static volatile float boundary_detect_dis;
    static volatile float plan_detect_dis;
    static volatile float back_detect_dis;

    static volatile float abb_vehicle_length;
    static volatile float abb_vehicle_width;

    //control
    static volatile float control_spd_kp;          // 速度PID参数kp
    static volatile float control_spd_ki;          // 速度PID参数ki
    static volatile float control_spd_kd;          // 速度PID参数kd
    static volatile float control_spd_integ_up;          //速度PID参数,积分限幅
    static volatile float control_spd_integ_dn;          //速度PID参数
    static volatile float control_spd_diff_up;          //速度PID参数，微分限幅
    static volatile float control_spd_diff_dn;          //速度PID参数
    static volatile float control_spd_out_up;          //速度PID参数,输出限幅
    static volatile float control_spd_out_dn;          //速度PID参数
    static volatile float control_delta_t;          //主控周期,s
    static volatile float control_inherent_acc;          //车辆固有加速度,m/s2
    static volatile float control_gravity;          //重力加速度,m/s2
    static volatile float control_mass_factor;          //质量系数
    static volatile float control_roll_factor;          //滚阻系数
    static volatile float control_engine_torque_max;          //发动机最大转矩,Nm
    static volatile float control_retarder_max_torque;         // 缓速器最大扭矩,Nm
    static volatile float control_brake_max_torque;            // 行车制动最大扭矩,Nm
    static volatile float control_throttle_empty_stroke;          //油门空行程,%
    static volatile float control_throttle_rate_limit;          //油门主控周期内速度限幅
    static volatile float control_brake_rate_limit;          //缓速主控周期内速度限幅
    static volatile float control_mechanical_brake_rate_limit;          //行车制动主控周期内速度限幅
    static volatile float control_dev_angle_factor;          // 方位角误差系数
    static volatile float control_dev_heading_factor;          // 航向角误差系数
    static volatile float control_forward_pre_dis;          //前行最小预瞄距离,m
    static volatile float control_dev_angle_back_factor;          // back方位角误差系数
    static volatile float control_dev_heading_back_factor;          // back航向角误差系数
    static volatile float control_backward_pre_dis;          //倒车最小预瞄距离,m
    static volatile float control_preview_time;          //单点预瞄时间,s
    static volatile float control_far_preview_dis;          //超前点距离,m
    static volatile float control_throtle_threshold;     //油门介入控制的门限值，高于此值，油门介入控制
    static volatile float control_brake_threshold;     //制动介入控制的门限值，低于此值，制动介入控制
    static volatile float control_steer_zero_compensate;
    static volatile float control_angle_factor_medium_curve;
    static volatile float control_angle_factor_large_curve;
    static volatile float control_retarder_factor_empty;
    static volatile float control_tram;          // 速度PID参数kp
    static volatile float control_velocity_enable_threshold;
    static volatile float control_steer_wheel_kp;
    static volatile float control_steer_wheel_ki;
    static volatile float control_steer_wheel_kd;
    static volatile float control_steer_wheel_coefficient;
    static volatile float control_break_mode_threshold;
    static volatile float control_preview_dis_tolerance;
    static volatile float control_preview_dis_tolerance_Park;
    static volatile float control_preview_dis_tolerance_R;
    static volatile float control_curve_detection_preview_dis;
    static volatile float control_time;
    static volatile float control_time_delt;
    static volatile float control_preview_dis;
    static volatile float control_steer_delt;
    static volatile float control_speed_erroe_coefficient;
    static volatile int control_bucket_throttle;
    static volatile int control_bucket_empty_min_throttle;
    static volatile int control_bucket_empty_max_throttle;
    static volatile int control_bucket_full_min_throttle;
    static volatile int control_bucket_full_max_throttle;
    static volatile int control_shaft_speed_error;
    static volatile float control_up_hill_min;
    static volatile float control_up_hill_max;
    static volatile float control_down_hill_min;
    static volatile float control_down_hill_max;
    static volatile int control_bucket_full;
    static volatile int control_bucket_empty_full;    

    static volatile float control_empty_throttle_acc_min;
    static volatile float control_empty_throttle_acc_max;
    static volatile float control_empty_throttle_acc_det;
    static volatile float control_empty_throttle_speed_min;
    static volatile float control_empty_throttle_speed_max;
    static volatile float control_empty_break_acc_min;
    static volatile float control_empty_break_acc_max;
    static volatile float control_empty_break_acc_det;
    static volatile float control_empty_break_speed_min;
    static volatile float control_empty_break_speed_max;
    static volatile float control_full_throttle_acc_min;
    static volatile float control_full_throttle_acc_max;
    static volatile float control_full_throttle_acc_det;
    static volatile float control_full_throttle_speed_min;
    static volatile float control_full_throttle_speed_max;
    static volatile float control_full_break_acc_min;
    static volatile float control_full_break_acc_max;
    static volatile float control_full_break_acc_det;
    static volatile float control_full_break_speed_min;
    static volatile float control_full_break_speed_max;
    static volatile int point_key_add;
    //105 ACC PID参数
    static volatile float acc_pid_kp;
    static volatile float acc_pid_ki;
    static volatile float acc_pid_kd;
    static volatile float empty_brake_approch_speed;
    static volatile float full_brake_approch_speed;
    //static volatile float dumping_forward_lifting_dis;

    // platform
    static volatile float platform_circle_test;          //绕圈测试
    static volatile float platform_perception_start;      //启动感知

    //plan
    static volatile float lengh_ag;
    static volatile float lengh_ab_max;
    static volatile float lengh_ab_min;
    static volatile float lengh_bc_max;
    static volatile float lengh_bc_min;
    static volatile float lengh_cd_max;
    static volatile float lengh_cd_min;
    static volatile float drift_angle_max;
    static volatile float drift_angle_min;
    static volatile float drift_angle_delt;
    static volatile float lengh_delt;
    static volatile float arc_raidus_min;
    static volatile float arc_radius_max;
    static volatile float arc_radius_delt;
    static volatile float rho_raidus_min;
    static volatile float rho_radius_max;
    static volatile float rho_radius_delt;
    static volatile float plan_min_radius;
    static volatile float plan_forwad_index;
    static string logfilename;

    static string logfilename_control;
    static string empty_throttle_name;
    static string empty_break_name;
    static string full_throttle_name;
    static string full_break_name;
    static volatile float obst_start_break_coefficient;

    static volatile float empty_max_speed_error;
    static volatile float empty_max_mechanical_brake;
    static volatile float empty_max_brake_pedal;
    static volatile float empty_max_throttle;
    static volatile float empty_up_hill_compensate_coefficient;
    static volatile float empty_down_hill_compensate_coefficient;
    static volatile float empty_flat_throttle_increment;
    static volatile float empty_slope_start_time;
    static volatile float empty_slope_start_throttle;
    static volatile float empty_gps_speed_error_coefficient;
    static volatile float empty_flat_speed_error_compensate_coefficient;
    static volatile float empty_down_hill_speed_error_compensate_coefficient;
    static volatile float empty_break_delt_coefficient;
    static volatile float empty_throttle_delt_coefficient;
    static volatile float empty_min_break_threshold;
    static volatile float empty_down_hill_speed_error_threshold;
    static volatile float empty_down_hill_max_throttle;
    static volatile float empty_down_hill_break_compensate_coefficient;
    static volatile float empty_break_smooth_coefficient;
    static volatile float empty_speed_error_mechanical_threshold;
    static volatile float empty_speed_error_mechanical_coefficient;
    static volatile float empty_pitch_throttle_compensate_coefficient;
    static volatile float empty_max_throttle_coefficient;
    static volatile float empty_min_throttle_coefficient;
    static volatile float empty_flat_start_min_throttle;
    static volatile float empty_up_hill_start_min_throttle;
    static volatile float empty_down_hill_start_min_throttle;
    static volatile float empty_start_speed_thrsehold_coefficient;
    static volatile float empty_slope_throttle_cofficient;
    static volatile float empty_mechanical_brake_index;
    static volatile float empty_gpitch_brake_index;
    static volatile float empty_slope_mechanical_threshold;
    static volatile int slope_delay_flag;//
    static volatile float slope_delay_spd_d;//
    static volatile float slope_delay_slope;//

    static volatile float acc_smooth_gain_Thr_bigSlope;
    static volatile float acc_smooth_Full_Load_BrkFullDec;

    static volatile float full_max_speed_error;
    static volatile float full_max_mechanical_brake;
    static volatile float full_max_brake_pedal;
    static volatile float full_max_throttle;
    static volatile float full_up_hill_compensate_coefficient;
    static volatile float full_down_hill_compensate_coefficient;
    static volatile float full_flat_throttle_increment;
    static volatile float full_slope_start_time;
    static volatile float full_slope_start_throttle;
    static volatile float full_gps_speed_error_coefficient;
    static volatile float full_flat_speed_error_compensate_coefficient;
    static volatile float full_down_hill_speed_error_compensate_coefficient;
    static volatile float full_break_delt_coefficient;
    static volatile float full_throttle_delt_coefficient;
    static volatile float full_min_break_threshold;
    static volatile float full_down_hill_speed_error_threshold;
    static volatile float full_down_hill_max_throttle;
    static volatile float full_down_hill_break_compensate_coefficient;
    static volatile float full_break_smooth_coefficient;
    static volatile float full_speed_error_mechanical_threshold;
    static volatile float full_speed_error_mechanical_coefficient;
    static volatile float full_pitch_throttle_compensate_coefficient;
    static volatile float full_max_throttle_coefficient;
    static volatile float full_min_throttle_coefficient;
    static volatile float full_flat_start_min_throttle;
    static volatile float full_up_hill_start_min_throttle;
    static volatile float full_down_hill_start_min_throttle;
    static volatile float full_start_speed_thrsehold_coefficient;
    static volatile float full_slope_throttle_cofficient;
    static volatile float test_desird_speed;
    static volatile float min_pos_angle_weight;
    static volatile float min_pos_dis_threshold;
    /*throttle*/
    static volatile float lon_control_kp;
    static volatile float lon_control_ki;
    static volatile float lon_control_kd;
    static volatile float lon_control_pid_upper;
    static volatile float lon_control_pid_lower;
    /*brake*/
    static volatile float lon_control_kp1;
    static volatile float lon_control_ki1;
    static volatile float lon_control_kd1;
    static volatile float lon_control_pid_upper1;
    static volatile float lon_control_pid_lower1;
    /*accel*/
    static volatile float lon_acc_control_kp;
    static volatile float lon_acc_control_ki;
    static volatile float lon_acc_control_kd;
    static volatile float lon_acc_control_pid_upper;
    static volatile float lon_acc_control_pid_lower;

    static volatile float AccTestVehSpeed;
     static volatile float empty_break_coff;
     static volatile float full_break_coff;

     static volatile float empty_slope_delt_throttle_cofficient;
     static volatile float full_slope_delt_throttle_cofficient;

     static volatile float empty_slope_break_cofficient;
     static volatile float empty_slope_delt_break_cofficient;
     static volatile float full_slope_break_cofficient;
     static volatile float full_slope_delt_break_cofficient;

     static volatile float enable_retarder_min_dis;
     static volatile float retarder_gain;

     static volatile float acc_smooth_gain_accRtr;
     static volatile float acc_smooth_gain_accBrk;
     static volatile float acc_smooth_gain_accBrkNeg;
     static volatile float acc_smooth_gain_accThr;
     static volatile float acc_follow;
     static volatile float acc_smooth_gain_Thr;
     static volatile float acc_dec_full;
     static volatile float acc_dec_full_big_slope;
     static volatile float acc_smooth_gain_Rtr;
     static volatile float acc_smooth_gain_Brk;
     static volatile float acc_smooth_gain_BrkFull;
     static volatile float acc_smooth_gain_BrkFullDec;

     static volatile float min_control_threshold_thr;
     static volatile float min_control_threshold_thr_down;
     static volatile float min_control_threshold_thr_down_full;
     static volatile float min_control_threshold_brk;

     static volatile float dec_gain;
     static volatile float lon_control_stop_dec_time;

     static volatile float lon_control_gps_speed_gain;
     static volatile float lon_control_shaft_speed_gain;
     static volatile float lon_control_vehicle_speed_gain;

    static volatile float bucket_speed_full_max;
    static volatile float bucket_speed_full_min;
    static volatile float bucket_speed_empty_max;
    static volatile float bucket_speed_empty_min;
    static volatile float dump_forword_lift_dis;
    static volatile float left_collision_dis_threshold;
    static volatile float right_collision_dis_threshold;
    static volatile float middle_collision_dis_threshold;
    static volatile int turning_test_flag;
    static volatile int throttle_increase_test_flag;
    static volatile float straight_curve_threshold;
    static volatile float straight_curve_steer_max_delt;

    static volatile float lat_control_preview_dis;
    static volatile float full_slope_break_cofficient_1;
    static volatile float full_slope_break_cofficient_2;

    static volatile float lat_control_kp;
    static volatile float lat_control_ki;
    static volatile float lat_control_kd;
    static volatile float lat_control_pid_upper;
    static volatile float lat_control_pid_lower;
    static volatile float lat_control_pursuit_weight;
    static volatile float lat_control_stanley_weight;
    static volatile float lat_control_dwa_weight;
    static volatile float lat_control_dev_angle_factor;
    static volatile float lat_control_dev_heading_factor;
    static volatile float lat_control_dev_angle_back_factor;
    static volatile float lat_control_dev_heading_back_factor;
    static volatile float lat_control_angle_factor_medium_curve;
    static volatile float lat_control_angle_factor_large_curve;
    static volatile float lat_control_time;
    static volatile float lat_control_time_delt;
    static volatile float lat_control_steer_delt;
    static volatile float lat_stanley_gain;
    static volatile float lat_min_pos_angle_weight;
    static volatile float lat_steer_ratio_max;
    static volatile float lat_lqr_gain;
    static volatile float lat_puresuit_gain;
    static volatile float lat_lqr_gain_R;
    static volatile float lat_puresuit_gain_R;

    static volatile float lon_control_dec_gain;
    static volatile float lon_control_slope_start_time;
    static volatile float lon_control_slope_start_throttle;
    static volatile float lon_control_min_control_threshold;
    static volatile float lon_control_acc_smooth_gain;
    static volatile float lon_control_max_brake_pedal;
    static volatile float lon_control_max_mechanical_brake;
    static volatile float lon_control_max_throttle;
    static volatile float lon_control_enable_retarder_min_dis;
    static volatile float lon_control_empty_slope_throttle_cofficient;
    static volatile float lon_control_empty_slope_delt_throttle_cofficient;
    static volatile float lon_control_full_slope_throttle_cofficient;
    static volatile float lon_control_full_slope_delt_throttle_cofficient;
    static volatile float lon_control_retarder_gain;
    static volatile float lon_control_empty_slope_break_cofficient;
    static volatile float lon_control_full_slope_break_cofficient;

    static volatile int lose_decide_count;
    static volatile int lose_297_count;

    static volatile float Q1;
    static volatile float Q2;
    static volatile float Q3;
    static volatile float Q4;
    static volatile float R1;

    static volatile int LQR_Loop;
    static volatile double LQR_Min_Loop_val;

    static volatile float mass_front;
    static volatile float mass_rear;
    static volatile float mass_front_full;
    static volatile float mass_rear_full;
    static volatile float cf_cr;

    static volatile double ed_limit_min;
    static volatile double ed_limit_max;
    static volatile double ephi_limit_min;
    static volatile double ephi_limit_max;

    // 风阻系数 路面阻力系数
    static volatile float Kb;
    static volatile double Cd;
    static volatile double Veh_A;
    static volatile double air_density;
    static volatile double f0;
    static volatile double f1;
    static volatile double motor_effi;
    static volatile int spd_empty_max;
    static volatile int spd_full_max;

    //  扭矩控制切换条件
    static volatile double accTorCtrlThr;

    static volatile int lqr_on;

    static void coutToFileFunc(string str);
protected:
    /*read config.ini file*/
    FILE* ini_fp;//ini file pointer
    char ini_value[INI_LINE_LEN];
    INI_MAINKEYMAP ini_map;
    /*read config.ini file*/

    Mutex bit_map_mutex;
};


#endif // PUBAPI_H
