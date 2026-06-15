#ifndef _REMOTE_CONFIG_H_
#define _REMOTE_CONFIG_H_
#include <string.h>
#include "./common/read_ini.h"
#include "protoDef.h"
#include "./common/yaml.h"
#include "zkbs/zkbs.hpp"
/*********************************************************************
 * CLASS
 */
#ifndef RET_OK
#define RET_OK 1
#define RET_ERROR 0
#endif

class Remote_Config
{
public:
    Remote_Config();
    ~Remote_Config();

    void Init();
    // 读取配置文件
    void ReadVehicleConfig();
	void ReadRemoteConfig();

public:
    /*配置文件*/
	std::string vehicle_iccid;
	std::string vehicle_vin;
	std::string vehicle_type;
	std::string current_part;
	std::string vehicle_HardVer = "0101";
	std::string vehicle_SoftVer = "0222";//02软件类型 20软件版本号 20不调整，21调整气压速度显示小数点后一位，增加刹车提醒；22 调整130 车型、水温显示;30新增障碍物、车道线等

	/*202605 排土场倒车*/
	SANY_U32 dumpBack_isOpen = 1;

	float boundary_stop_without_curb_dis = 3.0f; // 挡墙数据无效时，边界停车距离
	float boundary_curb_diff_dis = 3.0f; // 边界距离、挡墙距离的差异冗余，即挡墙距离有效的冗余
	float curb_stop_dis = 1.9f;    // 挡墙数据有效，连续5帧过近停车距离
	float boundary_stop_with_curb_dis = 1.2f;  // 挡墙数据有效时，边界停车距离

	// 倒车限扭矩
	float full_back_throttle_limit = 1.0f; // 重载倒车扭矩限制基准值
	float slope_throttle_compensation = 2.0f; // 坡度油门系数
	float dev_angle_throttle_compensation = 1.0f; // 抵墙卸料转角补偿系数
	float vehile_back_throttle_max = 52.0f;	// 倒车扭矩最大限制值

	// ----------俯仰角异常 阈值参数（静态或配置） ---------- 
	float top_level_pitch_limit = 1.0f;         // 二次起步判断高于俯仰角基准限值
	float low_level_pitch_limit = 0.8f;         // 二次起步判断低于俯仰角基准限值
	float pitch_abnormal_dis = 2.0f;           // 检测俯仰角异常的距离阈值
	float pitch_pair_chang_value  = 0.5f;      // 俯仰角波动异常阈值
	float up_level_pitch_max      = 3.5f;      // 上仰异常阈值
	float down_level_pitch_max    = 1.6f;      // 下降异常阈值



};

#endif
