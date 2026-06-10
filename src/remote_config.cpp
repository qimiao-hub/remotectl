#include "../include/remote_config.h"


#define CONFIG_PATH "/data/sany/config/runSetup.yaml"
#define LINES_CFG_PATH "../config/remote_config.ini"

Remote_Config::Remote_Config()
{
	
}

Remote_Config::~Remote_Config(){
	
}

void Remote_Config::Init(){
    ReadVehicleConfig();
    // ReadRemoteConfig();
}
/*读取配置文件*/
void Remote_Config::ReadVehicleConfig(){

    YAML::Node config = YAML::LoadFile(CONFIG_PATH);
	if(! config)
	{
		MINE_LOG(SANY_LOG_ERROR, "Read Config-runSetup.yaml fail!!!");
		return;
	}
	else
	{
		 vehicle_vin = config["vehicle"]["vehicle_VIN"].as<std::string>();
		 vehicle_iccid = config["vehicle"]["vehicle_ICCID"].as<std::string>();
		SANY_U32 config_v_type = config["vehicle"]["vehicle_type"].as<int>();
	//	brake_delay = config["vehicle"]["vehicle_delay"].as<int>();
	//	read_brakeing = config["vehicle"]["vehicle_brake"].as<int>();
		// 1 为油车
		if(config_v_type == 1)
		{
			 vehicle_type = "SKT-I";
		}
		// 2 为电车
		else if(config_v_type == 2)
		{
			 vehicle_type = "SKT-E";
		}
		/* 20250306 liuss31 新增车型 3 为 130H混型-------- start*/ 
		else if(config_v_type == 3)
		{
			 vehicle_type = "SKT-H";
		}
		else{
			// 默认油车
			MINE_LOG(SANY_LOG_ERROR, "Read Config-runSetup.yaml vehicle_type is %d, SET 1 (SKT-I) !!!\n", config_v_type);
			config_v_type = 1;
			 vehicle_type = "SKT-I";
		}
		/* 20250306 liuss31 新增车型 130H混型-------- end*/ 
		std::string s = config["system"]["currentPart"].as<std::string>();

		 std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return std::tolower(c); });
				   
		current_part = "syzk_" + s;
		MINE_LOG(SANY_LOG_INFO, "Vehicle VIN is %s\n",  vehicle_vin.c_str());
		MINE_LOG(SANY_LOG_INFO, "Vehicle ICCID is %s\n",  vehicle_iccid.c_str());
		MINE_LOG(SANY_LOG_INFO, "Vehicle type is %s\n",  vehicle_type.c_str());
		MINE_LOG(SANY_LOG_INFO, "Current_Part is %s\n", current_part.c_str());
	}
}

/*读取车道线配置文件 2025.03.24 start */
void Remote_Config::ReadRemoteConfig(){
	ini::iniReader config;
	bool ret = config.ReadConfig(LINES_CFG_PATH);
	if(ret == false){
		MINE_LOG(SANY_LOG_ERROR, "Read Config remote_lines.ini fail!!\n");
		return;
	}
	


	dumpBack_isOpen = config.ReadInt("DumpBack", "dumpBack_isOpen", 1);

	boundary_stop_without_curb_dis = config.ReadFloat("DumpBack", "boundary_stop_without_curb_dis", 3.0f); 
	curb_stop_dis = config.ReadFloat("DumpBack", "curb_stop_dis", 1.9f);  
	boundary_stop_with_curb_dis = config.ReadFloat("DumpBack", "boundary_stop_with_curb_dis", 1.2f);  

	full_back_throttle_limit = config.ReadFloat("DumpBack", "full_back_throttle_limit", 34.0f);
	slope_throttle_compensation = config.ReadFloat("DumpBack", "slope_throttle_compensation", 2.0f);
	dev_angle_throttle_compensation = config.ReadFloat("DumpBack", "dev_angle_throttle_compensation", 1.0f);
	vehile_back_throttle_max = config.ReadFloat("DumpBack", "vehile_back_throttle_max", 52.0f);

	pitch_abnormal_dis = config.ReadFloat("DumpBack", "pitch_abnormal_dis", 2.0f);         
	pitch_pair_chang_value = config.ReadFloat("DumpBack", "pitch_pair_chang_value", 0.5f);
	up_level_pitch_max = config.ReadFloat("DumpBack", "up_level_pitch_max", 3.0f);
	down_level_pitch_max = config.ReadFloat("DumpBack", "down_level_pitch_max", 3.0f);

	MINE_LOG(SANY_LOG_INFO, "dumpBack_isOpen  (%d)\n", dumpBack_isOpen);

	MINE_LOG(SANY_LOG_INFO, "boundary_stop_without_curb_dis (%f)\n",  boundary_stop_without_curb_dis);
	MINE_LOG(SANY_LOG_INFO, "curb_stop_dis(%f)\n",  curb_stop_dis);
	MINE_LOG(SANY_LOG_INFO, "boundary_stop_with_curb_dis (%f)\n",  boundary_stop_with_curb_dis);

	MINE_LOG(SANY_LOG_INFO, "full_back_throttle_limit (%f)\n",  full_back_throttle_limit);
	MINE_LOG(SANY_LOG_INFO, "slope_throttle_compensation(%f)\n",  slope_throttle_compensation);
	MINE_LOG(SANY_LOG_INFO, "dev_angle_throttle_compensation (%f)\n",  dev_angle_throttle_compensation);
	MINE_LOG(SANY_LOG_INFO, "vehile_back_throttle_max(%f)\n",  vehile_back_throttle_max);

	MINE_LOG(SANY_LOG_INFO, "pitch_abnormal_dis (%f)\n",  pitch_abnormal_dis);
	MINE_LOG(SANY_LOG_INFO, "pitch_pair_chang_value (%f)\n",  pitch_pair_chang_value);
	MINE_LOG(SANY_LOG_INFO, "up_level_pitch_max(%f)\n",  up_level_pitch_max);
    MINE_LOG(SANY_LOG_INFO, "down_level_pitch_max(%f)\n",  down_level_pitch_max);
}
/*读取车道线配置文件 2025.03.24 end */
