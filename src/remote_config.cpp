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
		MINE_LOG(SANY_LOG_INFO, "Vehicle VIN is %s\n",  vehicle_vin.c_str());
		MINE_LOG(SANY_LOG_INFO, "Vehicle ICCID is %s\n",  vehicle_iccid.c_str());
		MINE_LOG(SANY_LOG_INFO, "Vehicle type is %s\n",  vehicle_type.c_str());
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
	// 车宽
	vehicle_width = config.ReadInt("Vehicle", "width", 3980);
	vehicle_length = config.ReadInt("Vehicle", "length", 9260);
	vehicle_height = config.ReadInt("Vehicle", "height", 4350);

	camera_height = config.ReadInt("Camera", "height", 2600);
	camera_pitchDegree = config.ReadInt("Camera", "pitchDegree", 20);
	camera_blindDist = config.ReadInt("Camera", "blindDist", 2500);

    audio_isOpen = config.ReadInt("Audio", "isOpen", 0);
    audio_isSave = config.ReadInt("Audio", "isSave", 0);
	MINE_LOG(SANY_LOG_INFO, "Vehicle width (%d)\n", vehicle_width);
	MINE_LOG(SANY_LOG_INFO, "Vehicle length (%d)\n",  vehicle_length);
	MINE_LOG(SANY_LOG_INFO, "Vehicle height (%d)\n",  vehicle_height);
	MINE_LOG(SANY_LOG_INFO, "Camera height (%d)\n",  camera_height);
	MINE_LOG(SANY_LOG_INFO, "Camera blindDist (%d)\n",  camera_blindDist);
	MINE_LOG(SANY_LOG_INFO, "Camera pitchDegree (%d)\n",  camera_pitchDegree);
	MINE_LOG(SANY_LOG_INFO, "Audio isOpen (%d)\n",  audio_isOpen);
    MINE_LOG(SANY_LOG_INFO, "Audio isSave (%d)\n",  audio_isSave);
}
/*读取车道线配置文件 2025.03.24 end */
