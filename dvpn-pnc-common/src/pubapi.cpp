#include "pubapi.h"
#include <float.h>

// vector<float> config_unload_point_lat;
// vector<float> config_unload_point_lon;
// vector<float> config_unload_point_head;

PubApi::PubApi() : CSingleton<PubApi>()
{

    /*read config.ini file*/
    memset(ini_value, 0, sizeof(ini_value));
    ini_fp = NULL;
    /*read config.ini file*/

    //    Geocentric earth(Constants::WGS84_a(), Constants::WGS84_f());
}

PubApi::~PubApi()
{
    PubApi::logfilename = nullptr;
    PubApi::logfilename_control = nullptr;
}
/*read config.ini file*/
INI_RESULT PubApi::readIniFile(const char *path_name)
{
    string ini_line, last_main_key;
    char str_line[INI_LINE_LEN] = {0};
    INI_KEYMAP last_map;
    string::size_type index_pos = -1;
    string::size_type left_pos = -1;
    string::size_type right_pos = -1;
    ini_fp = fopen(path_name, "rb");
    if (ini_fp == NULL)
        return INI_OPENFILE_ERROR;
    else
    {
        ini_map.clear();

        while (fgets(str_line, INI_LINE_LEN, ini_fp))
        {
            if (strlen(str_line) == 0) // 跳过空行！！必须加
            {
                last_main_key = ini_line;
                continue;
            }
            ini_line.assign(str_line);
            // 删除字符串中的非必要字符
            left_pos = ini_line.find("\n");
            if (string::npos != left_pos)
                ini_line.erase(left_pos, 1);
            left_pos = ini_line.find("\r");
            if (string::npos != left_pos)
                ini_line.erase(left_pos, 1);
            // 清除空格
            while (string::npos != (left_pos = ini_line.find(" ")))
                ini_line = ini_line.replace(left_pos, left_pos + 1, "");
            // 判断是否是主键
            left_pos = ini_line.find("[");
            right_pos = ini_line.find("]");
            // find 失败返回string::npos
            if (left_pos != string::npos && right_pos != string::npos)
            {
                ini_line.erase(left_pos, 1);
                right_pos--;
                ini_line.erase(right_pos, 1);
                // 跳过第一次没有子键的情况
                if (last_map.empty())
                {
                    last_main_key = ini_line;
                    continue;
                }
                ini_map[last_main_key] = last_map;
                last_main_key = ini_line;
                last_map.clear();
            }
            else
            {
                // 是否是子键
                index_pos = ini_line.find("=");
                if (index_pos != string::npos)
                {
                    string sub_key, sub_value;
                    sub_key = ini_line.substr(0, index_pos);
                    sub_value = ini_line.substr(index_pos + 1, ini_line.length() - index_pos - 1);
                    last_map[sub_key] = sub_value;
                }
                else
                {
                    continue;
                    // 不符合ini键值模板的内容 如注释等
                }
            }
        }
        ini_map[last_main_key] = last_map;
        fclose(ini_fp);
        ini_fp = NULL;
        writeIniLog(ini_map);
        return INI_SUCCESS;
    }
}

INI_RESULT PubApi::writeIniLog(INI_MAINKEYMAP &ini_map)
{
    FILE *ini_logfp;
    ini_logfp = fopen("../log/inilog.txt", "ab");
    const char *str1;
    char str[INI_LINE_LEN];
    if (ini_logfp == NULL)
        return INI_OPENFILE_ERROR;
    else
    {
        // memset(str,0,sizeof(str));
        str1 = "INI文件主键数量 ==> ";
        fputs(str1, ini_logfp);
        sprintf(str, "%d", (int)ini_map.size());
        fputs(str, ini_logfp);
        fputc('\n', ini_logfp);
        for (auto i = ini_map.begin(); i != ini_map.end(); i++)
        {

            fputc('[', ini_logfp);
            fputs(i->first.c_str(), ini_logfp);
            fputc(']', ini_logfp);
            fputc('\n', ini_logfp);
            str1 = "INI文件子键数量 ==> ";
            fputs(str1, ini_logfp);
            sprintf(str, "%d", (int)i->second.size());
            fputs(str, ini_logfp);
            fputc('\n', ini_logfp);
            for (auto j = i->second.begin(); j != i->second.end(); j++)
            {
                fputs(j->first.c_str(), ini_logfp);
                fputc('=', ini_logfp);
                fputs(j->second.c_str(), ini_logfp);
                fputc('\n', ini_logfp);
            }
            fputc('\n', ini_logfp);
        }
    }
    fclose(ini_logfp);
    return INI_SUCCESS;
}
INI_RESULT PubApi::getIniKey(const char *main_key, const char *key, char *value)
{

    INI_MAINKEYMAP::iterator it_main = ini_map.find(main_key);
    if (it_main == ini_map.end())
    {
        // 没找到主键
        cout << "标定文件缺失   " << main_key << endl;
        return INI_NO_ATTR;
    }
    else
    {
        // 找到主键
        INI_KEYMAP mkey = ini_map[main_key];
        INI_KEYMAP::iterator it_child = mkey.find(key);
        if (it_child == mkey.end())
        {
            // 没找到子键
            cout << "标定文件缺失   " << key << endl;
            return INI_NO_ATTR;
        }
        else
        {
            // 找到子键
            string temp = mkey[key];
            strcpy(value, temp.c_str());
            return INI_SUCCESS;
        }
    }
}
float PubApi::getIniNum(const char *main_key, const char *key)
{
    float temp = 0;
    memset(ini_value, 0, sizeof(ini_value));
    if (INI_SUCCESS == getIniKey(main_key, key, ini_value))
        temp = atof(ini_value);
    else
        temp = ERROR_NUMBER;
    return temp;
}

int PubApi::getIniNum(const char *main_key, const char *key, int org_val)
{
    float temp = 0;
    memset(ini_value, 0, sizeof(ini_value));
    if (INI_SUCCESS == getIniKey(main_key, key, ini_value))
        temp = atof(ini_value);
    else
        temp = org_val;
    return temp;
}

float PubApi::getIniNum(const char *main_key, const char *key, float org_val)
{
    float temp = 0;
    memset(ini_value, 0, sizeof(ini_value));
    if (INI_SUCCESS == getIniKey(main_key, key, ini_value))
        temp = atof(ini_value);
    else
        temp = org_val;
    return temp;
}

double PubApi::getIniNum(const char *main_key, const char *key, double org_val)
{
    double temp = 0;
    memset(ini_value, 0, sizeof(ini_value));
    if (INI_SUCCESS == getIniKey(main_key, key, ini_value))
        temp = atof(ini_value);
    else
        temp = org_val;
    return temp;
}

string PubApi::getIniStr(const char *main_key, const char *key)
{
    //    volatile
    string temp;
    memset(ini_value, 0, sizeof(ini_value));
    if (INI_SUCCESS != getIniKey(main_key, key, ini_value))
    {
        strcpy(ini_value, "NULL");
    }
    temp = ini_value;
    return temp;
}

float PubApi::getDistance(double lat1, double lng1, double lat2, double lng2)
{
    double disn, _x, _y;
    _x = fabs(lng1 - lng2) * 111700 * cos(lat1 / 180 * M_PI);
    _y = fabs(lat1 - lat2) * 111700;

    disn = sqrt(_x * _x + _y * _y);
    return disn;
}

float PubApi::calXyDis(float x1, float y1, float x2, float y2)
{
    float dis_x, dis_y, dis;
    dis_x = fabs(x1 - x2);
    dis_y = fabs(y1 - y2);
    dis = sqrt(dis_x * dis_x + dis_y * dis_y);
    return dis;
}

/**
 * @brief PubApi::calVerticalDis
 * 计算（des_x,des_y)到（line_x1,line_y1),（line_x2,line_y2)连线的垂线距离
 * 两点（x1,x2）、（y1,y2）确定的直线方程为Ax+By+C=0,其中A=y2-y1;B=x1-x2;C=x2*y1-x1*y2
 * 则某一点(x0,y0)到直线的距离公式为d=|A*x0+B*y0+C|/sqrt(A*A+B*B)
 */
float PubApi::calVerticalDis(float des_x, float des_y, float line_x1, float line_y1, float line_x2, float line_y2)
{
    float line_a, line_b, line_c, dis;

    line_a = line_y2 - line_y1;
    line_b = line_x1 - line_x2;
    line_c = line_x2 * line_y1 - line_x1 * line_y2;

    if (fabs(line_a) < 0.0001 && fabs(line_b) < 0.0001)                                            // line_x1 = line_x2且line_y1 = line_y2
        dis = sqrt((des_x - line_x1) * (des_x - line_x1) + (des_y - line_y1) * (des_y - line_y1)); // 计算目标点和该点直线距离
    else
        dis = fabs(line_a * des_x + line_b * des_y + line_c) / sqrt(line_a * line_a + line_b * line_b);

    return dis;
}

// 计算目标经纬度点在当前车体坐标系下的x，y坐标
void PubApi::transGpsIntoInertia(GPSInfo &my_gps, GPSInfo &target_point,
                                 double *inertia_x, double *inertia_y)
{

    //float dis = getDistance(my_gps.iLatitude(), my_gps.iLongitude(), target_point.iLatitude(), target_point.iLongitude());          // 两点之间绝对位置
    float azimuth = getAngleByTwoGps(my_gps.iLatitude(), my_gps.iLongitude(), target_point.iLatitude(), target_point.iLongitude()); // 由车辆的gps点到目标点的航向,正北为0度,0~360顺时针
    float d_angle = azimuth - my_gps.iHead();
    if (d_angle >= 180)
        d_angle -= 360;
    if (d_angle < -180)
        d_angle += 360;
    //    *inertia_x = dis*sin(d_angle*M_PI/180);
    //    *inertia_y = dis*cos(d_angle*M_PI/180);

    double z;
    geo_translation.Reset(my_gps.iLatitude(), my_gps.iLongitude(), my_gps.altitude());
    geo_translation.Forward(target_point.iLatitude(), target_point.iLongitude(), target_point.altitude(), *inertia_x, *inertia_y, z);
}

// 以正北为y轴正方向，my_gps为坐标原点，计算target_point的坐标
void PubApi::transGpsIntoGlobalInertia(GPSInfo &my_gps, GPSInfo &target_point, double *inertia_x, double *inertia_y)
{
    //    *inertia_x = (target_point.iLongitude()-my_gps.iLongitude())*111700*cos(my_gps.iLatitude()*M_PI/180);
    //    *inertia_y = (target_point.iLatitude()-my_gps.iLatitude())*111700;

    double z;
    geo_translation.Reset(my_gps.iLatitude(), my_gps.iLongitude(), my_gps.altitude());
    geo_translation.Forward(target_point.iLatitude(), target_point.iLongitude(), target_point.altitude(), *inertia_x, *inertia_y, z);
}

void PubApi::transGpsIntoInertiaPerception(GPSInfo &my_gps, GPSInfo &target_point,
                                         double *inertia_x, double *inertia_y)
{

    float dis = getDistance(my_gps.iLatitude(),my_gps.iLongitude(),target_point.iLatitude(),target_point.iLongitude());//两点之间绝对位置
    float azimuth = getAngleByTwoGps(my_gps.iLatitude(),my_gps.iLongitude(),target_point.iLatitude(),target_point.iLongitude());//由车辆的gps点到目标点的航向,正北为0度,0~360顺时针
    float d_angle = azimuth - my_gps.iHead();
    if(d_angle >=180)
        d_angle -=360;
    if(d_angle <-180)
        d_angle +=360;
   *inertia_x = dis*sin(d_angle*M_PI/180);
   *inertia_y = dis*cos(d_angle*M_PI/180);

    // double z;
    // geo_translation.Reset(my_gps.iLatitude(),my_gps.iLongitude(),my_gps.altitude());
    // geo_translation.Forward(target_point.iLatitude(),target_point.iLongitude(),target_point.altitude(),*inertia_x,*inertia_y,z);

}

float PubApi::getAngleByTwoGps(double lat1, double lng1, double lat2, double lng2)
{
    float result_angle;
    float dx = (lng2 - lng1) * 111700 * cos(lat1 / 180 * M_PI);
    float dy = (lat2 - lat1) * 111700;

    result_angle = atan2(dx, dy) / M_PI * 180; // 计算与y轴正方向的夹角(-180~180)
    result_angle = (result_angle < 0) ? 360 + result_angle : result_angle;

    return result_angle;
}

float PubApi::getAngleByTwoXy(float x1, float y1, float x2, float y2)
{
    float result_angle;
    float dx = x2 - x1;
    float dy = y2 - y1;

    result_angle = atan2(dx, dy) / M_PI * 180; // 计算与y轴正方向的夹角(-180~180)
    result_angle = (result_angle < 0) ? 360 + result_angle : result_angle;

    return result_angle;
}

GPSPos PubApi::calculateGpsByAngle(double longitude, double latitude, float head, float distance)
{
    GPSPos gi;   // 计算后的坐标点
    float slope; // 直线的斜率
    float dx = 0, dy = 0;

    if (head >= 2.0 && head <= 178.0)
    {
        slope = tan((90 - head) / 180 * 3.141592);
        dx = -1 * distance / sqrt(1 + slope * slope);
        dy = dx * slope;
    }
    else if (head >= 182.0 && head <= 358.0)
    {
        slope = tan((90 - head) / 180 * 3.141592);
        dx = distance / sqrt(1 + slope * slope);
        dy = dx * slope;
    }
    else if (head > 358.0 || head < 2.0)
    {
        dx = 0;
        dy = -1 * distance;
    }
    else if (head > 178.0 && head < 182.0)
    {
        dx = 0;
        dy = distance;
    }

    gi.iLongitude = dx / (111700 * cos(latitude / 180 * 3.141592)) + longitude;
    gi.iLatitude = dy / 111700 + latitude;

    return gi;
}

float PubApi::getXDistanceBetweenPoses(VehiclePos currentpos, VehiclePos comparedpos)
{
    float dis_Hypotenuse;
    dis_Hypotenuse = getDistance(comparedpos.lat(), comparedpos.lng(), currentpos.lat(), currentpos.lng());
    float angle_diff;
    angle_diff = getAngleByTwoGps(comparedpos.lat(), comparedpos.lng(), currentpos.lat(), currentpos.lng()) - comparedpos.head();
    float ver_distance = dis_Hypotenuse * sin(angle_diff * M_PI / 180);
    return ver_distance;
}

float PubApi::getYDistanceBetweenPoses(VehiclePos currentpos, VehiclePos comparedpos)
{
    float dis_Hypotenuse;
    dis_Hypotenuse = getDistance(comparedpos.lat(), comparedpos.lng(), currentpos.lat(), currentpos.lng());
    float angle_diff;
    angle_diff = getAngleByTwoGps(comparedpos.lat(), comparedpos.lng(), currentpos.lat(), currentpos.lng()) - comparedpos.head();
    float hor_distance = dis_Hypotenuse * cos(angle_diff * M_PI / 180);
    return hor_distance;
}

char *PubApi::getVIN(const char *main_key, const char *key)
{
    static char temp[] = "SKT90I20210330001";
    memset(ini_value, 0, sizeof(ini_value));
    if (INI_SUCCESS == getIniKey(main_key, key, ini_value))
        memcpy(temp, ini_value, 17);
    else
        memcpy(temp, "NULL", 4);
    return temp;
}

/**
 * @brief DecideLayer::findKeyPoint
 * 查找整条路经上的关键点
 */

inline float PubApi::cal_value(GPSInfo p1, GPSInfo p2)
{
    float dis = getDistance(p1.iLatitude(), p1.iLongitude(), p2.iLatitude(), p2.iLongitude());
    float delt_head = p1.iHead() - p2.iHead();
    if (delt_head > 180)
        delt_head -= 360;
    else if (delt_head < -180)
    {
        delt_head += 360;
    }

    min_pos_angle_weight = 0.2;
    if (fabs(delt_head) > 90)
        min_pos_angle_weight *= fabs(delt_head) / 90;

    //float total_cost = dis + min_pos_angle_weight * fabs(delt_head);

    return dis;
}

int PubApi::findKeyPoint(vector<GPSInfo> road_list, double iLongitude, double iLatitude, double iHead, int key_last, KEY_SEARCH_METHOD search_method)
{

    // 首次查询离最近的点下标
    int road_point_size = road_list.size();
    GPSInfo gii;
    float dis;
    float min = FLT_MAX;

    int key = key_last;
    switch (search_method)
    {
    case ERGODIC:
        for (int j = 0; j < road_point_size; j++)
        {
            gii = road_list.at(j);

            //            printf("iLongitude=%f iLatitude=%f  gii.iLongitude()=%f  gii.iLatitude()=%f\n",iLongitude,iLatitude,gii.iLongitude(),gii.iLatitude());
            dis = getDistance(gii.iLatitude(), gii.iLongitude(), iLatitude, iLongitude);
            if (min > dis)
            {
                min = dis;
                key = j;
            }
        }
        break;
    case CURRENT_ERGODIC:
        for (int j = key_last; j < road_point_size; j++)
        {
            gii = road_list.at(j);
            dis = getDistance(gii.iLatitude(), gii.iLongitude(), iLatitude, iLongitude);
            if (min > dis)
            {
                min = dis;
                key = j;
            }
        }
        break;
    case CIRCLE_PARTIAL:
        for (int j = key_last; j < (key_last + 70); j++)
        {
            gii = road_list.at(j % road_point_size);
            dis = getDistance(gii.iLatitude(), gii.iLongitude(), iLatitude, iLongitude);

            //            printf("pose is :%f,%f  %f,%f\n",gii.iLatitude,gii.iLongitude,iLatitude,iLongitude);

            if (min > dis)
            {
                min = dis;
                key = j % road_point_size;
            }
        }
        break;
    case NONCIRCLE_PARTIAL:
        for (int j = key_last; j < (key_last + 70); j++)
        {
            if (j >= road_point_size)
                break;
            gii = road_list.at(j % road_point_size);
            dis = getDistance(gii.iLatitude(), gii.iLongitude(), iLatitude, iLongitude);

            if (min > dis)
            {
                min = dis;
                key = j;
            }
        }
        break;
    }

    //    printf("key=%d  min=%f  min_pos_dis_threshold=%f  min_pos_angle_weight=%f\n",key,min,min_pos_dis_threshold,min_pos_angle_weight);
    //    if(min>min_pos_dis_threshold)
    //        return -1;

    //    printf("key=%d  dis=%f\n",key,dis);

    return key;
}

void PubApi::initKalmanFilter(KalmanInfo *info, float Q, float R)
{
    info->A = 1;
    info->H = 1;
    info->P = 1;
    info->Q = Q;
    info->R = R;
    info->filterValue = 0;
}
float PubApi::kalmanFilterFun(KalmanInfo *info, float new_value)
{
    float predictValue = info->A * info->filterValue;                                 // 计算预测值
    info->P = info->A * info->A * info->P + info->Q;                                  // 求协方差
    info->kalmanGain = info->P * info->H / (info->P * info->H * info->H + info->R);   // 计算卡尔曼增益
    info->filterValue = predictValue + (new_value - predictValue) * info->kalmanGain; // 计算输出的值
    info->P = (1 - info->kalmanGain * info->H) * info->P;                             // 更新协方差
    return info->filterValue;
}

/**
 * @brief PubApi::checkGps
 * gps单帧校验
 */
bool PubApi::checkGps(const char *gps_ch)
{
    int num = 4;
    // 增加windows文件容错代码 2020.10.19 ZhangR
    if (strchr(gps_ch, '\r') == NULL)
    {
        num = 3;
    }
    int check_bt = 0;
    int length_data = strlen(gps_ch);
    int result = gps_ch[1];
    for (int i = 2; (gps_ch[i] != '*') && (i < length_data); i++)
    {
        result ^= gps_ch[i];
    }
    if (gps_ch[length_data - num] >= '0' && gps_ch[length_data - num] <= '9')
    {
        check_bt = (int)(gps_ch[length_data - num] - '0') * 16;
    }
    else
    {
        check_bt = (int)(gps_ch[length_data - num] - 'A' + 10) * 16;
    }

    if (gps_ch[length_data - num + 1] >= '0' && gps_ch[length_data - num + 1] <= '9')
    {
        check_bt = check_bt + (int)(gps_ch[length_data - num + 1] - '0');
    }
    else
    {
        check_bt = check_bt + (int)(gps_ch[length_data - num + 1] - 'A' + 10);
    }
    if ((check_bt == result) && (gps_ch[0] == '$'))
        return true;
    else
        return false;
}

string PubApi::getFiles(const string path)
{
    //int iFileCnt = 0;
    DIR *dirptr = NULL;
    struct dirent *dirp;
    string file;

    if ((dirptr = opendir(path.c_str())) == NULL) // 打开一个目录
    {
        return 0;
    }

    while ((dirp = readdir(dirptr)) != NULL)
    {
        string a = strchr(dirp->d_name, '.');
#if defined(_WRS_KERNEL)
        if (0 == strcmp(strchr(dirp->d_name, '.'), ".txt")) // 判断文件后缀名
#else
        if ((dirp->d_type == DT_REG) && 0 == (strcmp(strchr(dirp->d_name, '.'), ".txt"))) // 判断是否为文件以及文件后缀名
#endif
        {
            file = dirp->d_name;
            closedir(dirptr);
            return file;
        }
    }

    closedir(dirptr);
    return 0;
}

void PubApi::coutToFileFunc(string str)
{
    freopen("./out.txt", "ab", stdout);
    cout << str << "\n";
    freopen("/dev/tty", "w", stdout);
}

/**
 * @brief PubApi::calDisByKeyPoint
 *计算当前点距离任务点的距离
 */
float PubApi::calDisByKeyPoint(int target_key, int point_key, vector<GPSInfo> *qGpsPathList)
{
    float dis = 0;
    if (point_key < target_key)
    {
        for (int i = point_key; i < target_key; i++)
            dis += qGpsPathList->at(i).iDistance();
    }
    return dis;
}

float PubApi::filterChangeLimit(float new_value, float last_value, float limit)
{
    if (fabs(new_value - last_value) > limit)
        return new_value > last_value ? last_value + limit : last_value - limit;
    else
        return new_value;
}

float PubApi::filterChangeLimit(float new_value, float last_value, float increase_limit, float decrease_limit)
{
    float output;
    if (new_value - last_value > increase_limit)
        output = last_value + increase_limit;
    else if (last_value - new_value > decrease_limit)
        output = last_value - decrease_limit;
    else
        output = new_value;
    return output;
}

void PubApi::clamp(float &raw_value, float low_limit, float high_limit)
{
    raw_value = min(max(raw_value, low_limit), high_limit);
}

void PubApi::clamp(int &raw_value, int low_limit, int high_limit)
{
    raw_value = min(max(raw_value, low_limit), high_limit);
}

/**
 * @brief getVarSection 计算变量所处的区间
 * @param var   需查询的变量值
 * @param var_array  变量数组
 * @param array_size  变量数组长度
 * @return  变量所处区间的右值
 */
int PubApi::getVarSection(float &var, volatile float var_array[], int array_size)
{
    clamp(var, var_array[0], var_array[array_size - 1]);
    int i;
    for (i = 0; i < array_size; i++)
    {
        if (var < var_array[i])
            break;
    }
    clamp(i, 1, array_size - 1);
    return i;
}

/**
 * @brief lookupTable1D  一维查表（线性插值）
 * @param var_value   自变量值
 * @param var_array   自变量数组
 * @param array_size  自变量数组长度
 * @param table_data  被查询的一维数组
 * @return  查表所得的值
 */
float PubApi::lookupTable1D(float var_value, volatile float var_array[], int array_size, volatile float table_data[])
{
    float ratio;
    float interpolation_value;
    int var_index_right;

    var_index_right = getVarSection(var_value, var_array, array_size);
    ratio = (var_array[var_index_right] - var_value) / (var_array[var_index_right] - var_array[var_index_right - 1]);
    interpolation_value = table_data[var_index_right - 1] * ratio + table_data[var_index_right] * (1 - ratio);
    return interpolation_value;
}

/**
 * @brief lookupTable2D  二维查表（线性插值）
 * @param column_value  需查询的列的值
 * @param column_array  列数组
 * @param column_size   列数组长度
 * @param row_value     需查询的行的值
 * @param row_array     行数组
 * @param row_size      行数组长度
 * @param table_data    被查询的二维数组
 * @return   查表所得的值
 */
float PubApi::lookupTable2D(float column_value, volatile float column_array[], int column_size,
                            float row_value, volatile float row_array[], int row_size,
                            volatile float table_data[5][5])
{
    float ratio_column;
    float ratio_row;
    int column_index_right;
    int row_index_right;
    float interpolation_column;
    float interpolation_output;

    column_index_right = getVarSection(column_value, column_array, column_size);
    ratio_column = (column_array[column_index_right] - column_value) / (column_array[column_index_right] - column_array[column_index_right - 1]);

    row_index_right = getVarSection(row_value, row_array, row_size);
    ratio_row = (row_array[row_index_right] - row_value) / (row_array[row_index_right] - row_array[row_index_right - 1]);

    interpolation_column = table_data[row_index_right - 1][column_index_right - 1] * ratio_column + table_data[row_index_right - 1][column_index_right] * (1 - ratio_column);
    interpolation_output = (table_data[row_index_right][column_index_right - 1] * ratio_column + table_data[row_index_right][column_index_right] * (1 - ratio_column)) * (1 - ratio_row) + interpolation_column * ratio_row;
    return interpolation_output;
}

float PubApi::lookupTable2D(float column_value, volatile float column_array[], int column_size,
                            float row_value, volatile float row_array[], int row_size,
                            volatile float table_data[8][7])
{
    float ratio_column;
    float ratio_row;
    int column_index_right;
    int row_index_right;
    float interpolation_column;
    float interpolation_output;

    column_index_right = getVarSection(column_value, column_array, column_size);
    ratio_column = (column_array[column_index_right] - column_value) / (column_array[column_index_right] - column_array[column_index_right - 1]);

    row_index_right = getVarSection(row_value, row_array, row_size);
    ratio_row = (row_array[row_index_right] - row_value) / (row_array[row_index_right] - row_array[row_index_right - 1]);

    interpolation_column = table_data[row_index_right - 1][column_index_right - 1] * ratio_column + table_data[row_index_right - 1][column_index_right] * (1 - ratio_column);
    interpolation_output = (table_data[row_index_right][column_index_right - 1] * ratio_column + table_data[row_index_right][column_index_right] * (1 - ratio_column)) * (1 - ratio_row) + interpolation_column * ratio_row;
    return interpolation_output;
}

float PubApi::lookupTable2D(float column_value, volatile float column_array[], int column_size,
                            float row_value, volatile float row_array[], int row_size,
                            volatile float table_data[10][10])
{
    float ratio_column;
    float ratio_row;
    int column_index_right;
    int row_index_right;
    float interpolation_column;
    float interpolation_output;

    column_index_right = getVarSection(column_value, column_array, column_size);
    ratio_column = (column_array[column_index_right] - column_value) / (column_array[column_index_right] - column_array[column_index_right - 1]);

    row_index_right = getVarSection(row_value, row_array, row_size);
    ratio_row = (row_array[row_index_right] - row_value) / (row_array[row_index_right] - row_array[row_index_right - 1]);

    interpolation_column = table_data[row_index_right - 1][column_index_right - 1] * ratio_column + table_data[row_index_right - 1][column_index_right] * (1 - ratio_column);
    interpolation_output = (table_data[row_index_right][column_index_right - 1] * ratio_column + table_data[row_index_right][column_index_right] * (1 - ratio_column)) * (1 - ratio_row) + interpolation_column * ratio_row;
    return interpolation_output;
}
/*
 * zhangcaming 2021/06/01
 * up_rate 每个周期上升变化量
 * down_rate 每个周期下降变化量
 * last_value 上个周期的值
 */
float PubApi::ramp(float up_rate, float input, float down_rate, float last_value)
{
    float output = 0;

    float diff = 0;
    diff = input - last_value;
    if (diff > up_rate)
        diff = up_rate;
    if (diff < down_rate)
        diff = down_rate;

    output = last_value + diff;
    return output;
}
/*
 * zhangcaming 2021/06/01
 * up_bound 上门限
 * low_bound 下门限
 */
float PubApi::saturation(float up_bound, float input, float low_bound)
{
    float output = input;
    if (input > up_bound)
        output = up_bound;
    if (input < low_bound)
        output = low_bound;
    return output;
}

float PubApi::normRadAngle(float angle)
{
    while (angle < -180)
        angle += 360;

    while (angle >= 180)
        angle -= 360;

    angle *= (M_PI / 180);

    return angle;
}

float PubApi::getDeltAngle(float park_head, float road_head)
{
    float d_angle = road_head - park_head;

    if (d_angle > 180)
        d_angle = d_angle - 360;
    if (d_angle < -180)
        d_angle = d_angle + 360;

    return d_angle;
}

/*initialize config parameter*/
// vehicle
volatile float PubApi::vehicle_mass = 0;
volatile float PubApi::vehicle_length = 0;
volatile float PubApi::vehicle_width = 0;
volatile float PubApi::vehicle_height = 0;
volatile float PubApi::vehicle_wheel_base = 4.55;
volatile float PubApi::vehicle_center_front_ratio = 0;
volatile float PubApi::vehicle_left_max_steer_wheel = 38;
volatile float PubApi::vehicle_right_max_steer_wheel = 29;
volatile float PubApi::vehicle_wheel_radius = 0.65;
volatile float PubApi::vehicle_main_ratio = 12.86;
volatile float PubApi::vehicle_trans_ratio_D = 2.654;
volatile float PubApi::vehicle_trans_ratio_D1 = 0;
volatile float PubApi::vehicle_trans_ratio_D2 = 0;
volatile float PubApi::vehicle_trans_ratio_D3 = 0;
volatile float PubApi::vehicle_trans_ratio_D4 = 0;
volatile float PubApi::vehicle_trans_ratio_D5 = 0;
volatile float PubApi::vehicle_trans_ratio_D6 = 0;
volatile float PubApi::vehicle_trans_ratio_D7 = 0;  
volatile float PubApi::vehicle_trans_ratio_R = 0;
volatile int PubApi::vehicle_type = SKT90EI; // 车辆类型，0,电动小车，1,90i宽体车
volatile char PubApi::vehicle_VIN[18] = "SKT90I20210330001";
volatile float PubApi::front_wheel_base = 2.835;//前轴距
volatile float PubApi::rear_wheel_base = 2.859;//后轴距

// decide
volatile bool PubApi::decide_plat_mode = false;
volatile bool PubApi::decide_avoid_mode = false;
volatile float PubApi::decide_drive_speed_up_bound=10;
volatile float PubApi::decide_drive_speed_down_bound=6;
volatile float PubApi::decide_forward_const_speed=0;
volatile float PubApi::decide_lift_forward_const_speed=5;
volatile float PubApi::decide_back_const_speed=0;
volatile float PubApi::decide_dis_for_ask_pos=0;
volatile float PubApi::decide_dis_for_ask_plan=0;
volatile float PubApi::decide_dis_for_ask_planPre=0;
volatile float PubApi::decide_dis_for_stop=0;
volatile float PubApi::decide_park_speed_up_bound=0;
volatile float PubApi::decide_park_k=0;
volatile float PubApi::decide_park_b=0;
volatile double PubApi::decide_load_pos_lng=0;
volatile double PubApi::decide_load_pos_lat=0;
volatile float PubApi::decide_load_pos_head=0;
volatile double PubApi::decide_dump_pos_lng=0;
volatile double PubApi::decide_dump_pos_lat=0;
volatile float PubApi::decide_dump_pos_head=0;

volatile float dumping_forward_dis = 3;   //
volatile float dumping_back_dis = 3;      //
volatile float loading_forward_dis = 3;   //
volatile float loading_back_dis = 3;      //
volatile float parking_forward_dis = 3;   //
volatile float parking_back_dis = 3;      //
volatile float chargeing_driving_dis = 3; //
volatile float parking_driving_dis = 3;   //
volatile float dumping_forward_lifting_dis=4;
volatile float load_wait_time=10;
volatile float lifting_wait_time=3;
volatile float lifted_wait_time=2;
volatile float continue_angle_diff=2.5;
volatile float continue_dis_diff=30;

//丢失前向感知判断次数
volatile int PubApi::losePercpFrontIndex = 15;
//丢失后向感知判断次数
volatile int PubApi::losePercpBackIndex = 15;
//丢失V2V的判断次数
volatile int PubApi::loseV2VIndex = 75;
//基于V2V交叉路口让行，1为打开，0为关闭
volatile int PubApi::v2v_cross_yield_mode = 1;
//基于V2V交叉路径让行预瞄路径距离
volatile int PubApi::v2vCrossRoadPreDis = 50;
//判断为会车的距离差
volatile float PubApi::mid_side_delt_dis = 0.5;
//判断为会车的角度差
volatile float PubApi::mid_side_delt_head = 10;
//会车的期望车速
volatile float PubApi::keep_mid_desire_spd = 5;
//行驶到主路径释放装料区距离
volatile float PubApi::release_load_area_dis = 30;
//行驶到主路径释放卸料区距离
volatile float PubApi::release_dump_area_dis = 30;
//二次泊车横向偏差
volatile float PubApi::reparking_dis_x = 10;
//二次泊车纵向偏差
volatile float PubApi::reparking_dis_y = 10;
//二次泊车角度差
volatile float PubApi::reparking_dis_head = 10;
//无v2v的挖机通信方式，1为开启，0为关闭
volatile int PubApi::no_v2v_load_mode = 0;
//绕障前的前车静止时间判断
volatile float PubApi::avoid_obst_time = 10;
//举斗前进功能，1为开启，0为关闭
volatile int PubApi::lifting_forward_dis = 1;
//屏蔽装料点驶出部分路段感知,不屏蔽改成-1
volatile float PubApi::shield_out_load_area_dis = 15;
//屏蔽卸料点驶出部分路段感知，不屏蔽改成-1
volatile float PubApi::shield_out_dump_area_dis = 15;
//感知遇障距离、系数5km/h
volatile float PubApi::perception_stop_dis_down_5kmh = 15;
volatile float PubApi::perception_stop_index_5kmh = 4;
//感知遇障距离、系数10km/h
volatile float PubApi::perception_stop_dis_down_10kmh = 25;
volatile float PubApi::perception_stop_index_10kmh = 4;
//感知遇障距离、系数15km/h
volatile float PubApi::perception_stop_dis_down_15kmh = 40;
volatile float PubApi::perception_stop_index_15kmh = 2.5;
//感知遇障距离、系数20km/h
volatile float PubApi::perception_stop_dis_down_20kmh = 45;
volatile float PubApi::perception_stop_index_20kmh = 12;
//感知遇障距离、系数25km/h
volatile float PubApi::perception_stop_dis_down_25kmh = 50;
volatile float PubApi::perception_stop_index_25kmh = 16;
//感知遇障距离、系数30km/h
volatile float PubApi::perception_stop_dis_down_30kmh = 55;
volatile float PubApi::perception_stop_index_30kmh = 20;
//基于感知绕障距离，在遇障停车基础上增加检测距离
volatile float PubApi::perception_avoid_add_dis = 20;
//v2v碰撞检测，障碍车尺寸
volatile float PubApi::v2v_obst_vehicle_length = 9;
volatile float PubApi::v2v_obst_vehicle_width = 4;
//跟车限速在5-15km/h
//基于v2v遇障停车距离、减速系数、跟车距离5km/h
volatile float PubApi::v2v_stop_dis_down_5kmh = 50;
volatile float PubApi::v2v_stop_index_5kmh = 0.2;
volatile float PubApi::v2v_follow_dis_down_5kmh = 25;
//基于v2v遇障停车距离、减速系数、跟车距离10km/h
volatile float PubApi::v2v_stop_dis_down_10kmh = 65;
volatile float PubApi::v2v_stop_index_10kmh = 0.2;
volatile float PubApi::v2v_follow_dis_down_10kmh = 30;
//基于v2v遇障停车距离、减速系数、跟车距离15km/h
volatile float PubApi::v2v_stop_dis_down_15kmh = 65;
volatile float PubApi::v2v_stop_index_15kmh = 0.25;
volatile float PubApi::v2v_follow_dis_down_15kmh = 35;
//基于v2v遇障停车距离、减速系数、跟车距离20km/h
volatile float PubApi::v2v_stop_dis_down_20kmh = 70;
volatile float PubApi::v2v_stop_index_20kmh = 0.25;
volatile float PubApi::v2v_follow_dis_down_20kmh = 40;
//基于v2v遇障停车距离、减速系数、跟车距离25km/h
volatile float PubApi::v2v_stop_dis_down_25kmh = 75;
volatile float PubApi::v2v_stop_index_25kmh = 0.25;
volatile float PubApi::v2v_follow_dis_down_25kmh = 45;
//基于v2v遇障停车距离、减速系数、跟车距离30km/h
volatile float PubApi::v2v_stop_dis_down_30kmh = 80;
volatile float PubApi::v2v_stop_index_30kmh = 0.25;
volatile float PubApi::v2v_follow_dis_down_30kmh = 50;


// obstavoid
volatile float PubApi::obstavoid_slow_k_factor=0;
volatile int PubApi::obstavoid_filter_up_count=0;
volatile float PubApi::obstavoid_filter_dis=0;
volatile float PubApi::obstavoid_brake_bound=0;
volatile float PubApi::obstavoid_brake_back_bound=0;
volatile float PubApi::obstavoid_one_side_width=0;
volatile float PubApi::obstavoid_obst_length=0;
volatile float PubApi::obstavoid_delta_length=0;
volatile float PubApi::obstavoid_default_ttc=0;
volatile float PubApi::obstavoid_default_y=0;
volatile float PubApi::obstavoid_default_x=0;
volatile float PubApi::obst_start_break_coefficient=8;
volatile float PubApi::obstavoid_curve_default_x = 2.5;
volatile float PubApi::obstavoid_curve_default_y = 20;
volatile float PubApi::obstavoid_filter_z = 3.65;
volatile float PubApi::obstavoid_curve_default_angle = 90;

volatile bool PubApi::fifo_flag = false;
volatile bool PubApi::percep_front_flag = true;
volatile bool PubApi::percep_back_flag = true;

volatile float PubApi::boundary_detect_dis = 10;
volatile float PubApi::plan_detect_dis = 7;
volatile float PubApi::back_detect_dis = 15;

volatile float PubApi::abb_vehicle_length=8.967;
volatile float PubApi::abb_vehicle_width=3.468;
volatile float PubApi::left_collision_dis_threshold = 12;//6.43
volatile float PubApi::right_collision_dis_threshold = 12;//6.43
volatile float PubApi::middle_collision_dis_threshold = 10;//6.2

// control
volatile float PubApi::control_spd_kp = 0.3;
volatile float PubApi::control_spd_ki = 0.1;
volatile float PubApi::control_spd_kd = 0.1;
volatile float PubApi::control_spd_integ_up = 0;
volatile float PubApi::control_spd_integ_dn = 0;
volatile float PubApi::control_spd_diff_up = 1;
volatile float PubApi::control_spd_diff_dn = -1;
volatile float PubApi::control_spd_out_up = 3;
volatile float PubApi::control_spd_out_dn = -6;
volatile float PubApi::control_delta_t = 0.05;
volatile float PubApi::control_inherent_acc = 0.1;
volatile float PubApi::control_gravity = 9.81;
volatile float PubApi::control_mass_factor = 1;
volatile float PubApi::control_roll_factor = 0.04;
volatile float PubApi::control_engine_torque_max = 21420;
volatile float PubApi::control_retarder_max_torque = -60000;
volatile float PubApi::control_brake_max_torque = -30000;
volatile float PubApi::control_throttle_empty_stroke = 8;
volatile float PubApi::control_throttle_rate_limit = 2;
volatile float PubApi::control_brake_rate_limit = 40;
volatile float PubApi::control_mechanical_brake_rate_limit = 40;
volatile float PubApi::control_dev_angle_factor = 0.6;
volatile float PubApi::control_dev_heading_factor = 0.45;
volatile float PubApi::control_forward_pre_dis = 7;
volatile float PubApi::control_dev_angle_back_factor = -0.45;
volatile float PubApi::control_dev_heading_back_factor = -0.55;
volatile float PubApi::control_backward_pre_dis = 6;
volatile float PubApi::control_preview_time = 1.2;
volatile float PubApi::control_far_preview_dis = 20;
volatile float PubApi::control_throtle_threshold = 1000;    // 油门介入控制的门限值，高于此值，油门介入控制
volatile float PubApi::control_brake_threshold = -1000;     // 制动介入控制的门限值，低于此值，制动介入控制
volatile float PubApi::control_steer_zero_compensate = 0.5; // 转向0度的补偿值
volatile float PubApi::control_angle_factor_medium_curve = 1.6;
volatile float PubApi::control_angle_factor_large_curve = 1.9;
volatile float PubApi::control_retarder_factor_empty = 0;
volatile float PubApi::control_tram = 0.5;
volatile float PubApi::bucket_speed_full_max = 30;
volatile float PubApi::bucket_speed_full_min = 25;
volatile float PubApi::bucket_speed_empty_max = 25;
volatile float PubApi::bucket_speed_empty_min = 20;

volatile float PubApi::control_velocity_enable_threshold = 2.5;
volatile float PubApi::control_steer_wheel_kp = 1;
volatile float PubApi::control_steer_wheel_ki = 1;
volatile float PubApi::control_steer_wheel_kd = 1;
volatile float PubApi::control_steer_wheel_coefficient = 10;
volatile float PubApi::control_break_mode_threshold = 4;
volatile float PubApi::control_preview_dis_tolerance = 6;
volatile float PubApi::control_preview_dis_tolerance_Park = 8;
volatile float PubApi::control_preview_dis_tolerance_R = 3;
volatile float PubApi::control_curve_detection_preview_dis = 35;
volatile float PubApi::control_time = 3;
volatile float PubApi::control_time_delt = 0.05;
volatile float PubApi::control_preview_dis = 20;
volatile float PubApi::control_steer_delt = 0.5;
volatile float PubApi::control_speed_erroe_coefficient = 1.5;
volatile int PubApi::control_bucket_throttle = 15;
volatile int PubApi::control_shaft_speed_error = 20;
volatile float PubApi::control_up_hill_min = 1.2;
volatile float PubApi::control_up_hill_max = 1.5;
volatile float PubApi::control_down_hill_min = 0.5;
volatile float PubApi::control_down_hill_max = 0.8;
volatile int PubApi::control_bucket_full = 1;
volatile int PubApi::control_bucket_empty_full = 0;
string PubApi::empty_throttle_name="empty_throttle";
string PubApi::empty_break_name="empty_break";
string PubApi::full_throttle_name="full_throttle";
string PubApi::full_break_name="full_break";
//105 ACC PID参数
volatile float PubApi::acc_pid_kp = 0.5;
volatile float PubApi::acc_pid_ki = 0.1;
volatile float PubApi::acc_pid_kd = 0;
volatile float PubApi::empty_brake_approch_speed = 0;
volatile float PubApi::full_brake_approch_speed = 0;
volatile float PubApi::dumping_forward_lifting_dis = 0;


// platform
volatile float PubApi::platform_circle_test=0;
volatile float PubApi::platform_perception_start=0;

volatile float PubApi::lengh_ag=20;
volatile float PubApi::lengh_ab_max=10;
volatile float PubApi::lengh_ab_min=6;
volatile float PubApi::lengh_bc_max=13;
volatile float PubApi::lengh_bc_min=6;
volatile float PubApi::lengh_cd_max=20;
volatile float PubApi::lengh_cd_min=10;
volatile float PubApi::drift_angle_max=45;
volatile float PubApi::drift_angle_min=15;
volatile float PubApi::drift_angle_delt=5;
volatile float PubApi::lengh_delt=1;
volatile float PubApi::arc_raidus_min=13;
volatile float PubApi::arc_radius_max=13;
volatile float PubApi::arc_radius_delt=1;
volatile float PubApi::rho_raidus_min=10;
volatile float PubApi::rho_radius_max=13;
volatile float PubApi::rho_radius_delt=1;
volatile float PubApi::plan_min_radius=13;
volatile float PubApi::plan_forwad_index=3;

string PubApi::logfilename = "/home/nvidia/yfworkspace/logdata/";
string PubApi::logfilename_control = "/home/sany/mnt/log/syzk_plan_ctrl/";

volatile float PubApi::control_empty_throttle_acc_min = -1;
volatile float PubApi::control_empty_throttle_acc_max = 1;
volatile float PubApi::control_empty_throttle_acc_det = 0.02;
volatile float PubApi::control_empty_throttle_speed_min = 0;
volatile float PubApi::control_empty_throttle_speed_max = 25;
volatile float PubApi::control_empty_break_acc_min = -1;
volatile float PubApi::control_empty_break_acc_max = 1;
volatile float PubApi::control_empty_break_acc_det = 0.02;
volatile float PubApi::control_empty_break_speed_min = 0;
volatile float PubApi::control_empty_break_speed_max = 25;
volatile float PubApi::control_full_throttle_acc_min = -1;
volatile float PubApi::control_full_throttle_acc_max = 1;
volatile float PubApi::control_full_throttle_acc_det = 0.02;
volatile float PubApi::control_full_throttle_speed_min = 0;
volatile float PubApi::control_full_throttle_speed_max = 20;
volatile float PubApi::control_full_break_acc_min = -1;
volatile float PubApi::control_full_break_acc_max = 1;
volatile float PubApi::control_full_break_acc_det = 0.02;
volatile float PubApi::control_full_break_speed_min = 0;
volatile float PubApi::control_full_break_speed_max = 20;
volatile float PubApi::empty_max_speed_error = -3;
volatile float PubApi::empty_max_mechanical_brake = 100;
volatile float PubApi::empty_max_brake_pedal = 120;
volatile float PubApi::empty_max_throttle = 100;
volatile float PubApi::empty_up_hill_compensate_coefficient = 2;
volatile float PubApi::empty_down_hill_compensate_coefficient = 2;
volatile float PubApi::empty_flat_throttle_increment = 0.5;
volatile float PubApi::empty_slope_start_time = 2;
volatile float PubApi::empty_slope_start_throttle = 40;
volatile float PubApi::empty_gps_speed_error_coefficient = 0.6;
volatile float PubApi::empty_flat_speed_error_compensate_coefficient = 5;
volatile float PubApi::empty_down_hill_speed_error_compensate_coefficient = 2.5;
volatile float PubApi::empty_break_delt_coefficient = 0.65;
volatile float PubApi::empty_throttle_delt_coefficient = 0.5;
volatile float PubApi::empty_min_break_threshold = 5;
volatile float PubApi::empty_down_hill_speed_error_threshold = 1;
volatile float PubApi::empty_down_hill_max_throttle = 15;
volatile float PubApi::empty_down_hill_break_compensate_coefficient = -10;
volatile float PubApi::empty_break_smooth_coefficient = 0.5;
volatile float PubApi::empty_speed_error_mechanical_threshold = -1;
volatile float PubApi::empty_speed_error_mechanical_coefficient = 10;
volatile float PubApi::empty_pitch_throttle_compensate_coefficient = 3;
volatile float PubApi::empty_max_throttle_coefficient = 1.5;
volatile float PubApi::empty_min_throttle_coefficient = 0.5;
volatile float PubApi::empty_flat_start_min_throttle = 10;
volatile float PubApi::empty_up_hill_start_min_throttle = 40;
volatile float PubApi::empty_down_hill_start_min_throttle = 10;
volatile float PubApi::empty_start_speed_thrsehold_coefficient = 0.7;
volatile float PubApi::empty_slope_throttle_cofficient = 3.5;

volatile float PubApi::full_max_speed_error = -3;
volatile float PubApi::full_max_mechanical_brake = 100;
volatile float PubApi::full_max_brake_pedal = 120;
volatile float PubApi::full_max_throttle = 100;
volatile float PubApi::full_up_hill_compensate_coefficient = 3;
volatile float PubApi::full_down_hill_compensate_coefficient = 2;
volatile float PubApi::full_flat_throttle_increment = 0.5;
volatile float PubApi::full_slope_start_time = 2;
volatile float PubApi::full_slope_start_throttle = 50;
volatile float PubApi::full_gps_speed_error_coefficient = 0.6;
volatile float PubApi::full_flat_speed_error_compensate_coefficient = 15;
volatile float PubApi::full_down_hill_speed_error_compensate_coefficient = 2.5;
volatile float PubApi::full_break_delt_coefficient = 0.8;
volatile float PubApi::full_throttle_delt_coefficient = 0.5;
volatile float PubApi::full_min_break_threshold = 5;
volatile float PubApi::full_down_hill_speed_error_threshold = 1;
volatile float PubApi::full_down_hill_max_throttle = 10;
volatile float PubApi::full_down_hill_break_compensate_coefficient = -5;
volatile float PubApi::full_break_smooth_coefficient = 0.5;
volatile float PubApi::full_speed_error_mechanical_threshold = -1.5;
volatile float PubApi::full_speed_error_mechanical_coefficient = 15;
volatile float PubApi::full_pitch_throttle_compensate_coefficient = 7;
volatile float PubApi::full_max_throttle_coefficient = 1.5;
volatile float PubApi::full_min_throttle_coefficient = 0.5;
volatile float PubApi::full_flat_start_min_throttle = 30;
volatile float PubApi::full_up_hill_start_min_throttle = 50;
volatile float PubApi::full_down_hill_start_min_throttle = 15;
volatile float PubApi::full_start_speed_thrsehold_coefficient = 0.75;
volatile float PubApi::full_slope_throttle_cofficient = 6;
volatile float PubApi::test_desird_speed = 10;
volatile int PubApi::point_key_add = 44;
volatile float PubApi::dump_forword_lift_dis = 1.5;
volatile int PubApi::turning_test_flag = 0;
volatile int PubApi::throttle_increase_test_flag = 0;

volatile float PubApi::min_pos_angle_weight = 0.2;
volatile float PubApi::min_pos_dis_threshold = 50;

volatile float PubApi::lon_control_kp = 0.3;
volatile float PubApi::lon_control_ki = 0.005;
volatile float PubApi::lon_control_kd = 0.025;
volatile float PubApi::lon_control_pid_upper = 100;
volatile float PubApi::lon_control_pid_lower = -100;

volatile float PubApi::lon_control_kp1 = 1;
volatile float PubApi::lon_control_ki1 = 0;
volatile float PubApi::lon_control_kd1 = 0.01;
volatile float PubApi::lon_control_pid_upper1 = 100;
volatile float PubApi::lon_control_pid_lower1 = -100;

volatile float PubApi::lon_acc_control_kp = 0.135;
volatile float PubApi::lon_acc_control_ki = 0;
volatile float PubApi::lon_acc_control_kd = 0.085;
volatile float PubApi::lon_acc_control_pid_upper = 1;
volatile float PubApi::lon_acc_control_pid_lower = -1;
volatile float PubApi::AccTestVehSpeed = 15;
volatile float PubApi::empty_break_coff = 0.33;
volatile float PubApi::full_break_coff = 1.85;

volatile float PubApi::empty_slope_delt_throttle_cofficient = 1.8; //
volatile float PubApi::full_slope_delt_throttle_cofficient = 3.6;  //

volatile float PubApi::empty_slope_break_cofficient = 3.5;    //
volatile float PubApi::empty_slope_delt_break_cofficient = 3; //
volatile float PubApi::full_slope_break_cofficient = 2;       //
volatile float PubApi::full_slope_delt_break_cofficient = 8;  //

volatile float PubApi::enable_retarder_min_dis = 5; //
volatile float PubApi::retarder_gain = 1.1;         //

volatile float PubApi::acc_smooth_gain_accRtr = 0.9;     //
volatile float PubApi::acc_smooth_gain_accBrk = 3.5;     //
volatile float PubApi::acc_smooth_gain_accBrkNeg = 1.1;  //
volatile float PubApi::acc_smooth_gain_accThr = 1.5;     //
volatile float PubApi::acc_follow = 0.5;                 //
volatile float PubApi::acc_smooth_gain_Thr = 0.55;       //
volatile float PubApi::acc_dec_full = 0.35;              //
volatile float PubApi::acc_dec_full_big_slope = 0.085;   //
volatile float PubApi::acc_smooth_gain_Rtr = 0.9;        //
volatile float PubApi::acc_smooth_gain_Brk = 1.05;       //
volatile float PubApi::acc_smooth_gain_BrkFull = 2;      //
volatile float PubApi::acc_smooth_gain_BrkFullDec = 3.5; //

volatile float PubApi::min_control_threshold_thr = 0.5;          //
volatile float PubApi::min_control_threshold_thr_down = 4.5;     //
volatile float PubApi::min_control_threshold_thr_down_full = 10; //
volatile float PubApi::min_control_threshold_brk = 6;            //

volatile float PubApi::lon_control_gps_speed_gain = 1.55;     //
volatile float PubApi::lon_control_shaft_speed_gain = 0.2;    //
volatile float PubApi::lon_control_vehicle_speed_gain = 1.25; //

volatile float PubApi::dec_gain = 1;
volatile float PubApi::lon_control_stop_dec_time = 1.6;

volatile float PubApi::lat_control_preview_dis = 0;
volatile float PubApi::full_slope_break_cofficient_1 = 2.2;
volatile float PubApi::full_slope_break_cofficient_2 = 2.5;

volatile float PubApi::lat_control_kp = 0.6;
volatile float PubApi::lat_control_ki = 0.1;
volatile float PubApi::lat_control_kd = 0.1;

volatile float PubApi::lat_control_pid_upper = 1.5;
volatile float PubApi::lat_control_pid_lower = -1.5;
volatile float PubApi::lat_control_pursuit_weight = 0.7;
volatile float PubApi::lat_control_stanley_weight = 0.3;
volatile float PubApi::lat_control_dwa_weight = 0;
volatile float PubApi::lat_control_dev_angle_factor = 0;
volatile float PubApi::lat_control_dev_heading_factor = 0;
volatile float PubApi::lat_control_dev_angle_back_factor = 0;
volatile float PubApi::lat_control_dev_heading_back_factor = 0;
volatile float PubApi::lat_control_angle_factor_medium_curve = 0;
volatile float PubApi::lat_control_angle_factor_large_curve = 0;
volatile float PubApi::lat_control_time = 0;
volatile float PubApi::lat_control_time_delt = 0;
volatile float PubApi::lat_control_steer_delt = 0;
volatile float PubApi::lat_stanley_gain = 0;
volatile float PubApi::lat_min_pos_angle_weight = 0;
volatile float PubApi::lat_steer_ratio_max = 0;
volatile float PubApi::lat_lqr_gain = 0.3;
volatile float PubApi::lat_puresuit_gain = 0.7;
volatile float PubApi::lat_lqr_gain_R = 0.3;
volatile float PubApi::lat_puresuit_gain_R = 0.7;

volatile float PubApi::lon_control_dec_gain = 0;
volatile float PubApi::lon_control_slope_start_time = 0;
volatile float PubApi::lon_control_slope_start_throttle = 0;
volatile float PubApi::lon_control_min_control_threshold = 0;
volatile float PubApi::lon_control_acc_smooth_gain = 0;
volatile float PubApi::lon_control_max_brake_pedal = 0;
volatile float PubApi::lon_control_max_mechanical_brake = 0;
volatile float PubApi::lon_control_max_throttle = 0;
volatile float PubApi::lon_control_enable_retarder_min_dis = 5;
volatile float PubApi::lon_control_empty_slope_throttle_cofficient = 3.75;
volatile float PubApi::lon_control_empty_slope_delt_throttle_cofficient = 5.5;
volatile float PubApi::lon_control_full_slope_throttle_cofficient = 4.5;
volatile float PubApi::lon_control_full_slope_delt_throttle_cofficient = 11;
volatile float PubApi::lon_control_retarder_gain = 0;
volatile float PubApi::lon_control_empty_slope_break_cofficient = 0.775;
volatile float PubApi::lon_control_full_slope_break_cofficient = 0;
volatile float PubApi::acc_smooth_gain_Thr_bigSlope = 0.3;
volatile float PubApi::acc_smooth_Full_Load_BrkFullDec = 1;

volatile int PubApi::lose_decide_count = 20;
volatile int PubApi::lose_297_count = 20;

volatile float PubApi::Q1 = 0.15;
volatile float PubApi::Q2 = 0.6;
volatile float PubApi::Q3 = 0.2;
volatile float PubApi::Q4 = 0.35;
volatile float PubApi::R1 = 0.5;

volatile int PubApi::LQR_Loop = 200;
volatile double PubApi::LQR_Min_Loop_val = 0.01;

volatile float PubApi::mass_front = 10496;
volatile float PubApi::mass_rear = 21504;
volatile float PubApi::mass_front_full = 18400;
volatile float PubApi::mass_rear_full = 73600;
volatile float PubApi::cf_cr = 436754.2681;

volatile double PubApi::ed_limit_min = 0.4;
volatile double PubApi::ed_limit_max = 0.8;
volatile double PubApi::ephi_limit_min = 0.3;
volatile double PubApi::ephi_limit_max = 1;

volatile float PubApi::Kb = 0;
volatile double PubApi::Cd = 0;
volatile double PubApi::Veh_A = 0;
volatile double PubApi::air_density = 0;
volatile double PubApi::f0 = 0;
volatile double PubApi::f1 = 0;
volatile double PubApi::motor_effi = 0;
volatile double PubApi::accTorCtrlThr = 0.1;
volatile int PubApi::spd_empty_max = 30;
volatile int PubApi::spd_full_max = 20;
volatile int PubApi::lqr_on = 1;

