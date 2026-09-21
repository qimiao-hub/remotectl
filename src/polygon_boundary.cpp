#include "../include/polygon_boundary.h"

// namespace fs = std::filesystem;

static constexpr double WGS84_R = 6378137.0; // WGS84 地球半径(m)
static constexpr double PI = 3.14159265358979323846;

static inline double deg2rad(double deg) {
    return deg * PI / 180.0;
}

// 航向角转方向向量（北为0度，顺时针为正）
static void headingToVector(double heading_deg, double& dx, double& dy) {
    double yaw = deg2rad(heading_deg);
    dx = sin(yaw);  // 北=0 时用 sin/cos
    dy = cos(yaw);
}

Polygon_Boundary::Polygon_Boundary() {
    m_mainTaskState = -1;
    // loadBoundary();
    // pDataTaskState = new TaskBehaviorStatePubSubType();
    // pSubTaskState = new ddsSubscriber<TaskBehaviorState>(pDataTaskState, std::bind(&Polygon_Boundary::taskStateCallback, this, std::placeholders::_1));
}

// /*状态机回调*/
// SANY_VOID Polygon_Boundary::taskStateCallback(TaskBehaviorState &data){
//     // if(1 == g_u8TaskStartFlag)
// 	// {
// 		m_mainTaskState = data.mainState;
// 	// }
// 	return;
// }



/* ================== 坐标转换 ================== */
void Polygon_Boundary::wgs84ToXY(double lon, double lat, double refLon, double refLat, double& x, double& y) {
    double latRad = deg2rad(refLat);
    x = WGS84_R * cos(latRad) * deg2rad(lon - refLon);
    y = WGS84_R * deg2rad(lat - refLat);
}

/* ================== 点在多边形内 ================== */
bool Polygon_Boundary::pointInPolygon(const CoordinatePoint& p, const std::vector<CoordinatePoint>& poly) {
    bool inside = false;
    size_t n = poly.size();

    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        const auto& a = poly[i];
        const auto& b = poly[j];

        if (((a.y > p.y) != (b.y > p.y)) &&
            (p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x)) {
            inside = !inside;
        }
    }
    return inside;
}

/* ================== 点在边界内，转换坐标 ================== */
bool Polygon_Boundary::pointInBoundary(double lng, double lat, vector<GPSPoint>& boundaryList){
    if (boundaryList.size() < 3) {
        MINE_LOG(SANY_LOG_ERROR, "boundaryList.size = %ld\n", boundaryList.size());
        return false;
    }

    //转换起点和终点对应的xy坐标系 
    double start_x, start_y;
    wgs84ToXY(lng, lat, lng, lat, start_x, start_y); // 求a点的xy坐标，求出来一定是(0, 0)

    // 转换边界经纬度到xy坐标系
    // 将边界y转为坐标，参考点必须一致
    
    std::vector<CoordinatePoint> boundry_xys;
    for (size_t i = 0; i < boundaryList.size(); i++){
        double temp_x, temp_y;
        // wgs84ToXY(lng, lat, boundaryList[i].iLongitude, boundaryList[i].iLatitude, temp_x ,temp_y);
        wgs84ToXY( boundaryList[i].iLongitude, boundaryList[i].iLatitude, lng, lat, temp_x ,temp_y);
        boundry_xys.push_back(CoordinatePoint{temp_x, temp_y});
    }

    // 在xy坐标系判断是否在边界内
    CoordinatePoint start;
    start.x = start_x;
    start.y = start_y;
    if (!pointInPolygon(start, boundry_xys)){
        MINE_LOG(SANY_LOG_INFO, "start not in boundary!\n");
        m_pointInDump_xys.clear();
        return false;
    } 
    m_pointInDump_xys.clear();
    // m_pointInDump_xys = boundaryList;
    m_pointInDump_xys = boundry_xys;
    return true;
}


/* ================== 点到线段距离 ================== */
double Polygon_Boundary::pointToSegment(const CoordinatePoint& p, const CoordinatePoint& a, const CoordinatePoint& b) {
    double dx = b.x - a.x;
    double dy = b.y - a.y;

    if (dx == 0.0 && dy == 0.0)
        return hypot(p.x - a.x, p.y - a.y);

    double t = ((p.x - a.x) * dx + (p.y - a.y) * dy) /
               (dx * dx + dy * dy);

    if (t < 0.0) return hypot(p.x - a.x, p.y - a.y);
    if (t > 1.0) return hypot(p.x - b.x, p.y - b.y);

    return hypot(p.x - (a.x + t * dx),
                 p.y - (a.y + t * dy));
}

/* ================== 点到多边形最短距离 ================== */
double Polygon_Boundary::distanceToPolygon(const CoordinatePoint& p,  const std::vector<CoordinatePoint>& poly) {
    double minDist = 1e18;
    size_t n = poly.size();

    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        minDist = std::min(minDist,
                           pointToSegment(p, poly[i], poly[j]));
    }
    return minDist;
}


/* ================== 点到线段最近点及距离 ================== */
double Polygon_Boundary::pointToSegmentWithNearest(const CoordinatePoint& p,
                                        const CoordinatePoint& a,
                                        const CoordinatePoint& b,
                                        CoordinatePoint& nearest) {
    double dx = b.x - a.x;
    double dy = b.y - a.y;

    if (dx == 0.0 && dy == 0.0) {
        nearest = a;
        return hypot(p.x - a.x, p.y - a.y);
    }

    double t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / (dx * dx + dy * dy);

    if (t < 0.0) {
        nearest = a;
    } else if (t > 1.0) {
        nearest = b;
    } else {
        nearest.x = a.x + t * dx;
        nearest.y = a.y + t * dy;
    }

    return hypot(p.x - nearest.x, p.y - nearest.y);
}

double Polygon_Boundary::minDisToRearBoundary(double lng, double lat, double heading_deg, 
            const std::vector<CoordinatePoint>& polygon){
     // 车辆位置转换为局部坐标（原点）
    double x = 0.0, y = 0.0;
    wgs84ToXY(lng, lat, lng, lat, x, y);
    CoordinatePoint vehicle{x, y};

    // 后方方向向量（车尾方向）
    double rearDirX, rearDirY;
    headingToVector(heading_deg + 180.0, rearDirX, rearDirY);

    double minDist = std::numeric_limits<double>::max();
    size_t n = polygon.size();

    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        const auto& A = polygon[i];
        const auto& B = polygon[j];

        CoordinatePoint nearest;
        double dist = pointToSegmentWithNearest(vehicle, A, B, nearest);

        // 从车辆指向最近点的向量
        double vx = nearest.x - vehicle.x;
        double vy = nearest.y - vehicle.y;

        // 点积 > 0 表示最近点位于后方半平面（夹角 < 90°）
        if (vx * rearDirX + vy * rearDirY > 0) {
            if (dist < minDist) {
                minDist = dist;
            }
        }
    }

    return (minDist > 1e17) ? -1.0 : minDist;
}


std::string Polygon_Boundary::execuCommandRet(const char* cmd, int &exitCode) {  
    std::array<char, 4096> buffer;
    std::string result;
     FILE* pipe = popen(cmd, "r");
    if (!pipe) {
        int err = errno;
        MINE_LOG(SANY_LOG_ERROR, "popen() failed, errno=%d (%s)\n", err, strerror(err));
        throw std::runtime_error("popen() failed: " + std::string(strerror(err)));
    }
    // 读取所有输出
    while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
        result += buffer.data();
    }
    // 关闭管道并获取退出状态
    int status = pclose(pipe);
    if (WIFEXITED(status)) {
        exitCode = WEXITSTATUS(status);
    } else {
        exitCode = -1;  // 非正常退出（如信号终止）
    }


    MINE_LOG(SANY_LOG_INFO, "execute cmd : %s!\n", cmd);
    MINE_LOG(SANY_LOG_INFO, "execute result : %s!\n", result.c_str());
    MINE_LOG(SANY_LOG_INFO, "execute exitCode : %d!\n", exitCode);
    
    return result;  
}  

#if 0
// std::string Polygon_Boundary::execuCommandRet(const char* cmd, int &exitCode) {
//     // 临时文件名（可用随机数或进程ID避免冲突）
//     std::string tmpFile = "/tmp/cmd_output_" + std::to_string(getpid()) + "_" + std::to_string(rand());
    
//     // 构建带输出重定向的命令
//     // std::string fullCmd = std::string(cmd) + " > " + tmpFile + " 2>&1";
    
//     MINE_LOG(SANY_LOG_INFO, "executing cmd: %s\n", cmd);
    
//     // 执行命令
//     int ret = system(cmd);
//     if (WIFEXITED(ret)) {
//         exitCode = WEXITSTATUS(ret);
//     } else {
//         exitCode = -1;
//     }
    
//     // 读取临时文件内容
//     std::string result;
//     std::ifstream ifs(tmpFile);
//     if (ifs.is_open()) {
//         result.assign((std::istreambuf_iterator<char>(ifs)),
//                        std::istreambuf_iterator<char>());
//         ifs.close();
//     }
    
//     // 删除临时文件
//     std::remove(tmpFile.c_str());
    
//     MINE_LOG(SANY_LOG_INFO, "execute cmd : %s\n", cmd);
//     MINE_LOG(SANY_LOG_INFO, "execute result : %s\n", result.c_str());
//     MINE_LOG(SANY_LOG_INFO, "execute exitCode : %d\n", exitCode);
    
//     return result;
// }
#endif

bool Polygon_Boundary::scpMapFromOrinToZXK(){
    int exitCode = 0;
    std::string check_sshpass = "ls -l ./sshpass";
    std::string check_result;
     try {
        check_result = execuCommandRet(check_sshpass.c_str(), exitCode);
    } catch (const std::exception& e) {
        MINE_LOG(SANY_LOG_ERROR, "execuCommandRet exception for check: %s\n", e.what());
        return false;
    }
    if (check_result.empty()) {
        MINE_LOG(SANY_LOG_ERROR, "sshpass executable not found or not accessible !!!\n");
        return false;
    }

    // 判断netcloud在哪个核
    std::string sshpass_path = std::string("/data/sany/") + m_Config->current_part + "/project/application/bin/sshpass";
    std::string ps_string = sshpass_path + " -p 'nvidia' ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null nvidia@" + NVIDIA_A_IP +
            " 'ps -ef | grep syzk-netcloud'";
    std::string ps_result;
    try {
        ps_result = execuCommandRet(ps_string.c_str(), exitCode);
    } catch (const std::exception& e) {
        MINE_LOG(SANY_LOG_ERROR, "execuCommandRet exception for check: %s\n", e.what());
        return false;
    }
    if (exitCode != 0) {
        MINE_LOG(SANY_LOG_ERROR, "sshpass executable not found or not accessible !!!\n");
        return false;
    }
    std::string nvidia_id = NVIDIA_A_IP;
    if(ps_result.find("./syzk-netcloud") == std::string::npos){
        nvidia_id = NVIDIA_B_IP;
    }

    // scp文件
    std::string scp_string = sshpass_path + " -p 'nvidia' scp -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null nvidia@" + nvidia_id + ":" + ORIN_HD_OBJECT_POLYGON_PATH + " " + SAVE_HD_OBJECT_POLYGON_PATH;
    MINE_LOG(SANY_LOG_INFO, "scp map to gateway: %s \n", scp_string.c_str());

    
    std::string scp_result;
      try {
        scp_result = execuCommandRet(scp_string.c_str(), exitCode);
    } catch (const std::exception& e) {
        MINE_LOG(SANY_LOG_ERROR, "execuCommandRet exception for scp: %s\n", e.what());
        return false;
    }
    MINE_LOG(SANY_LOG_INFO, "scp map to gateway, exitCode (%d), result (%s) !!!\n", exitCode, scp_result.c_str());

    if (exitCode != 0) {
        MINE_LOG(SANY_LOG_ERROR, "scp map to gateway, ERROR !!!\n");
        return false;
    }
    return true;
}

/* ================== 计算两个GPS点球面距离（Haversine） ================== */
double Polygon_Boundary::distanceBetweenGPSPoints(const GPSPoint& a, const GPSPoint& b) {
    double lon1 = deg2rad(a.iLongitude);
    double lat1 = deg2rad(a.iLatitude);
    double lon2 = deg2rad(b.iLongitude);
    double lat2 = deg2rad(b.iLatitude);
    double dlon = lon2 - lon1;
    double dlat = lat2 - lat1;
    double haversine = sin(dlat/2) * sin(dlat/2) +
                       cos(lat1) * cos(lat2) * sin(dlon/2) * sin(dlon/2);
    double c = 2 * atan2(sqrt(haversine), sqrt(1 - haversine));
    return WGS84_R * c;
}

/* ================== 边界点线性插值 ================== */
void Polygon_Boundary::interpolateBoundaryPoints(std::vector<GPSPoint>& pts, double maxIntervalMeters) {
    if (pts.size() < 2) return;

    std::vector<GPSPoint> newPts;
    newPts.reserve(pts.size() * 2);

    for (size_t i = 0; i < pts.size(); ++i) {
        const GPSPoint& curr = pts[i];
        const GPSPoint& next = pts[(i + 1) % pts.size()];

        // 添加当前点
        newPts.push_back(curr);

        // 计算两点间距离
        double dist = distanceBetweenGPSPoints(curr, next);
        if (dist <= maxIntervalMeters) {
            continue;
        }

        // 需要插入的点数（确保相邻点间距 <= maxIntervalMeters）
        int n = static_cast<int>(ceil(dist / maxIntervalMeters));
        double step = 1.0 / n;  // 参数 t 步长

        for (int k = 1; k < n; ++k) {
            double t = k * step;
            GPSPoint interp;
            interp.iLongitude = curr.iLongitude + t * (next.iLongitude - curr.iLongitude);
            interp.iLatitude  = curr.iLatitude  + t * (next.iLatitude  - curr.iLatitude);
            newPts.push_back(interp);
        }
    }

    // 替换原有点集
    pts.swap(newPts);
}

/* ================== 对外接口 ================== */
/* ================== 地图加载 ================== */
bool Polygon_Boundary::loadBoundary() {

    bool ret = scpMapFromOrinToZXK();
    if(ret == false){
        MINE_LOG(SANY_LOG_ERROR, "scp map to ZXK failed\n", ret);
        return false;
    }

   

    // if (!fs::exists(HD_OBJECT_POLYGON_PATH))
    // {
    //     MINE_LOG(SANY_LOG_ERROR, "hd_object_polygon File not exist\n");
    //     return false;
    // }

    MINE_LOG(SANY_LOG_INFO, "start read hd_object_polygon\n");
   

    std::ifstream ifs(HD_OBJECT_POLYGON_PATH);
     if (!ifs.is_open()) {
        MINE_LOG(SANY_LOG_ERROR, "Failed to open file: %s\n", HD_OBJECT_POLYGON_PATH);
        return false;
    }
    //打开文件并判断是否打开成功如果成功则读取数据
    std::string content;
    std::string line;

    while (std::getline(ifs, line)) {
        content += line;
    }
    //关闭文件
    ifs.close();
    try
    {
        auto jsonData = nlohmann::json::parse(content); // 解析 JSON 字符串
        // 解析 crs
        std::string version = "";
        if (jsonData.find("crs") != jsonData.end())
        {
            auto crs = jsonData["crs"];
            if (crs.find("properties") != crs.end())
            {
                auto properties = crs["properties"];
                if (properties.find("version") != properties.end() && properties["version"].is_string())
                {
                    version = properties["version"];
                    MINE_LOG(SANY_LOG_INFO, "the version is :%s\n", version.c_str());
                }
            }
        }

        if (version.empty())
        {
            MINE_LOG(SANY_LOG_INFO, "the version is not exist:%s\n", version.c_str());
            return false;
        }
        {
            MINE_LOG(SANY_LOG_INFO, "load dump polygons lock!\n");
            std::lock_guard<std::mutex> lock(m_mutex); // 家所

            m_dumpPolygons.clear();
            // 解析 features
            if (jsonData.find("features") != jsonData.end() && jsonData["features"].is_array())
            {
                for (const auto &feature : jsonData["features"])
                {
                    AreaInfo props;
                    // 解析properties, 判断area_type
                    if (feature.find("properties") != feature.end())
                    {
                        auto properties = feature["properties"];
                        if (properties.find("area_type") != properties.end() && properties["area_type"].is_string())
                        {
                            // 只保存排土场
                            if(properties["area_type"] != "dump"){
                                continue;
                            }
                        }
                        
                        if (properties.find("obj_id") != properties.end() && properties["obj_id"].is_string())
                        {
                            props.areaId = properties["obj_id"];
                        }
                        if (properties.find("area_name") != properties.end() && properties["area_name"].is_string())
                        {
                            props.areaName = properties["area_name"];
                        }
                        MINE_LOG(SANY_LOG_INFO, "props: %s, %s\n", props.areaId.c_str(), props.areaName.c_str());
                    }
                    // 解析 coordinates
                    if (feature.find("geometry") != feature.end())
                    {
                        auto geometry = feature["geometry"];
                        if (geometry.find("coordinates") != geometry.end() && geometry["coordinates"].is_array())
                        {
                            if (geometry["coordinates"].size() <= 0 || geometry["coordinates"][0].size() <= 0)
                            {
                                MINE_LOG(SANY_LOG_ERROR, "coordinates is null\n");
                                continue;
                            }
                            for (const auto &coord : geometry["coordinates"][0][0])
                            {
                                if (coord.size() != 3)
                                {
                                    MINE_LOG(SANY_LOG_ERROR, "Invalid coordinate format\n");
                                    continue;
                                }
                                GPSPoint point;
                                point.iLongitude = coord[0];
                                point.iLatitude = coord[1];
                                props.pts.push_back(point);
                            }
                        }
                    }
                    if (!props.pts.empty()) {
                        // 插值加密边界点（间隔0.5米）
                        interpolateBoundaryPoints(props.pts, 0.5);
                        m_dumpPolygons.push_back(props);
                    }
                }
                // for (auto k : m_dumpPolygons)
                // {
                //     MINE_LOG(SANY_LOG_INFO, "m_dumpPolygons = %s, %s, %d\n", k.areaId.c_str(), k.areaName.c_str(), k.pts.size());
                // }
                
            }
            MINE_LOG(SANY_LOG_INFO, "load dump polygons end: %zu\n", m_dumpPolygons.size());
        }

    }
    catch (const std::exception &e)
    {
        MINE_LOG(SANY_LOG_ERROR, "the geoson json file is not valid:%s\n", e.what());
        return false;
    }
   
    // 
    return true;
}

std::string Polygon_Boundary::isPointInDump(double lng, double lat) {
    std::lock_guard<std::mutex> lock(m_mutex); // 加锁

    for(auto it: m_dumpPolygons){
        if(pointInBoundary(lng, lat, it.pts)){
            MINE_LOG(SANY_LOG_INFO, "point in dump %s, %s\n", it.areaId.c_str(), it.areaName.c_str());
            return it.areaId;
        }
    }

    return "";
}

double Polygon_Boundary::minDisToDumpBoundary(double lng, double lat, std::string areaId) {
    std::lock_guard<std::mutex> lock(m_mutex);

    double x = 0.0, y = 0.0;
    wgs84ToXY(lng, lat, lng, lat, x, y);
    CoordinatePoint p{x, y};

    double minDist = std::numeric_limits<double>::max();
    minDist = std::min(minDist, distanceToPolygon(p, m_pointInDump_xys));
  
    if (minDist > 1e17)
        return -1.0;

    return minDist; // 米
}

double Polygon_Boundary::minDisToRearDumpBoundary(double lng, double lat, double heading_deg, std::string areaId)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_pointInDump_xys.empty())
        return -1.0;

    return minDisToRearBoundary(lng, lat, heading_deg, m_pointInDump_xys);
}