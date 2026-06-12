#ifndef POLYGON_BOUNDARY_H
#define POLYGON_BOUNDARY_H

#include <vector>
#include <mutex>
#include <string>
#include <cmath>
#include <unistd.h>
#include <array>      // std::array
#include <memory>     // std::unique_ptr
#include <cstdio>     // popen, pclose, fgets (POSIX)
#include <stdexcept>  // std::runtime_error

#include <nlohmann/json.hpp>
#include <fstream>

#include "remote_config.h"
// #include <filesystem>
#include "../dvpn-pnc-common/ddsType/include/gps_msg.h"
#include "../dvpn-pnc-common/ddsType/include/gps_msgPubSubTypes.h"
#include "../dvpn-pnc-common/ddsType/include/decide_state_machine.h"
#include "../dvpn-pnc-common/ddsType/include/decide_state_machinePubSubTypes.h"

#include <zkbs/ddsBase/ddsPubBase.h>
#include <zkbs/ddsBase/ddsSubBase.h>
#include "zkbs/zkbs.hpp"


#define SSHPASS_PATH "./sshpass"
#define ORIN_HD_OBJECT_POLYGON_PATH "/home/sany/config/map/hd_object_polygon.geojson"
#define SAVE_HD_OBJECT_POLYGON_PATH "/data/sany/config/"
#define HD_OBJECT_POLYGON_PATH "/data/sany/config/hd_object_polygon.geojson"
#define NVIDIA_A_IP "192.168.1.102"
#define NVIDIA_B_IP "192.168.1.103"

extern Remote_Config *m_Config;

struct CoordinatePoint { //转换坐标后的x、y
    double x; // 米
    double y; // 米
};

struct GPSPoint{ //经纬度
    double iLongitude;//经度
    double iLatitude;//纬度
};

struct AreaInfo {
    std::string areaId;
    std::string areaName;
    std::vector<GPSPoint> pts;
};

class Polygon_Boundary {
public:
    Polygon_Boundary();
    ~Polygon_Boundary() = default;

    /*  对外接口 */
    bool loadBoundary();
    std::string isPointInDump(double lng, double lat);
    double minDisToDumpBoundary(double lng, double lat, std::string areaId);
   
    double minDisToRearDumpBoundary(double lng, double lat, double heading_deg, std::string areaId);


private:
    SANY_U8 scpMapFromOrinToZXK();

    std::string execuCommandRet(const char* cmd, int &exitCode);
    

    void wgs84ToXY(double lon, double lat,
                   double refLon, double refLat,
                   double& x, double& y);

    bool pointInPolygon(const CoordinatePoint& p,
                        const std::vector<CoordinatePoint>& poly);

    bool pointInBoundary(double lng, double lat, std::vector<GPSPoint>& boundaryList);



    double pointToSegment(const CoordinatePoint& p,
                          const CoordinatePoint& a,
                          const CoordinatePoint& b);

    double distanceToPolygon(const CoordinatePoint& p,
                             const std::vector<CoordinatePoint>& poly);

    double pointToSegmentWithNearest(const CoordinatePoint& p,
                                        const CoordinatePoint& a,
                                        const CoordinatePoint& b,
                                        CoordinatePoint& nearest) ;

    double minDisToRearBoundary(double lng, double lat, double heading_deg, 
                            const std::vector<CoordinatePoint>& polygon);
    /* ================== 计算两个GPS点球面距离（Haversine） ================== */
    double distanceBetweenGPSPoints(const GPSPoint& a, const GPSPoint& b);
    /* ================== 边界点线性插值 ================== */
    void interpolateBoundaryPoints(std::vector<GPSPoint>& pts, double maxIntervalMeters);
 

private:
    std::vector<AreaInfo> m_dumpPolygons;
    std::vector<CoordinatePoint> m_pointInDump_xys;
    std::mutex           m_mutex;

private:
    /* dds */
    /*状态机反馈*/
    uint m_mainTaskState;
    TopicDataType *pDataTaskState = NULL;
    ddsSubscriber<TaskBehaviorState> *pSubTaskState = NULL;

    // /*挡墙边界*/
    // DDS_FusionCurbMessage m_fusionCurbMsg;
    // float m_curbDis;
    // TopicDataType *pDataFusionCurbMsg = NULL;
    // ddsSubscriber<DDS_FusionCurbMessage> *pSubFusionCurbMsg = NULL;
    
private:
     /*状态机回调*/
    SANY_VOID taskStateCallback(TaskBehaviorState &data);

    
};

#endif