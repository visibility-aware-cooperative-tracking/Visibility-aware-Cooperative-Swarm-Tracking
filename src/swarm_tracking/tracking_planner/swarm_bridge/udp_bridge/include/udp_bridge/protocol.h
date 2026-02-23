#ifndef PROTOCOL_H
#define PROTOCOL_H

#include "cstdio"

namespace udp_bridge {

    enum MESSAGE_TYPE {
        UNDEFINED = 100,
        TIME_SYNC,
        IP_ID,
        IMU,
        ODOM,
        QUAD_STATE,
		BAG_TIME,
        TRIGGER_AUTO,
        GLOBAL_EXTRINSIC,
        TARGET_OBSRV,
        MINCO_TRAJ,
        CHUNK_STAMP,
        MAP_LOCAL,
    };

    struct Header {
        int32_t sec;
        int32_t nsec;
    };
///================USER DEFINED DATA STRUCTURE======================================

    struct TimeSyncMsg {
        Header t1, t2, t3, t4;
        uint8_t server_id;
        uint8_t client_id;
    };
    union TimeSyncMsgCvt {
        TimeSyncMsg data;
        uint8_t binary[sizeof(TimeSyncMsg)];
    };
//Send my Ip and drone id
    struct IpIdMsg {
        uint8_t local_ip[4]; 
        uint8_t local_id;
    };
    union IpIdMsgCvt {
        IpIdMsg data;
        uint8_t binary[sizeof(IpIdMsg)];
    };

//Send my Bag Start Time
struct BagTimeMsg {
  Header stamp;
};
union BagTimeCvt {
  BagTimeMsg data;
  uint8_t binary[sizeof(BagTimeMsg)];
};

//Send Trigger
    struct TriggerMsg {
        int32_t trigger2auto;
    };
    union TriggerCvt {
        TriggerMsg data;
        uint8_t binary[sizeof(TriggerMsg)];
    };

///================USER DEFINED DATA STRUCTURE======================================
    struct ImuData{
        float gyro[3];
        float acc[3];
        Header header;
    };
    union ImuDataCvt {
        ImuData data;
        uint8_t binary[sizeof(ImuData)];
    };

///================USER DEFINED DATA STRUCTURE======================================
/// 1) define the typename and struct
#define MAX_UAV_NUM 6
    struct QuadStateTeammate {
        uint8_t teammate_id;
        bool is_observe;
        float observed_pos[3];
    };
    struct QuadState {
        Header header;
        uint8_t drone_id;
        float pos[3];
        float quat_w;
        float quat_x;
        float quat_y;
        float quat_z;
        float gyr[3];
        float vel[3];
        QuadStateTeammate teammate[MAX_UAV_NUM];
    };
    union QuadStateCvt {
        QuadState data;
        uint8_t binary[sizeof(QuadState)];
    };
///================USER DEFINED DATA STRUCTURE======================================
    struct GlobalExtrinsic {
        uint8_t teammate_id;
        float rot_deg[3];
        float trans[3];
    };
    struct GlobalExtrinsicStatus {
        Header header;
        uint8_t drone_id;
        GlobalExtrinsic extrinsic[MAX_UAV_NUM];
    };
    union GlobalExtrinsicStatusCvt {
        GlobalExtrinsicStatus data;
        uint8_t binary[sizeof(GlobalExtrinsicStatus)];
    };

///================USER DEFINED DATA STRUCTURE======================================
    struct TargetObsrv {
        Header header;
        uint8_t drone_id;
        float target_pos[3];
    };

    union TargetObsrvCvt {
        TargetObsrv data;
        uint8_t binary[sizeof(TargetObsrv)];
    };

    
    /// 2) define the static encode and decode function
    static void RosMsgToStruct() {

    }

    static void StructToRosMsg() {

    }


}

#endif