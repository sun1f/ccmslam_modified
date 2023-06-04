// CentralControl类没有源文件
#ifndef CSLAM_CENTRALCONTROL_H_
#define CSLAM_CENTRALCONTROL_H_

// C++
#include <boost/shared_ptr.hpp>
#include <mutex>

// ROS
#include <ros/ros.h>

// CSLAM
#include <cslam/config.h>
#include <cslam/estd.h>
#include <cslam/ClientHandler.h>

// Thirdpary
#include "thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

using namespace std;
using namespace estd;

namespace cslam
{

    // forward decs
    class ClientHandler;
    //--------------

    struct CentralControl
    {
    public:
        typedef boost::shared_ptr<ClientHandler> chptr;
        typedef boost::shared_ptr<UniqueIdDispenser> uidptr;

    public:
        CentralControl(ros::NodeHandle Nh, ros::NodeHandle NhPrivate,
                       size_t ClientId,
                       eSystemState SysState,
                       chptr pCH = nullptr,
                       uidptr pUID = nullptr,
                       g2o::Sim3 g2oS_wc_wm = g2o::Sim3())
            : mNh(Nh), mNhPrivate(NhPrivate),
              mClientId(ClientId),
              mpCH(pCH),
              mpUID(pUID),
              mbOptActive(false),
              mSysState(SysState),
              mbCommLock(false), mbMappingLock(false), mbPlaceRecLock(false), mbTrackingLock(false),
              mg2oS_wcurmap_wclientmap(g2oS_wc_wm),
              mbGotMerged(false), mbOptimized(false)
        {
            //...
        }

        // ROS
        ros::NodeHandle mNh;
        ros::NodeHandle mNhPrivate;
        // Infrastucture
        uidptr mpUID;
        size_t mClientId;
        string mNativeOdomFrame;
        chptr mpCH;
        g2o::Sim3 mg2oS_wcurmap_wclientmap; // Sim3 world client to world map
        eSystemState mSysState;
        // System Control
        bool mbOptActive;
        bool mbGotMerged;
        bool mbOptimized; // signalizes that has seen GBA;
        Eigen::Matrix4d mT_SC;
        // Thread Sync
        bool LockComm()
        {
            unique_lock<mutex> lock(mMutexComm);
            if (!mbCommLock)
            {
                mbCommLock = true;
                return true;
            }
            else
                return false;
        }
        bool LockMapping()
        {
            unique_lock<mutex> lock(mMutexMapping);
            if (!mbMappingLock)
            {
                mbMappingLock = true;
                return true;
            }
            else
                return false;
        }
        bool LockPlaceRec()
        {
            unique_lock<mutex> lock(mMutexPlaceRec);
            if (!mbPlaceRecLock)
            {
                mbPlaceRecLock = true;
                return true;
            }
            else
                return false;
        }
        bool LockTracking()
        {
            unique_lock<mutex> lock(mMutexTracking);
            if (!mbTrackingLock)
            {
                mbTrackingLock = true;
                return true;
            }
            else
                return false;
        }
        void UnLockComm()
        {
            unique_lock<mutex> lock(mMutexComm);
            if (mbCommLock)
            {
                mbCommLock = false;
            }
            else
            {
                cout << "\033[1;31m!!! ERROR !!!\033[0m \"CentralControl\": Attempt to UnLock Comm -- was not locked" << endl;
                throw estd::infrastructure_ex();
            }
        }
        void UnLockMapping()
        {
            unique_lock<mutex> lock(mMutexMapping);
            if (mbMappingLock)
            {
                mbMappingLock = false;
            }
            else
            {
                cout << "\033[1;31m!!! ERROR !!!\033[0m \"CentralControl\": Attempt to UnLock Mapping -- was not locked" << endl;
                throw estd::infrastructure_ex();
            }
        }
        void UnLockPlaceRec()
        {
            unique_lock<mutex> lock(mMutexPlaceRec);
            if (mbPlaceRecLock)
            {
                mbPlaceRecLock = false;
            }
            else
            {
                cout << "\033[1;31m!!! ERROR !!!\033[0m \"CentralControl\": Attempt to UnLock PlaceRec -- was not locked" << endl;
                throw estd::infrastructure_ex();
            }
        }
        void UnLockTracking()
        {
            unique_lock<mutex> lock(mMutexTracking);
            if (mbTrackingLock)
            {
                mbTrackingLock = false;
            }
            else
            {
                cout << "\033[1;31m!!! ERROR !!!\033[0m \"CentralControl\": Attempt to UnLock Tracking -- was not locked" << endl;
                throw estd::infrastructure_ex();
            }
        }

        bool IsTrackingLocked()
        {
            unique_lock<mutex> lock(mMutexTracking);
            return mbTrackingLock;
        }

#ifdef LOGGING
        boost::shared_ptr<estd::mylog> mpLogger;
#endif

    private:
        // Thread Sync
        bool mbCommLock;
        bool mbMappingLock;
        bool mbPlaceRecLock;
        bool mbTrackingLock;
        // Mutexes
        mutex mMutexComm;
        mutex mMutexMapping;
        mutex mMutexPlaceRec;
        mutex mMutexTracking;
    };

} // end ns

#endif
