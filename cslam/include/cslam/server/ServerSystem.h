#ifndef CSLAM_SERVERSYSTEM_H_
#define CSLAM_SERVERSYSTEM_H_

// C++
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <thread>
#include <mutex>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

// CSLAM
#include <cslam/config.h>
#include <cslam/estd.h>
#include <cslam/Datatypes.h>
#include <cslam/ORBVocabulary.h>
#include <cslam/Map.h>
#include <cslam/Database.h>
#include <cslam/ClientHandler.h>
#include <cslam/MapMatcher.h>
#include <cslam/Viewer.h>

#include "ccmslam/ServiceSaveMap.h"

using namespace std;
using namespace estd;

namespace cslam
{

    class ServerSystem
    {
    public:
        typedef boost::shared_ptr<ClientHandler> chptr;
        typedef boost::shared_ptr<KeyFrameDatabase> dbptr;
        typedef boost::shared_ptr<Map> mapptr;
        typedef boost::shared_ptr<MapMatcher> matchptr;
        typedef boost::shared_ptr<Viewer> viewptr;
        typedef boost::shared_ptr<CentralControl> ccptr;

    public:
        ServerSystem(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, const string &strVocFile);
        void InitializeClients();
        void InitializeMapMatcher();

        bool CallbackSaveMap(ccmslam::ServiceSaveMap::Request &req, ccmslam::ServiceSaveMap::Response &res);

    private:
        void LoadVocabulary(const string &strVocFile);
        void InitializeMaps();
        void InitializeKFDB();
        void InitializeMapping();
        void InitializeViewer();

        void CleanWriteOutFile(std::string sFileName);

        // ROS infrastructure
        ros::NodeHandle mNh;
        ros::NodeHandle mNhPrivate;

        ros::ServiceServer mServiceSavemap;

        vocptr mpVoc;
        dbptr mpKFDB;
        matchptr mpMapMatcher;
        viewptr mpViewer;

        chptr mpClient0;
        mapptr mpMap0;
        chptr mpClient1;
        mapptr mpMap1;
        chptr mpClient2;
        mapptr mpMap2;
        chptr mpClient3;
        mapptr mpMap3;

        const uidptr mpUID;

        // threads
        threadptr mptMapMatching;
        threadptr mptViewer;

        int mNumOfClients;
        int mMaxClients;

#ifdef LOGGING
        boost::shared_ptr<estd::mylog> mpLogger;
        threadptr mptLogger;
#endif
    };

} // end namespace

#endif
