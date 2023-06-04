#ifndef CSLAM_MAPMERGER_H_
#define CSLAM_MAPMERGER_H_

// C++
#include <boost/shared_ptr.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>

// ROS
#include <ros/ros.h>
#include <ros/publisher.h>

// CSLAM
#include <cslam/config.h>
#include <cslam/estd.h>
#include <cslam/Converter.h>
#include <cslam/Datatypes.h>
#include <cslam/CentralControl.h>
#include <cslam/Map.h>
#include <cslam/KeyFrame.h>
#include <cslam/Optimizer.h>
#include <cslam/MapMatcher.h>
#include <cslam/ClientHandler.h>
#include <cslam/MapPoint.h>

// THIRDPARTY
#include "thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

using namespace std;
using namespace estd;

namespace cslam
{

    // forward decs
    class MapMatcher;
    class ClientHandler;
    class Map;
    class CentralControl;
    class KeyFrame;
    class MapPoint;
    struct MapMatchHit;
    //-----------------

    class MapMerger
    {
    public:
        typedef boost::shared_ptr<MapMerger> mergeptr;
        typedef boost::shared_ptr<MapMatcher> matchptr;
        typedef boost::shared_ptr<ClientHandler> chptr;
        typedef boost::shared_ptr<KeyFrame> kfptr;
        typedef boost::shared_ptr<MapPoint> mpptr;
        typedef boost::shared_ptr<Map> mapptr;
        typedef boost::shared_ptr<CentralControl> ccptr;

    public:
        typedef pair<set<kfptr>, int> ConsistentGroup;
        typedef map<kfptr, g2o::Sim3, std::less<kfptr>,
                    Eigen::aligned_allocator<std::pair<const kfptr, g2o::Sim3>>>
            KeyFrameAndPose;

    public:
        MapMerger(matchptr pMatcher);
        mapptr MergeMaps(mapptr pMapCurr, mapptr pMapMatch, vector<MapMatchHit> vMatchHits);

        bool isBusy();

    private:
        void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap, std::vector<mpptr> vpLoopMapPoints);
        void SetBusy();
        void SetIdle();

        matchptr mpMatcher;

        bool bIsBusy;
        mutex mMutexBusy;

        std::vector<kfptr> mvpCurrentConnectedKFs;

        void RunGBA(idpair nLoopKf, mapptr pFusedMap);
    };

} // end namespace

#endif
