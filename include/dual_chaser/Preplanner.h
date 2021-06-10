//
// Created by jbs on 21. 6. 9..
//

#ifndef DUAL_CHASER_PREPLANNER_H
#define DUAL_CHASER_PREPLANNER_H

#include <chasing_utils/TargetManager.h>
#include <dual_chaser/ScoreField.h>

using namespace chasing_utils;
using namespace std;

namespace dual_chaser{
    namespace preplanner {

        typedef pcl::PointCloud<pcl::PointXYZI> pclIntensity;
        typedef pcl::PointCloud<pcl::PointXYZ> pclLocation;

        struct Param {
            // planning
            float lengthStep = 0.2;
            int nMaxStep = 4;

                // vsf construction
            Eigen::Vector3d ellipsoidScale; // ellipsoid model for targets of interest (for inter occlusion)
            float margin_xy = 4.0;
            float margin_z = 2.0;
            float resolution = 0.2; // larger than edt resolution
            // todo modify for online (they are no more global things..)
            float minZ;
            float maxZ;

            int nThread = 6; // thread to compute VSF
            int graphNodeStride = 2; // stride when selecting the graph nodes from VSF grid
            float targetCollisionEps = 0.3;
            float collisionEps = 0.1;
            float occlusionEps = 0.1;
            float collisionRayStride = 0.1;
            float maxBearing = M_PI/3.0*2.0;
            float maxDist = 3.0; // maximum connection distance
            float desShotDist = 3;

            // graph solve
            float w_dist = 1.0;
            float w_visibility = 1.0;
            float w_bearing = 0.3;
            float w_des_rel_dist = 0.5;
            float w_init_vel_dir = 0.5;


            // visualizer
            std_msgs::ColorRGBA targetCollisionVolumeColor;

            // to be sync with global horizon
            string worldFrameId = "map";
            float horizon = 1.0;
            int nTarget =2;
        };

        struct PlanningInput{
            ros::Time tTrigger;
            vector<PredictionOutput> predictionOutput; // (X1,X2,X3,...,XN)_{m=1,...,M}
            PointSet targetInit; // X0_{m=1,..M}
            ChaserState initStateForPlanning; // either real drone state or last plan pose

        };

        struct PlanningOutput{
            // result


            // report

        };


        class Preplanner {
            struct PublisherSet {
                ros::Publisher targetCollisionVolume;
                vector<ros::Publisher> targetPathSet; // future knots in the planning
                vector<ros::Publisher> vsfPclPath;
            };

            /**
             * Save visualizer of planning inputs and outputs
             */
            struct VisualizationSet {
                visualization_msgs::Marker markerTargetCollision;
                vector<nav_msgs::Path> pathTargetPath;
                vector<pclIntensity> vsfPclPath; // path of VSF (multi) pcl for visualization
            };

            struct State {
                // parsed from planning input
                octomap_server::EdtOctomapServer* edtServerPtr;
                PointSet curTargets;
                vector<TrajStamped> futureTargetPoints;
                vector<PointSet> targetSetPath; // path of target set (= targetPathSet)
                Point chaserInit; // this might not be same with true drone state at the triggering
                Point chaserInitVel;
                int nLastQueryStep = 1; // [1,nMaxStep;]

                // Flag
                bool isInitSession = false;
                bool vsfSuccess = false;

                // outcome of planning
                MultiVisibilityScoreField* vsf_path;
            };


        private:
            mutex mutex_; // mutex only between vis update and publish.
            Param param;
            State state;
            PublisherSet pubSet;
            VisualizationSet visSet;
            ros::NodeHandle nh;
            ros::NodeHandle nhTimer;
            ros::Timer timerCaller;
            ros::AsyncSpinner* asyncSpinnerPtr;
            ros::CallbackQueue callbackQueue;

            bool createVsfPath();

            bool initSession(const PlanningInput& planningInput);
            bool inSession();

            void asyncTimerCallback(const ros::TimerEvent& event);
            void initROS();
        public:
            Preplanner(octomap_server::EdtOctomapServer* edtServerPtr);
            bool plan(const PlanningInput& planningInput,PlanningOutput& planningOutput);
        };

    }

}




#endif //DUAL_CHASER_PREPLANNER_H
