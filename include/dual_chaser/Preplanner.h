//
// Created by jbs on 21. 6. 9..
//

#ifndef DUAL_CHASER_PREPLANNER_H
#define DUAL_CHASER_PREPLANNER_H

#include <chasing_utils/TargetManager.h>
#include <dual_chaser/ScoreField.h>
#include <dual_chaser/GraphUtils.h>

using namespace chasing_utils;
using namespace std;

namespace dual_chaser{
    namespace preplanner {

        typedef pcl::PointCloud<pcl::PointXYZI> pclIntensity;
        typedef pcl::PointCloud<pcl::PointXYZ> pclLocation;

        struct Param {
            // planning
            float seedClearRad = 0.4;
            float lengthStep = 0.2;
            int nMaxStep = 4;

            // vsf construction
            Eigen::Vector3d ellipsoidScaleOcclusion; // ellipsoid model for targets of interest (for inter occlusion)
            float margin_xy = 4.0;
            float margin_z_up = 2.0;
            float margin_z_down = 2.0;
            float resolution = 0.2; // larger than edt resolution
            // todo modify for online (they are no more global things. Currently not using)
            float minZ;
            float maxZ;

            // graph construction
            bool TC_simple = true; // target collision by simple line checking
            int nThread = 6; // thread to compute VSF
            int graphNodeStride = 2; // stride when selecting the graph nodes from VSF grid
            float targetCollisionEps = 0.3; // used in simple sphere model for chaser-target collision checking
            Eigen::Vector3d TC_ellipsoidScaleCollisionMove; //  used in ellipsoid model for chaser-target collision checking
            Eigen::Vector3d TC_ellipsoidScaleCollisionStep; //  used in ellipsoid model for chaser-target collision checking
            float collisionEps = 0.4;
            float occlusionEps = 0.3;
            float collisionRayStride = 0.1;
            float maxBearing = M_PI/3.0*2.0;
            float maxConnectVelocity = 3.0; // maximum connection distance
            float desShotDist = 3;

            // graph solve
            float w_dist = 1.0;
            float w_visibility = 1.0;
            float w_bearing = 0.3;
            float w_des_rel_dist = 0.5;
            float w_init_vel_dir = 0.5;

            // visualizer
            std_msgs::ColorRGBA targetCollisionVolumeColor;
            std_msgs::ColorRGBA preplannerBearingColor;
            float preplannerBearingWidth;

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


        struct Report{
            int N; // number of connected time step
            vector<int> nRejectEdgesTargetCollision; //!
            vector<int> nRejectEdgesDistanceAllowable;
            vector<int> nRejectEdgesTraverseObstacleCollision;
            vector<int> nRejectEndPointOcclusion;
            vector<int> nRejectEndPointBearingViolation;

            vector<int> nEdgesFromPrevious;
            vector<int> nFeasibleAndConnectedNodes;
            vector<int> nTriedConnectionNode;

            float avgCostDist;
            float avgCostVis;
            float avgCostBearing;
            float avgCostRelDist;
            float avgCostDir;

            double elapseConstruction;
            double elapseSolve;
            void init() ;
            void report();
        };



        struct PlanningOutput{
            // result
            TrajStamped  waypoints; // t1~tN
            vector<TrajStamped> targetPoints; // t0~tN


            // report
            Report report;
        };

        class Preplanner {


            struct PublisherSet {
                ros::Publisher targetCollisionVolume;
                vector<ros::Publisher> targetPathSet; // future knots in the planning
                vector<ros::Publisher> vsfPclPath;
                vector<ros::Publisher> candidNodesPath;
                ros::Publisher graphSolution;
                ros::Publisher bearingMarker;
            };

            /**
             * Save visualizer of planning inputs and outputs
             */
            struct VisualizationSet {
                visualization_msgs::Marker markerTargetCollisionBase;
                visualization_msgs::MarkerArray markerTargetCollisionArray;
                vector<nav_msgs::Path> pathTargetPath;
                vector<pclIntensity> vsfPclPath; // path of VSF (multi) pcl for visualization
                vector<pcl::PointCloud<pcl::PointXYZ>> candidateNodesPath;
                nav_msgs::Path graphSolution;
                visualization_msgs::MarkerArray bearingMarker;
                visualization_msgs::Marker bearingBase;
            };

            struct State {
                // parsed from planning input
                octomap_server::EdtOctomapServer* edtServerPtr;
                PointSet curTargets;
                vector<TrajStamped> futureTargetPoints; // t1~
                vector<TrajStamped> totalTargetPoints; // t0~
                vector<PointSet> targetSetPath; // path of target set (= targetPathSet)
                Point chaserInit; // this might not be same with true drone state at the triggering
                Point chaserInitVel;
                int nLastQueryStep = 1; // [1,nMaxStep;]
                ros::Time tLastTrigger;
                vector<float> timeKnotLocal; /// 0 is included


                // Flag
                bool isInitSession = false;
                bool vsfSuccess = false;
                bool graphConstructionSuccess = false;
                bool graphSolveSuccess = false;

                // outcome of planning
                MultiVisibilityScoreField* vsf_path;
                ChaserGraph curGraph;
                Path solutionPath;
            };


        private:
            mutex mutex_; // mutex only between vis update and publish.
            Param param;
            State state;
            Report reporter;
            PublisherSet pubSet;
            VisualizationSet visSet;
            ros::NodeHandle nh;
            ros::NodeHandle nhTimer;
            ros::Timer timerCaller;
            ros::AsyncSpinner* asyncSpinnerPtr;
            ros::CallbackQueue callbackQueue;

            // MISC UTILS
            bool TC_collision (const LineSegment& targetMove,
                               const LineSegment& chaserMove) const;
            bool extendLayer(ChaserGraph& chaserGraph, const PointSet& newLayer, Param tryParam);

            // SUB ROUTINES
            bool createVsfPath();
            bool createGraph();
            bool solveGraph();

            // SESSION WRAPPER
            bool initSession(const PlanningInput& planningInput);
            bool inSession();
            void endSession(PlanningOutput& planningOutput);

            void asyncTimerCallback(const ros::TimerEvent& event);
            void initROS();
        public:
            Preplanner(octomap_server::EdtOctomapServer* edtServerPtr);
            bool plan(const PlanningInput& planningInput,PlanningOutput& planningOutput);
        };

    }

}




#endif //DUAL_CHASER_PREPLANNER_H
