//
// Created by jbs on 21. 6. 9..
//

#ifndef DUAL_CHASER_WRAPPER_H
#define DUAL_CHASER_WRAPPER_H
#include <dual_chaser/Preplanner.h>
#include <traj_gen/TrajGen.hpp>
#include <dual_chaser_msgs/Status.h>
#include <dual_chaser_msgs/ActivateChaser.h>


using namespace chasing_utils;


/**
 * IMPORTANT
 * 1. prediction should be called in the same thread of planning as locker of edtServer is involved
 * 2.
 */

namespace dual_chaser{

    typedef trajgen::LoosePin<float,3> LoosePin3f;
    typedef trajgen::FixPin<float,3> FixPin3f;
    typedef trajgen::Pin<float,3> Pin3f;
    typedef trajgen::PolyTrajGen<float,3> TrajGen;
    typedef trajgen::PolyParam TrajGenParam;

    namespace smooth_planner{

        struct PolynomialStamped{
            ros::Time refTime;
            TrajGen* trajPtr;
        };

        struct PlanningInput{
            ros::Time tTrigger;
            ChaserState initStateForPlanning;
            TrajStamped preplanResult; // (t0,X0), (t1,X1) ,..., (tN,XN)
            vector<TrajStamped> prediction;  // (t0,Q0), (t1,Q1) ,..., (tN,QN)

        };

        // This will be saved to the state of Wrapper
        struct PlanningOutput{
            float validHorizon;
            smooth_planner::PolynomialStamped droneTrajectory;
            bool isSafe( octomap_server::EdtOctomapServer* edtServer,
                        ros::Time fromHere, float safeRad) const ;

        };

    }



    class Wrapper{
        enum PLANNING_LEVEL{
            PRE_PLANNING,
            SMOOTH_PLANNING
        };

        struct Param{
            Vector3d objWeights;
            int nTarget = 2;
            float horizon = 1.0;
            float historyCollectInterval = 0.1;
            float bearingCollectInterval = 0.2;
            PLANNING_LEVEL mode = PLANNING_LEVEL::PRE_PLANNING;
            float lengthStepCorridorPoints = 0.3 ;
            float knotEps = 0.3;
            float OC_collisionEps = 0.3;
            Vector3d TC_ellipsoidScaleCollision;
            Eigen::Vector3d ellipsoidScaleOcclusion; // ellipsoid model for targets of interest (for inter occlusion)

            std_msgs::ColorRGBA corridorColor;

            int polyOrder = 5;
            int smoothPathEvalPts ;
            float planPoseSmoothing = 0.8;

            string worldFrameId = "map";
            string chaserFrameId = "drone_link"; // simple re broadcast zedFrameId as current client (NUC) time
            string zedFrameId = "base_link";
            string chaserPlanningFrameId = "desired_pose";

        };
        struct State{
            bool isChaserActive = false; // true = ruin planner
            float horizon;
            ros::Time tLastChaserStateUpdate; // time of the stamp of last lookup-ed tf of chaser
            ros::Time tLastCallbackTime; // ros time when the async callback called
            ros::Time tLastPlanningTrigger;  // even if planning failed, it is recorded ?
            ros::Time tLastPlanningCollect = ros::Time(0);
            ros::Time tLastBearingCollect = ros::Time(0);
            bool isChaserPose = false;
            bool isCorridor = false;
            bool isPlan = false;
            bool isPlanPoseEmitted = false;
            bool isLastPlanningFailed = false;
            int nLastCorridor = 0;

            vector<PredictionOutput> predictionOutput; // obtained prediction at plan trigger (not modified if prediction failed)
            smooth_planner::PlanningOutput curPlan; // current planning result (smooth plan)
            Pose curChaserPose; // latest update at async callback
            Point curChaserVelocity; // latest update at async callback (world frame)
            Point curChaserAcceleration;
            ChaserState lastEmitPlanPose; // update in emitCurrentPlanningPose

            ChaserState getChaserState() const; // returns current drone state
            ChaserState getPlanChaserState() const ; // returns current planning evaluation
            ChaserState getPlanChaserState(PointSet targets,
                                           bool seeFromDroneState = true ) const ; // returns current planning evaluation

            ChaserState getPlanChaserState(PointSet targets,
                                           ChaserState smoothFrom, float smoothing,
                                           bool seeFromDroneState = true ) const ; // returns current planning evaluation

            geometry_msgs::TwistStamped getVelocityMarker(string chaserFrame) const;
            nav_msgs::Path getPlanTraj(string worldFrameId, int nPnt) const;
        };

        struct PublisherSet {
            ros::Publisher chaserTwist;
            ros::Publisher corridor;
            ros::Publisher curPlan;
            ros::Publisher curPlanHistory;
            ros::Publisher bearingHistoryArrowBase[2];
            ros::Publisher bearingHistory[2]; // target A and B
            ros::Publisher targetHistory[2];
            ros::Publisher chaserStatus;
            ros::Publisher emitPlanningPose;
            ros::ServiceServer srvActivate;
        };

        struct VisualizationSet{
            visualization_msgs::Marker corridorMarkerBase;
            visualization_msgs::MarkerArray corridorMarkerArray;
            visualization_msgs::Marker eraser;
            nav_msgs::Path curPlan;
            nav_msgs::Path curPlanHistory;
            visualization_msgs::Marker bearingHistoryArrowBase[2];
            visualization_msgs::MarkerArray bearingHistory[2]; // target A and B
            nav_msgs::Path targetHistory[2];
        };

    private:
        mutex mutexVis;
        mutex mutexState;
        PublisherSet pubSet;
        VisualizationSet visSet;
        Param param;
        State state;
        ros::NodeHandle nhPrivate;
        ros::NodeHandle nhTimer;
        ros::Timer timerCaller;
        ros::AsyncSpinner* asyncSpinnerPtr;
        ros::CallbackQueue callbackQueue;
        tf::TransformListener* tfListenerPtr;
        tf::TransformBroadcaster* tfBroadcasterPtr;

        octomap_server::EdtOctomapServer * edtServerPtr;
        chasing_utils::TargetManager * targetManagerPtr;
        preplanner::Preplanner* preplannerPtr;

        void initROS();
        bool activateCallback(dual_chaser_msgs::ActivateChaserRequest& req,
                              dual_chaser_msgs::ActivateChaserResponse& resp);
        void asyncTimerCallback(const ros::TimerEvent& event);
        void updateTime(visualization_msgs::MarkerArray markerArray,ros::Time time);

        bool needPlanning();
        bool canPlanning();

        void prepare(preplanner::PlanningInput& planningInput);
        void prepare(smooth_planner::PlanningInput& planningInput,
                     const preplanner::PlanningOutput& planningOutput);

        vector<LoosePin3f> getCorridors(Traj preplanning, vector<Traj> targetPath);
        bool smoothPlan(const smooth_planner::PlanningInput& planningInput);

        bool plan();
        bool trigger();
        Pose emitCurrentPlanningPose();
        dual_chaser_msgs::Status getCurStatus() const;

    public:
        Wrapper();
        ~Wrapper() {asyncSpinnerPtr->stop();};
        void run();
    };


}




#endif //DUAL_CHASER_WRAPPER_H
