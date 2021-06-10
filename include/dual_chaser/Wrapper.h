//
// Created by jbs on 21. 6. 9..
//

#ifndef DUAL_CHASER_WRAPPER_H
#define DUAL_CHASER_WRAPPER_H
#include <dual_chaser/Preplanner.h>
using namespace chasing_utils;


/**
 * IMPORTANT
 * 1. prediction should be called in the same thread of planning as locker of edtServer is involved
 * 2.
 */

namespace dual_chaser{
    class Wrapper{
        enum PLANNING_LEVEL{
            PRE_PLANNING,
            SMOOTH_PLANNING
        };

        struct Param{
            int nTarget = 2;
            float horizon = 1.0; // read from target manager
            PLANNING_LEVEL mode = PLANNING_LEVEL::PRE_PLANNING;
        };

        struct State{
            vector<PredictionOutput> predictionOutput;
            Pose curChaserPose; // latest update at async callback
            Point curChaserVelocity; // latest update at async callback

            ChaserState getChaserState();
            ChaserState getPlanChaserState();

        };

    private:
        Param param;
        State state;
        ros::NodeHandle nhPrivate;
        ros::NodeHandle nhTimer;
        ros::Timer timerCaller;
        ros::AsyncSpinner* asyncSpinnerPtr;
        ros::CallbackQueue callbackQueue;

        octomap_server::EdtOctomapServer * edtServerPtr;
        chasing_utils::TargetManager * targetManagerPtr;
        preplanner::Preplanner* preplannerPtr;

        void initROS();
        void asyncTimerCallback(const ros::TimerEvent& event);

        bool needPlanning();
        bool canPlanning();

        void parseFromState(preplanner::PlanningInput& planningInput);

        bool plan();
        bool trigger();
    public:
        Wrapper();
        ~Wrapper() {asyncSpinnerPtr->stop();};
        void run();
    };


}




#endif //DUAL_CHASER_WRAPPER_H
