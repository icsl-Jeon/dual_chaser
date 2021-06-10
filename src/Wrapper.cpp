#include <dual_chaser/Wrapper.h>

namespace dual_chaser{

    Wrapper::Wrapper() : nhPrivate("~"){
        edtServerPtr = new octomap_server::EdtOctomapServer;
        targetManagerPtr = new TargetManager(edtServerPtr);
        preplannerPtr = new preplanner::Preplanner(edtServerPtr);

        initROS();
        asyncSpinnerPtr->start();
    }


    void Wrapper::initROS() {

        callbackQueue.clear();
        nhTimer.setCallbackQueue(&callbackQueue);
        timerCaller  = nhTimer.createTimer(ros::Duration(0.05), &Wrapper::asyncTimerCallback, this);
        asyncSpinnerPtr = new ros::AsyncSpinner(1, &callbackQueue);
    }

    void Wrapper::parseFromState(preplanner::PlanningInput &planningInput) {
        ros::Time curTime = ros::Time::now();
        planningInput.predictionOutput = state.predictionOutput;
        planningInput.tTrigger = curTime;
        vector<Pose> targetInitPose = targetManagerPtr->lookupCurrentTargets();
        for (int n = 0; n < param.nTarget ; n++) {
            planningInput.targetInit.points.push_back(targetInitPose[n].getTranslation());
        }


    }

    bool Wrapper::canPlanning() {

        return true;
    }

    bool Wrapper::needPlanning() {
        return false;
    }

    bool Wrapper::trigger() {
        bool triggerCondtion = canPlanning() and (needPlanning() or targetManagerPtr->needPrediction());
        if (triggerCondtion)
            ROS_INFO("Wrapper: trigger planning");
        return triggerCondtion;
    }

    bool Wrapper::plan(){

        /**
         *  1. prediction
         */

        if (targetManagerPtr->predict(state.predictionOutput)){

            /**
             * 2. Preplan: in = prediction, chaser stsate / out = corridor, waypoints
             */
            preplanner::PlanningInput preplanningInput; parseFromState(preplanningInput) ;
            preplanner::PlanningOutput preplanningOutput;
            if(not preplannerPtr->plan(preplanningInput,preplanningOutput)) {
                ROS_ERROR("Wrapper: preplanning failed. ");
                return false;
            }

            if (param.mode == PLANNING_LEVEL::PRE_PLANNING) {
                ROS_INFO("Wrapper: preplanning success. Finishing planning. ");
                return true;
            }
            /**
             * 3. Smooth planning:
             */


        }else{
            ROS_ERROR("Wrapper: prediction failed");
            return false;
        }

        return true;
    }

    /**
     * Outermost loop: all the submodules independently update their states
     * and trigger planning if conditions are met
     */
    void Wrapper::run() {
        while(ros::ok()){
            if (trigger())
                if (not plan()) {
                    // do some exception handling

                }

            ros::Rate(30).sleep();
            ros::spinOnce(); // for global callback queue (e.g. octomap)
        }
    }


    /**
     * Update chaser state
     * @param event
     */
    void Wrapper::asyncTimerCallback(const ros::TimerEvent &event) {




    }



}