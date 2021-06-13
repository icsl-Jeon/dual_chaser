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

        // Parameter parsing
        bool isPreplanMode;
        nhPrivate.param("only_preplan",isPreplanMode,false);
        param.mode = isPreplanMode  ? PLANNING_LEVEL::PRE_PLANNING :  PLANNING_LEVEL::SMOOTH_PLANNING;

        nhPrivate.param("horizon",param.horizon,1.0f);
        state.horizon = param.horizon;
        nhPrivate.param<string>("frame_id",param.worldFrameId,"/map");
        nhPrivate.param("smooth_planner/preplan_track_rad",param.knotEps,float(0.3));
        nhPrivate.param("smooth_planner/length_step",param.lengthStepCorridorPoints,float(0.3));
        nhPrivate.param("smooth_planner/poly_order",param.polyOrder,5);

        nhPrivate.param("smooth_planner/weight/qp_vel",param.objWeights(0),(0.6));
        nhPrivate.param("smooth_planner/weight/qp_accel",param.objWeights(1),(0.05));
        nhPrivate.param("smooth_planner/weight/qp_jerk",param.objWeights(2),(0.1));
        param.smoothPathEvalPts = int(param.horizon * 10);

        nhPrivate.param("visualization/corridor_color/r",param.corridorColor.r,float(0.2));
        nhPrivate.param("visualization/corridor_color/g",param.corridorColor.g,float(0.2));
        nhPrivate.param("visualization/corridor_color/b",param.corridorColor.b,float(0.8));
        nhPrivate.param("visualization/corridor_color/a",param.corridorColor.a,float(0.7));

        nhPrivate.param("preplanner/target_collision_ellipse/x", param.TC_ellipsoidScaleCollision(0), double(0.4));
        nhPrivate.param("preplanner/target_collision_ellipse/y", param.TC_ellipsoidScaleCollision(1), double(0.4));
        nhPrivate.param("preplanner/target_collision_ellipse/z", param.TC_ellipsoidScaleCollision(2), double(0.8));

        // advertise
        string topicPrefix = "wrapper";
        pubSet.chaserTwist = nhPrivate.advertise<geometry_msgs::TwistStamped>(topicPrefix + "/velocity",1);
        pubSet.corridor= nhPrivate.advertise<visualization_msgs::MarkerArray>(topicPrefix + "/corridor",1);
        pubSet.curPlan = nhPrivate.advertise<nav_msgs::Path>(topicPrefix+"/cur_plan_traj",1);
        pubSet.curPlanHistory= nhPrivate.advertise<nav_msgs::Path>(topicPrefix+"/history_plan",1);

        callbackQueue.clear();
        nhTimer.setCallbackQueue(&callbackQueue);
        timerCaller  = nhTimer.createTimer(ros::Duration(0.05), &Wrapper::asyncTimerCallback, this);
        asyncSpinnerPtr = new ros::AsyncSpinner(1, &callbackQueue);
        tfListenerPtr = new tf::TransformListener;
        tfBroadcasterPtr = new tf::TransformBroadcaster;

        // state and visualization init
        visSet.corridorMarkerBase.header.frame_id = param.worldFrameId;
        visSet.corridorMarkerBase.action = visualization_msgs::Marker::ADD;
        visSet.corridorMarkerBase.type = visualization_msgs::Marker::CUBE;
        visSet.corridorMarkerBase.pose.orientation.w = 1.0;
        visSet.corridorMarkerBase.color = param.corridorColor;
        visSet.eraser = visSet.corridorMarkerBase;
        visSet.eraser.type = visualization_msgs::Marker::DELETE;
        visSet.curPlanHistory.header.frame_id = param.worldFrameId;
    }

    void Wrapper::prepare(preplanner::PlanningInput &planningInput) {
        ros::Time curTime = ros::Time::now();
        planningInput.predictionOutput = state.predictionOutput;
        planningInput.tTrigger = curTime;
        vector<Pose> targetInitPose = targetManagerPtr->lookupLastTargets();
        for (int n = 0; n < param.nTarget ; n++)
            planningInput.targetInit.points.push_back(targetInitPose[n].getTranslation());

        if (not state.isPlan)
            planningInput.initStateForPlanning = state.getChaserState();
        else{
            planningInput.initStateForPlanning = state.getPlanChaserState();
        }
    }

    vector<LoosePin3f> Wrapper::getCorridors(Traj preplanning,vector<Traj> targetPath) {
        vector<LoosePin3f> pinSet;
        int N = preplanning.ts.size()-1 ;
        //  1. knot enforce
        for(int n = 1 ; n <= N ;n++){
            Vector3f xl = preplanning.points[n].toEigen() - param.knotEps/sqrt(3)*Eigen::Vector3f::Ones();
            Vector3f xu = preplanning.points[n].toEigen() + param.knotEps/sqrt(3)*Eigen::Vector3f::Ones();
            float t = preplanning.ts[n];
            pinSet.emplace_back(t,0,xl,xu);
        }

        edtServerPtr->getLocker().lock();
        //  2. interval enforce (+ expansion)
        for (int n = 0 ; n < N ; n++){
            // per segment
            float length = preplanning.points[n].distTo(preplanning.points[n+1]);
            int K = floor(length/param.lengthStepCorridorPoints);
            float dt = (preplanning.ts[n+1]-preplanning.ts[n])/length*param.lengthStepCorridorPoints;
            for(int k = 1 ; k <= K ; k++){
                // expansion try against obstacle
                float t = preplanning.ts[n]+dt*float(k);
                Point p = preplanning.eval(t); // preplanning points
                octomap::point3d pnt(p.x,p.y,p.z);
                float safeExpansionRad_CO = edtServerPtr->getDistance(pnt);
                // expansion try against target
                float safeExpansionRad_CT = numeric_limits<float>::max(); // 2d
                for (int m = 0 ; m < param.nTarget ; m++) {
                    Point q = targetPath[m].eval(t);
                    float dist2d = sqrt(pow(q.x - p.x,2) + pow(q.y - p.y,2)  );
                    safeExpansionRad_CT = min (double(safeExpansionRad_CT) ,
                                               dist2d-max(param.TC_ellipsoidScaleCollision(0),
                                                          param.TC_ellipsoidScaleCollision(1) ));
                }

                float safeExpansionRad = (min(safeExpansionRad_CT, safeExpansionRad_CO)); // this can be shrink actually..
                Vector3f xl = p.toEigen() - safeExpansionRad/sqrt(3)*Eigen::Vector3f::Ones();
                Vector3f xu = p.toEigen() + safeExpansionRad/sqrt(3)*Eigen::Vector3f::Ones();
                pinSet.emplace_back(t,0,xl,xu);
            }
        }

        edtServerPtr->getLocker().unlock();
        return pinSet;
    }


    void Wrapper::prepare(smooth_planner::PlanningInput &planningInput,
                          const preplanner::PlanningOutput &planningOutput) {
        planningInput.preplanResult = planningOutput.waypoints;
        planningInput.prediction = planningOutput.targetPoints;
        planningInput.tTrigger = state.tLastPlanningTrigger;

        if (not state.isPlan)
            planningInput.initStateForPlanning = state.getChaserState();
        else{
            planningInput.initStateForPlanning = state.getPlanChaserState();
        }
    }


    bool Wrapper::canPlanning() {
        if (not state.isChaserPose){
            ROS_WARN("No chase frame %s was received from tf ", param.chaserFrameId.c_str());
            return false;
        }

        return true;
    }

    bool Wrapper::needPlanning() {
        if (state.isLastPlanningFailed)
            return true;
        return false;
    }

    bool Wrapper::trigger() {
        bool triggerCondtion = canPlanning() and (needPlanning() or targetManagerPtr->needPrediction());
        if (triggerCondtion)
            ROS_INFO("Wrapper: trigger planning");
        return triggerCondtion;
    }

    bool Wrapper::smoothPlan(const smooth_planner::PlanningInput &planningInput) {

        /**
         * 1. Determine corridors (the corridor size can be smaller than preplanning as new obstacles are updated)
         */
        vector<Traj> targetPathSet;
        for (int m =0; m < param.nTarget ; m++)
            targetPathSet.push_back(planningInput.prediction[m].traj);
        vector<LoosePin3f> corridors = getCorridors(planningInput.preplanResult.traj,targetPathSet);

        /**
         * 2. QP solve for polynomial trajectory
         */
        bool qpSolve;
        vector<float> timeKnots = planningInput.preplanResult.traj.ts;
        float t0 = planningInput.preplanResult.traj.ts[0]; // should be zero
        vector<Pin3f*> pinPtrs;
        for (int n = 0; n < corridors.size(); n++)
            pinPtrs.push_back(&corridors[n]);
        Pin3f* initialCondition = new FixPin3f(t0,0,planningInput.initStateForPlanning.position.toEigen());
        pinPtrs.push_back(initialCondition);
        TrajGenParam pp(param.polyOrder,2,trajgen::ALGORITHM::POLY_COEFF);
        TrajGen* trajGen = new TrajGen (timeKnots,pp);
        trajGen->setDerivativeObj(param.objWeights.cast<float>());
        trajGen->addPinSet(pinPtrs);
        qpSolve = trajGen->solve(false);


        /**
         *  update state visualization
         */
        mutex_.lock();

        // corridor (this will be definitely obtained if preplanning was successful)
        state.isCorridor = true;
        visSet.corridorMarkerArray.markers.clear();
        int id = 0;
        for (const auto &  corridor: corridors){
            auto scales = corridor.xu - corridor.xl;
            visSet.corridorMarkerBase.scale.x = scales(0);
            visSet.corridorMarkerBase.scale.y = scales(1);
            visSet.corridorMarkerBase.scale.z = scales(2);

            geometry_msgs::Point corridorCenter;
            corridorCenter.x = (corridor.xl(0) + corridor.xu(0))/2.0;
            corridorCenter.y = (corridor.xl(1) + corridor.xu(1))/2.0;
            corridorCenter.z = (corridor.xl(2) + corridor.xu(2))/2.0;
            visSet.corridorMarkerBase.pose.position = corridorCenter;
            visSet.corridorMarkerBase.id = id++;
            visSet.corridorMarkerArray.markers.push_back(visSet.corridorMarkerBase);
        }
        for (int eraseIdx = id; eraseIdx < state.nLastCorridor; eraseIdx++) {
            visSet.eraser.id = eraseIdx;
            visSet.corridorMarkerArray.markers.push_back(visSet.eraser);
        }
        state.nLastCorridor = corridors.size();

        // smooth trajectory (update only if qp was solved\)
        if(not state.isPlan)
            state.isPlan = qpSolve;
        if (qpSolve) {
            state.curPlan.smoothTrajectory.refTime = planningInput.tTrigger;
            state.curPlan.smoothTrajectory.trajPtr = trajGen;
            visSet.curPlan = state.getPlanTraj(param.worldFrameId,param.smoothPathEvalPts);
        }
        mutex_.unlock();
        return qpSolve;
    }


    geometry_msgs::TwistStamped Wrapper::State::getVelocityMarker(string chaserFrame) const {
        geometry_msgs::TwistStamped twist;
        twist.header.frame_id = chaserFrame;
        twist.header.stamp = ros::Time::now();

        Point veloicty_d (curChaserPose.poseMat.rotation().inverse() * curChaserVelocity.toEigen()); // velocity w.r.t drone frame

        twist.twist.linear.x = veloicty_d.x;
        twist.twist.linear.y = veloicty_d.y;
        twist.twist.linear.z = veloicty_d.z;
        return twist;
    }

    /**
     * Outputs the state of the chaser of last lookup
     * @return
     */
    ChaserState Wrapper::State::getChaserState() const {

        ChaserState chaserState;
        chaserState.stamp = tLastChaserStateUpdate;
        chaserState.velocity = curChaserVelocity;
        chaserState.position = curChaserPose.getTranslation();
        return chaserState;
    }

    ChaserState Wrapper::State::getPlanChaserState() const {
        ros::Time curTime = ros::Time::now();
        ChaserState returnState;
        returnState.stamp =curTime;
        if (isPlan){
            // E/H horizon check
            float tEval = (curTime - curPlan.smoothTrajectory.refTime).toSec();
            if (tEval > horizon)
                ROS_WARN("Wrapper: eval on planning exceed horizon. Clamping time! ");
            tEval = min (tEval , horizon);
            returnState.position = curPlan.smoothTrajectory.trajPtr->eval(tEval,0);
            returnState.velocity = curPlan.smoothTrajectory.trajPtr->eval(tEval,1);

        }else{
            ROS_ERROR("Wrapper: requested get-chaser plan but still no plan");
        }

        return returnState;
    }

    /**
     * Compute desired chaser pose (x,y,z,yaw) by looking at the middle of the targets
     * @param targets
     * @param seeFromDroneState true = yaw chosen from current drone position / false = from planed position
     * @return
     */
    ChaserState Wrapper::State::getPlanChaserState(PointSet targets, bool seeFromDroneState) const{
        ChaserState returnState = getPlanChaserState();
        Point seeFrom;
        if(seeFromDroneState){
            seeFrom = getChaserState().position;
        }else
            seeFrom = returnState.position;

        Point targetCenter = targets.center();
        float yaw = atan2(targetCenter.y - seeFrom.y , targetCenter.x - seeFrom.x);
        returnState.yaw = yaw;
        return returnState;
    }

    nav_msgs::Path Wrapper::State::getPlanTraj(string worldFrameId,int nPnt = 20) const {
        nav_msgs::Path path;
        int Neval = nPnt;
        VectorXf tEval(Neval); tEval.setLinSpaced(Neval,0,horizon);
        for (int n = 0 ; n < Neval ; n++){
            float tp = tEval(n);
            Point position = curPlan.smoothTrajectory.trajPtr->eval(tp,0);
            geometry_msgs::PoseStamped poseStamped;
            poseStamped.pose.position = position.toGeometry();
            path.poses.push_back(poseStamped);
        }
        path.header.frame_id = worldFrameId;
        return path;
    }


    bool Wrapper::plan(){

        state.tLastPlanningTrigger = ros:: Time::now();
        /**
         *  1. prediction
         */
        if (targetManagerPtr->predict(state.predictionOutput)){

            /**
             * 2. Preplan: in = prediction, chaser stsate / out = corridor, waypoints
             */
            preplanner::PlanningInput preplanningInput;
            prepare(preplanningInput) ;
            preplanner::PlanningOutput preplanningOutput;
            Timer timerPreplanner;
            if(not preplannerPtr->plan(preplanningInput,preplanningOutput)) {
                ROS_ERROR("Wrapper: preplanning failed. ");
                return false;
            }
            ROS_INFO("Wrapper: preplanning finished after %f ms (step = %d)", timerPreplanner.stop(),preplanningOutput.report.N);

            if (param.mode == PLANNING_LEVEL::PRE_PLANNING) {
                ROS_INFO("Wrapper: preplanning success. Finishing planning. ");
                return true;
            }


            /**
             * 3. Smooth planning:
             */
            smooth_planner::PlanningInput smoothPlanningInput;
            prepare(smoothPlanningInput,preplanningOutput);
            if (not smoothPlan(smoothPlanningInput)){
                ROS_ERROR("Wrapper: smooth planning failed. ");
                return false;
            }


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
                    ROS_ERROR("Wrapper: planning failed. will trigger new..");
                }
            ROS_INFO("Wrapper: planning success");
            ros::Rate(30).sleep();
            ros::spinOnce(); // for global callback queue (e.g. octomap)
        }
    }

    void Wrapper::updateTime(visualization_msgs::MarkerArray markerArray,ros::Time time){

        for (auto & marker: markerArray.markers){
            marker.header.stamp = time;
        }
    }

    Pose Wrapper::evalCurrentPlanningPose() {

        // eval current desired planning
        PointSet targetLocations;
        vector<Pose> targetInitPose = targetManagerPtr->lookupLastTargets();
        for (int n = 0; n < param.nTarget ; n++)
            targetLocations.points.push_back(targetInitPose[n].getTranslation());
        ChaserState curPlanState = state.getPlanChaserState(targetLocations,false);
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.pose = curPlanState.toGeoPose();
        Pose evalPose (poseStamped);
        return evalPose;
    }

    /**
     * Update chaser state and publish
     * @param event
     */
    void Wrapper::asyncTimerCallback(const ros::TimerEvent &event) {
        ros::Time curTime = ros::Time::now();
        // 1. update chaser pose
        tf::StampedTransform transform;
        try {
            tfListenerPtr->lookupTransform(param.worldFrameId, param.zedFrameId,
                                           ros::Time(0), transform);
            transform.child_frame_id_ = param.chaserFrameId;
            transform.stamp_ = curTime;
            tfBroadcasterPtr->sendTransform(transform);

            Pose newPose(transform);
            float dt = (transform.stamp_ - state.tLastChaserStateUpdate).toSec();
            state.curChaserVelocity = (newPose.getTranslation() - state.curChaserPose.getTranslation()) * (1.0/dt);
            state.curChaserPose = newPose;
            state.tLastChaserStateUpdate = curTime;
            state.isChaserPose  = true;


        } catch (tf::TransformException& ex) {
            ROS_ERROR_STREAM(ex.what());
        }

        // 2. publish the lsat "updated" state
        if (state.isChaserPose){
            pubSet.chaserTwist.publish(state.getVelocityMarker(param.chaserFrameId));
        }
        if (state.isCorridor) {
            updateTime(visSet.corridorMarkerArray, curTime);
            pubSet.corridor.publish(visSet.corridorMarkerArray);
        }
        if (state.isPlan){
            visSet.curPlan.header.stamp = curTime;
            pubSet.curPlan.publish(visSet.curPlan); // planning trajectory
            Pose curPlanPose = evalCurrentPlanningPose(); // output planning pose from current state

            // broadcast desired tf
            auto desiredTf = curPlanPose.toTf(param.worldFrameId,param.chaserPlanningFrameId,curTime);
            tfBroadcasterPtr->sendTransform(desiredTf);

            // collect current planning
            if ((curTime - state.tLastPlanningCollect).toSec() > param.historyCollectInterval){
                visSet.curPlanHistory.header.stamp = curTime;
                geometry_msgs::PoseStamped poseStamped;
                poseStamped.pose = curPlanPose.toGeoPose();
                poseStamped.header.frame_id = param.worldFrameId;
                poseStamped.header.stamp = curTime;
                visSet.curPlanHistory.poses.push_back(poseStamped);
            }
            pubSet.curPlanHistory.publish(visSet.curPlanHistory);
        }
    }






}