//
// Created by jbs on 21. 6. 9..
//
#include <dual_chaser/Preplanner.h>

namespace dual_chaser{
    namespace preplanner{
        void Report::init() {
            N = 0; // longest tried time step for edge connection
            nRejectEdgesTargetCollision.clear();
            nRejectEdgesDistanceAllowable.clear();
            nRejectEdgesTraverseObstacleCollision.clear();
            nRejectEndPointOcclusion.clear();
            nRejectEndPointBearingViolation.clear();

            nEdgesFromPrevious.clear();
            nFeasibleAndConnectedNodes.clear();
            nTriedConnectionNode.clear();

            elapseConstruction = -1;
            elapseSolve = -1;
        }

        void Report::report() {

            // header
            int nEffectiveCol = 6;
            TOP_RULE_STAR(nEffectiveCol)
            TITLE(nEffectiveCol,"DAG statistics (n=0: current / n=1... planning)")

            TOP_RULE_STAR(nEffectiveCol)
            FILL_TAG("[u->v]")
            FILL_CELL_RIGHT("[CV/TE]")
            FILL_CELL_RIGHT("[R-TC]")
            FILL_CELL_RIGHT(" > [R-DV]")
            FILL_CELL_RIGHT(" > [R-OO]")
            FILL_CELL_RIGHT(" > [R-OC]")
            FILL_CELL_RIGHT(" > [R-B]")
            MID_RULE_DASH(nEffectiveCol)

            for (int n = 1 ; n <= N  ; n++){
                FILL_TAG(to_string(n-1) + "->" + to_string(n));
                if (n > 1)
                    FILL_CELL_RIGHT( to_string(nFeasibleAndConnectedNodes[n-1]) + "/"
                        + to_string(nFeasibleAndConnectedNodes[n-2]) + "x" + to_string(nTriedConnectionNode[n-1]) )
                else
                    FILL_CELL_RIGHT( to_string(nFeasibleAndConnectedNodes[n-1]) + "/"
                                     +   "1x" + to_string(nTriedConnectionNode[n-1]) )
                FILL_CELL_RIGHT(-nRejectEdgesTargetCollision[n - 1]);
                FILL_CELL_RIGHT(-nRejectEdgesDistanceAllowable[n - 1]);
                FILL_CELL_RIGHT(-nRejectEndPointOcclusion[n - 1]);
                FILL_CELL_RIGHT(-nRejectEdgesTraverseObstacleCollision[n - 1]);
                FILL_CELL_RIGHT(-nRejectEndPointBearingViolation[n - 1]);
                if (n != N)
                    NEW_LINE
            }
            MID_RULE_DASH(nEffectiveCol)
            cout << "CV: connected vertex / TV: tried edge" << endl;
            cout << "Edge rejection reason" << endl;
            cout << "TC: target colli. /DV: dist. viol./ OO pnt occlu. /OC: obst. line colli. / B: bearing" << endl;
            cout << "graph construction [ms] " << elapseConstruction << endl;
            cout << "graph solve [ms] " <<  elapseSolve << endl;
            MID_RULE_DASH(nEffectiveCol)
        }

        Preplanner::Preplanner(octomap_server::EdtOctomapServer* edtServerPtr) :nh("~") {
            state.edtServerPtr = edtServerPtr;
            initROS();
            asyncSpinnerPtr->start();
        }
        void Preplanner::initROS() {


            /**
             * 1. Parameter parsing
             */

            nh.param("horizon",param.horizon,1.0f);
            nh.param("n_target",param.nTarget,2);
            nh.param("preplanner/n_max_step",param.nMaxStep,4);
            nh.param("preplanner/max_bearing",param.maxBearing,float(M_PI/2.0));
            float angleDeg;
            nh.param("preplanner/max_angle_diff_from_sensor_deg",angleDeg,120.0f);
            param.maxAngleDeviationFromSensor = angleDeg * M_PI / 180.0;
            nh.param("preplanner/score_field_resolution",param.resolution,float(0.3));
            nh.param("preplanner/score_field_node_stride",param.graphNodeStride,3);
            nh.param("preplanner/score_field_margin/xy",param.margin_xy,float(4.0));
            nh.param("preplanner/score_field_margin/z_top",param.margin_z_up,float(2.5));
            nh.param("preplanner/score_field_margin/z_bottom",param.margin_z_down,float(0.5));
            nh.param("preplanner/min_height",param.minZ,-10000.0f);
            nh.param("preplanner/max_height",param.maxZ,10000.0f);
            nh.param("preplanner/des_dist",param.desShotDist,float(3.5));
            nh.param("preplanner/score_field_max_connect_vel", param.maxConnectVelocity, float(4.5));
            nh.param("preplanner/n_thread",param.nThread,6);

            nh.param("preplanner/weight/distance",param.w_dist,float(1.0));
            nh.param("preplanner/collision_clearance",param.collisionEps,float(0.5));
            nh.param("preplanner/seed_obst_clear_rad",param.seedClearRad,float(0.5));
            nh.param("preplanner/target_collision_simple",param.TC_simple,false);
            nh.param("preplanner/target_collision_sphere_rad",param.targetCollisionEps,float(0.8));
            nh.param("preplanner/target_collision_ellipse_line/x", param.TC_ellipsoidScaleCollisionMove(0), double(0.4));
            nh.param("preplanner/target_collision_ellipse_line/y", param.TC_ellipsoidScaleCollisionMove(1), double(0.4));
            nh.param("preplanner/target_collision_ellipse_line/z", param.TC_ellipsoidScaleCollisionMove(2), double(0.8));
            nh.param("preplanner/target_collision_ellipse_step/x", param.TC_ellipsoidScaleCollisionStep(0), double(0.4));
            nh.param("preplanner/target_collision_ellipse_step/y", param.TC_ellipsoidScaleCollisionStep(1), double(0.4));
            nh.param("preplanner/target_collision_ellipse_step/z", param.TC_ellipsoidScaleCollisionStep(2), double(0.8));
            nh.param("preplanner/occlusion_clearance",param.occlusionEps,float(0.1));
            nh.param("preplanner/weight/bearing",param.w_bearing,float(0.3));
            nh.param("preplanner/weight/rel_distance",param.w_des_rel_dist,float(0.5));
            nh.param("preplanner/weight/visibility",param.w_visibility,float(1.0));
            nh.param("preplanner/weight/init_vel_dir",param.w_init_vel_dir,float(0.5));

            nh.param("preplanner/target_occlusion_ellipse/x", param.ellipsoidScaleOcclusion(0), double(0.5));
            nh.param("preplanner/target_occlusion_ellipse/y", param.ellipsoidScaleOcclusion(1), double(0.5));
            nh.param("preplanner/target_occlusion_ellipse/z", param.ellipsoidScaleOcclusion(2), double(0.5));

            nh.param("visualization/target_collision_volume/r",param.targetCollisionVolumeColor.r,float(0.2));
            nh.param("visualization/target_collision_volume/g",param.targetCollisionVolumeColor.g,float(0.2));
            nh.param("visualization/target_collision_volume/b",param.targetCollisionVolumeColor.b,float(0.8));
            nh.param("visualization/target_collision_volume/a",param.targetCollisionVolumeColor.a,float(0.7));

            nh.param("visualization/preplanner_bearing_arrow/r",param.preplannerBearingColor.r,float(0.1));
            nh.param("visualization/preplanner_bearing_arrow/g",param.preplannerBearingColor.g,float(0.1));
            nh.param("visualization/preplanner_bearing_arrow/b",param.preplannerBearingColor.b,float(0.1));
            nh.param("visualization/preplanner_bearing_arrow/a",param.preplannerBearingColor.a,float(0.7));
            nh.param("visualization/preplanner_bearing_arrow/w",param.preplannerBearingWidth,float(0.1));


            /**
             * 2. Communication register
             */

            string topicPrefix = "preplanner";
            pubSet.targetCollisionVolume = nh.advertise<visualization_msgs::MarkerArray>(topicPrefix + "/target_collision_volume",1);
            pubSet.bearingMarker = nh.advertise<visualization_msgs::MarkerArray>(topicPrefix + "/bearing",1);
            pubSet.graphSolution = nh.advertise<nav_msgs::Path>(topicPrefix + "/graph_solution",1);

            for(int m = 0 ; m < param.nTarget; m++) {
                string topicPrefixTarget = topicPrefix + "/target_" + to_string(m) ;
                ros::Publisher publisher = nh.advertise<nav_msgs::Path>(topicPrefixTarget + "/target_path" , 1);
                pubSet.targetPathSet.push_back(publisher);
            }

            for (int n = 0 ; n < param.nMaxStep ; n++) {
                string topicPrefixStep = topicPrefix + "/step_" + to_string(n) ;
                ros::Publisher publisher = nh.advertise<pclIntensity>(topicPrefixStep + "/vsf_path" , 1);
                pubSet.vsfPclPath.push_back(publisher);
                ros::Publisher publisher1 = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(topicPrefixStep + "/graph_nodes",1);
                pubSet.candidNodesPath.push_back(publisher1);
            }

            callbackQueue.clear();
            nhTimer.setCallbackQueue(&callbackQueue);
            timerCaller  = nhTimer.createTimer(ros::Duration(0.05), &Preplanner::asyncTimerCallback, this);
            asyncSpinnerPtr = new ros::AsyncSpinner(1, &callbackQueue);


            /**
             * 3. visualizer init
             */
            visSet.markerTargetCollisionBase.header.frame_id = param.worldFrameId;
            visSet.markerTargetCollisionBase.type = visualization_msgs::Marker::SPHERE;
            if (param.TC_simple) {
                visSet.markerTargetCollisionBase.scale.x = 2 * param.targetCollisionEps;
                visSet.markerTargetCollisionBase.scale.y = 2 * param.targetCollisionEps;
                visSet.markerTargetCollisionBase.scale.z = 2 * param.targetCollisionEps;
            }else{
                visSet.markerTargetCollisionBase.scale.x = 2 * param.TC_ellipsoidScaleCollisionMove(0);
                visSet.markerTargetCollisionBase.scale.y = 2 * param.TC_ellipsoidScaleCollisionMove(1);
                visSet.markerTargetCollisionBase.scale.z = 2 * param.TC_ellipsoidScaleCollisionMove(2);
            }

            visSet.markerTargetCollisionBase.color = param.targetCollisionVolumeColor;
            visSet.markerTargetCollisionBase.pose.orientation.w = 1.0;
            visSet.markerTargetCollisionArray.markers.resize(param.nTarget);

            visSet.pathTargetPath.resize(param.nTarget);

            visSet.bearingBase.header.frame_id = param.worldFrameId;
            visSet.bearingBase.type = visualization_msgs::Marker::ARROW;
            visSet.bearingBase.scale.y = 0.08;
            visSet.bearingBase.scale.x = param.preplannerBearingWidth;
            visSet.bearingBase.scale.z = 0.05;
            visSet.bearingBase.color = param.preplannerBearingColor;
            visSet.bearingBase.pose.orientation.w = 1.0;


            /**
             * 4. State initialization
             */
            state.vsf_path = new MultiVisibilityScoreField[param.nMaxStep];
            visSet.vsfPclPath.resize(param.nMaxStep);

        }

        /**
         * Prepare preplanner to run planning session by uploading planning inputs to state
         * @param planningInput
         * @return
         */
        bool Preplanner::initSession(const PlanningInput &planningInput) {
            // Sanity check
            if (planningInput.predictionOutput.empty()) {
                ROS_ERROR("Preplanner: no prediction. Aborting init session");
                state.isInitSession = false;
                return false;
            }
            if (param.nTarget <1){
                ROS_ERROR("Preplanner: n target set to zero. Aborting init session");
                state.isInitSession = false;
                return false;
            }

            // 1. determine future knots depending on the length of prediction (input -> state)
            ros::Time triggerTime = planningInput.tTrigger;
            ros::Time finalTime = triggerTime + ros::Duration(param.horizon);
            int nTarget = param.nTarget;
            vector<float> predictionLengthSet;
            for (int n = 0; n < nTarget; n++) {
                const PredictionOutput &prediction = planningInput.predictionOutput[n];
                Traj traj = prediction.getTraj();
                float t0 = (triggerTime - prediction.getRefTime()).toSec();
                float tf = (finalTime - prediction.getRefTime()).toSec();
                if (tf < param.horizon)
                    ROS_WARN("Wrapper: prediction horizon %f of %d th target is too short than required.",
                             param.horizon, n);
                float length = traj.get_length(t0, tf);
                predictionLengthSet.push_back(length);
            }

            float lengthMax = *max_element(predictionLengthSet.begin(), predictionLengthSet.end());
            int N = min(max(int(ceil(lengthMax / float(param.lengthStep))), 1),
                        param.nMaxStep); //  number of future step (X1,X2,...,XN)

            float dt = (finalTime - triggerTime).toSec() / float(N);

            vector<TrajStamped> totalTargetPoints; // t0 ~
            vector<TrajStamped> futureTargetPoints; // t1 ~
            for (int m = 0; m < nTarget; m++) {
                const PredictionOutput &prediction = planningInput.predictionOutput[m];
                TrajStamped trajTotal;
                TrajStamped traj; // (X1,X2,...,XN)
                traj.refTime = triggerTime;
                trajTotal.refTime = triggerTime;

                trajTotal.traj.ts.push_back(0);
                trajTotal.traj.points.push_back(planningInput.targetInit.points[m]);

                for (int n = 0; n < N; n++) {
                    float tLocal = (n + 1) * dt;
                    ros::Time tGlobal = traj.refTime + ros::Duration(tLocal);
                    traj.traj.ts.push_back(tLocal);
                    traj.traj.points.push_back(prediction.eval(tGlobal));

                    trajTotal.traj.ts.push_back(tLocal);
                    trajTotal.traj.points.push_back(prediction.eval(tGlobal));
                }
                futureTargetPoints.push_back(traj);
                totalTargetPoints.push_back(trajTotal);
            }

            // 2. Upload state and visualization of current planning inputs (state -> visualizer)
            state.dronePosition = planningInput.droneState.position;
            state.futureTargetPoints = futureTargetPoints;
            state.totalTargetPoints = totalTargetPoints;
            state.curTargets = planningInput.targetInit;
            state.isInitSession = true;
            state.nLastQueryStep = N;
            state.chaserInit = planningInput.initStateForPlanning.position;
            state.chaserInitVel = planningInput.initStateForPlanning.velocity;
            state.timeKnotLocal.clear(); state.timeKnotLocal.push_back(0.0);
            state.timeKnotLocal.insert(state.timeKnotLocal.end(),
                                       futureTargetPoints[0].traj.ts.begin(),
                                       futureTargetPoints[0].traj.ts.end());
            state.tLastTrigger = planningInput.tTrigger;

            mutex_.lock();
            for (int m = 0; m < nTarget; m++) {
                visSet.markerTargetCollisionBase.pose.position = state.curTargets.points[m].toGeometry();
                visSet.markerTargetCollisionBase.id = m;
                visSet.markerTargetCollisionArray.markers[m] =  visSet.markerTargetCollisionBase;
                visSet.pathTargetPath[m] = state.futureTargetPoints[m].traj.toNavPath(param.worldFrameId);
            }
            mutex_.unlock();

            return true;
        }

        /**
         * Check whether two linear movements between target and chaser collides
         * @param targetMove
         * @param chaserMove
         * @return
         * @details simple sphere model vs ellipsoid model
         */
        bool Preplanner::TC_collision(const LineSegment &targetMove, const LineSegment &chaserMove) const {

            if (param.TC_simple)
                return targetMove.distTo(chaserMove) < param.targetCollisionEps;
            else{
                unsigned int nTestPoints = 5;
                PointSet targetPoints = targetMove.samplePoints(nTestPoints);
                PointSet chaserPoints = chaserMove.samplePoints(nTestPoints);
                for (int n = 0; n < nTestPoints ; n++){
                    EllipsoidNoRot ellipse(targetPoints.points[n].toEigen().cast<double>(),
                            param.TC_ellipsoidScaleCollisionMove);
                    if (ellipse.evalDist(chaserPoints.points[n]) <= 0 )
                        return true;
                }
                return false;
            }
        }

        bool Preplanner::createVsfPath(){
            bool isOk = true;
            int N = state.nLastQueryStep;
            int M = param.nTarget;
            state.targetSetPath.resize(N);

            // 1. Sizing field parameters
            vector<ScoreFieldParam> vsfParamPath;
            bool isValidField = true;
            for (int n = 0; n < N; n++) {
                vector<EllipsoidNoRot> extraObstacles(M);
                state.targetSetPath[n].points.clear();
                for (int m = 0; m < M; m++) {
                    state.targetSetPath[n].points.push_back(state.futureTargetPoints[m].traj.points[n]);
                    extraObstacles[m] = EllipsoidNoRot(state.targetSetPath[n].points[m].toEigen().cast<double>(),
                                                       param.ellipsoidScaleOcclusion );
                }
                // determine size of field
                vector<Point> includePoints;
                includePoints.insert(includePoints.end(),state.targetSetPath[n].points.begin(),state.targetSetPath[n].points.end());
                if (n ==0 )
                    includePoints.push_back(state.chaserInit);
                ScoreFieldParam fieldParam(param.resolution, includePoints ,
                                           param.margin_xy ,
//                                           param.margin_xy ,
                                           param.margin_z_up  ,
                                           param.margin_z_down  ,
                                           param.minZ, param.maxZ, isValidField);

                if (not isValidField) {
                    ROS_ERROR("Preplanner: grid field of target %d found invalid. Aborting VSF creation.",n);
                    return false;
                }
                fieldParam.nThreadTraverse = param.nThread;
                fieldParam.extraObstacles = extraObstacles;
                vsfParamPath.push_back(fieldParam);
            }

            // 2. Construct VSF
            mutex_.lock();
            state.edtServerPtr->getLocker().lock();
            for (int n = 0 ; n < N; n++) {
                state.vsf_path[n].setup(state.edtServerPtr, param.nTarget); // update edf information
                isOk = isOk and state.vsf_path[n].compute(state.targetSetPath[n].points,vsfParamPath[n]);
            }
            state.edtServerPtr->getLocker().unlock();

            // Upload visualization
             if (isOk) {
                 for (int n = 0; n < N; n++) {
                     float meanHeight = 0;
                     for (int m = 0; m < M; m++) {
                         meanHeight += state.targetSetPath[n].points[m].z;
                     }
                     meanHeight /= float(M);

                     // set slice level as the mean of height of the targets
                     visSet.vsfPclPath[n] = (state.vsf_path[n].getFieldIntensity(meanHeight, param.occlusionEps));
                     visSet.vsfPclPath[n].header.frame_id = param.worldFrameId;
                 }

                 // report
                 for (int n =0; n < state.nLastQueryStep ; n++)
                     state.vsf_path[n].report();

             }else
                 ROS_ERROR("Preplanner: vsf chain computation failed. Not updating visualization.");
            state.vsfSuccess = isOk;
            mutex_.unlock();
            return isOk;
        }

        bool Preplanner::inSession() {
            if ( not createVsfPath() ) {
                ROS_ERROR("Preplanner: vsf creation failed");
                return false;
            }

            if ( not planGraph()) {
                ROS_ERROR("Preplanner: graph planning failed");
                return false;
            }
            return true;
        }

        bool Preplanner::plan(const PlanningInput &planningInput, PlanningOutput &planningOutput) {
            if (initSession(planningInput)) {
                if (inSession()){
                    endSession(planningOutput) ;
                    return true;
                }else
                    return false;
            }
            else
                return false;
        }

        /**
         * Timer callback of this object = publish only
         * @param event
         */
        void Preplanner::asyncTimerCallback(const ros::TimerEvent &event) {
            if(mutex_.try_lock()) {
                ros::Time curTime = ros::Time::now();
                // result of init session (e.g prediction )
                if (state.isInitSession) {
                    for (auto & marker: visSet.markerTargetCollisionArray.markers )
                        marker.header.stamp = curTime;
                    pubSet.targetCollisionVolume.publish(visSet.markerTargetCollisionArray);

                    for (int m = 0; m < param.nTarget; m++) {
                        visSet.pathTargetPath[m].header.stamp = curTime;
                        pubSet.targetPathSet[m].publish(visSet.pathTargetPath[m]);
                    }
                }

                // result of vsf path
                if (state.vsfSuccess){
                    for (int n = 0 ; n< state.nLastQueryStep ; n++){
                        visSet.vsfPclPath[n].header.stamp = pcl_conversions::toPCL(curTime);
                        pubSet.vsfPclPath[n].publish(visSet.vsfPclPath [n]);
                    }
                }

                // result of graph construction
                if (state.graphConstructionSuccess){
                    for (int n = 0 ; n< reporter.N ; n++){
                        visSet.candidateNodesPath[n].header.stamp = pcl_conversions::toPCL(curTime);
                        pubSet.candidNodesPath[n].publish(visSet.candidateNodesPath [n]);
                    }
                }

                // result of graph solution
                if (state.graphSolveSuccess){
                    visSet.graphSolution.header.stamp = curTime;
                    pubSet.graphSolution.publish(visSet.graphSolution);

                    for (auto& marker: visSet.bearingMarker.markers)
                        marker.header.stamp = curTime;
                   pubSet.bearingMarker.publish(visSet.bearingMarker);
                }
                mutex_.unlock();
            }else
                ROS_DEBUG("Preplanner: publish locked by plan thread");
        }

        void Preplanner::insertNodes(const MatrixXf & previousNodeBlock, int n ,
                                        vector<Point> triedPoints,pair<int,int> pntIndexRange,
                                        Param tryParam,
                                         vector<NodeExtensionResult>& nodeExtArr,
                                         LayerExtensionResult& resultOut) {
            Timer timer;
            LayerExtensionResult result;
            result.nTriedConnectionNode = triedPoints.size();

            int nPreviousNodes = previousNodeBlock.cols();
            nodeExtArr.clear();

            float dt = state.timeKnotLocal[1] - state.timeKnotLocal[0] ; // time interval time knots (uniform assumed)
            int M = param.nTarget;
            Point prev_node_pnt,cur_node_pnt;

            int nNodes = triedPoints.size();

            for (int i = 0 ; i < nNodes ; i++ ){ // index in current point set
                NodeExtensionResult nodeExt;
                nodeExt.pntIdx = pntIndexRange.first+i;
                nodeExt.n = n;

                for(int prev_node_idx = 0 ; prev_node_idx < nPreviousNodes; prev_node_idx++){
                    prev_node_pnt = Point(previousNodeBlock.block(2,prev_node_idx,3,1));
                    cur_node_pnt = triedPoints[i];

                    // Cost
                    float dist = prev_node_pnt.distTo(cur_node_pnt);
                    // here, target set path = n th future = [n-1]
                    float bearing =  bearingAngle(state.targetSetPath[n-1], triedPoints[i]);
                    float desRelDistDeviation = 0;
                    for (auto pnt : state.targetSetPath[n-1].points){
                        desRelDistDeviation += pow(pnt.distTo(cur_node_pnt) - tryParam.desShotDist,2);
                    }

                    // Edge distance violation
                    LineSegment chaserMoveLine(((cur_node_pnt)),(prev_node_pnt));

                    // Collision with target
                    LineSegment targetMoveLine;
                    bool isTargetCollision = false;
                    for(int m =0 ; m < M ; m++) {
                        targetMoveLine.p2 = state.targetSetPath[n-1].points[m];
                        if (n == 1)
                            targetMoveLine.p1 = state.curTargets.points[m]; // current target position
                        else
                            targetMoveLine.p1 = state.targetSetPath[n-2].points[m];

                        if (TC_collision(targetMoveLine,chaserMoveLine) ) {
                            isTargetCollision = true;
                            break;
                        }
                    }

                    if (not isTargetCollision) {
                        bool isDistanceViolation = dist / float(dt) >  (n > 1 ? tryParam.maxConnectVelocity : 1.3 * tryParam.maxConnectVelocity);
                        if (not isDistanceViolation) { // maximally connectable ?
                            // Is End point occluded?
                            float visScore = state.vsf_path[n - 1].eval(cur_node_pnt);
                            bool isEndPointOccluded =  visScore < param.occlusionEps;
                            result.nRejectEndPointOcclusion += (int) (isEndPointOccluded);

                            if (not isEndPointOccluded) {
                                // Is traverse collision?

                                // Exception handling, we effort to clear the octomap around the chaser
                                Point collisionRayStart;
                                if (n == 1){
                                    Point dir = (cur_node_pnt - prev_node_pnt) /  (cur_node_pnt - prev_node_pnt).norm();
                                    collisionRayStart = prev_node_pnt + dir * tryParam.seedClearRad;
                                }else
                                    collisionRayStart = prev_node_pnt;

                                bool isEdgeCollision = collisionRay(state.edtServerPtr, collisionRayStart, cur_node_pnt,tryParam.collisionRayStride,
                                                                    tryParam.collisionEps);
                                result.nRejectEdgesTraverseObstacleCollision+= (int) isEdgeCollision;

                                if (not isEdgeCollision) {
                                    bool isBearingViolated =false;
                                    if (param.nTarget > 1){
                                       isBearingViolated =  bearing > tryParam.maxBearing;
                                    }
                                    result.nRejectEndPointBearingViolation += (int) isBearingViolated;
                                    if (not isBearingViolated) {

                                        // register edge
                                        Edge edge;
                                        edge.u = previousNodeBlock(5,prev_node_idx);

                                        // 3. weight computation
                                        Point vel = state.chaserInitVel;
                                        Point dir = cur_node_pnt-prev_node_pnt;
                                        float angle = 0 ;
                                        if (n==1 and (vel.norm() > 0)) { // velocity direction
                                            angle = dir.angleTo(vel);
                                        }
                                        edge.w = tryParam.w_dist * pow(dist,1) + tryParam.w_visibility * 1/(visScore) +
                                                tryParam.w_bearing * bearing + tryParam.w_des_rel_dist * desRelDistDeviation +
                                                tryParam.w_init_vel_dir*angle;
                                        nodeExt.connectedEdges.push_back(edge);
                                        result.nEdgesFromPrevious++;
                                    }
                                }
                            }
                        }
                    }
                }

                // if this node has connection, + node
                if (not nodeExt.connectedEdges.empty()) {
                    nodeExtArr.push_back(nodeExt);
                    result.nFeasibleAndConnectedNodes++;
                }
            }
            result.isSuccess = result.nEdgesFromPrevious > 0;
            resultOut = result;
//            ROS_INFO("inner loop took %f",timer.stop());
        }

        void Report::push_back(LayerExtensionResult result) {

            nRejectEdgesTargetCollision.push_back(result.nRejectEdgesTargetCollision); //!
            nRejectEdgesDistanceAllowable.push_back(result.nRejectEdgesDistanceAllowable);
            nRejectEdgesTraverseObstacleCollision.push_back(result.nRejectEdgesTraverseObstacleCollision);
            nRejectEndPointOcclusion.push_back(result.nRejectEndPointOcclusion);
            nRejectEndPointBearingViolation.push_back(result.nRejectEndPointBearingViolation);
            nEdgesFromPrevious.push_back(result.nEdgesFromPrevious);
            nFeasibleAndConnectedNodes.push_back(result.nFeasibleAndConnectedNodes);
            nTriedConnectionNode.push_back(result.nTriedConnectionNode);
            N++;
        }


        bool Preplanner::planGraph() {
            /**
             * Initialize graph
             */
            Eigen::Vector3f chaserPnt = state.chaserInit.toEigen();
            // get param for graph  ...
            int N = state.nLastQueryStep;  // future step
            int M = param.nTarget; // target number

            // initialize graph (e.g. pre-allocation)
            int nMaxNodes = 1;
            int nMaxEdges = 0;
            int prevMaxNodes = 1;

            // determine candidate points
//            vector<vector<EllipsoidNoRot>> hollowPath;
            vector<PointSet> candidateNodesPath;

            for (int n = 0 ; n < N ; n++){

                vector<EllipsoidNoRot> hollow; // region to be removed
                for (int m =0; m < M ; m++)
                    hollow.emplace_back(state.futureTargetPoints[m].traj.points[n].toEigen().cast<double>(),
                                        param.TC_ellipsoidScaleCollisionStep);
                hollow.emplace_back(state.targetSetPath[n].center().toEigen().cast<double>(),
                                    param.TC_ellipsoidScaleCollisionStep);
//                hollowPath.push_back(hollow);

                // point sample
                PointSet candidateNodes;
                state.vsf_path[n].getPnt(param.graphNodeStride ,hollow,
                                         state.targetSetPath[n].center(), state.dronePosition, param.maxAngleDeviationFromSensor
                                         ,candidateNodes);
                candidateNodesPath.push_back(candidateNodes);

                nMaxNodes += candidateNodes.size();
                nMaxEdges += prevMaxNodes*nMaxNodes;
                prevMaxNodes = nMaxNodes;

            }

            Timer tGraphGen;
            reporter.init();
            bool isGraphConnect = true;
            ChaserGraph chaserGraph;
            ROS_INFO("graph allocation : (V=%d,E =%d)" , 5* nMaxNodes, 6*nMaxEdges);
            chaserGraph.edges = Eigen::MatrixXf(5,nMaxEdges);
            chaserGraph.nodes = Eigen::MatrixXf(6,nMaxNodes);
            chaserGraph.edge_div_location = new int[N]; chaserGraph.edge_div_location[0] = 0;
            chaserGraph.node_div_location = new int[N+1]; chaserGraph.node_div_location[0] = 0; chaserGraph.node_div_location[1] = 1;
            chaserGraph.N = N;

            // fill the seed element (current chaser location)
            int N_cur_layer_pnt = 1; int N_cur_layer_edge = 0;
            int node_insert_idx = 0; int edge_insert_idx = 0;

            chaserGraph.nodes(5,node_insert_idx) = 0; // node index
            chaserGraph.nodes(0,node_insert_idx) = chaserGraph.nodes(1,node_insert_idx)  = 0; // t/n
            chaserGraph.nodes.block(2,node_insert_idx,3,1) = chaserPnt; // point (x,y,z)
            chaserGraph.node_div_location[1] = chaserGraph.node_div_location[0] + N_cur_layer_pnt;
            node_insert_idx++;

            for (int n = 1 ; n <= N ; n++) { // index = future step
                const PointSet& candidateNodes = candidateNodesPath[n-1];
                int nThread = param.nThread;
                int nTotalPoint = candidateNodes.points.size();
                vector<std::thread> workerThread;
                vector<vector<NodeExtensionResult>> nodeExtArrSet(nThread);
                vector<LayerExtensionResult> extResultSet(nThread);
                int increment =ceil(nTotalPoint/float(nThread));
                int startIdx = 0;
                float spawnTime = 0;
                for (int threadIdx = 0; threadIdx < nThread ; threadIdx++){
                    Timer timerSmall;
                    int endIdx = min(startIdx + increment,nTotalPoint);
                    vector<Point> pntSet(candidateNodes.points.begin()+startIdx,
                                         candidateNodes.points.begin()+endIdx);
                    pair<int,int> pntIndexRange = make_pair(startIdx,endIdx);
                    startIdx = endIdx;
                    workerThread.emplace_back(std::thread(&Preplanner::insertNodes,this,
                                                          chaserGraph.nodes.block(
                                                                  0,chaserGraph.node_div_location[n-1],6, chaserGraph.node_div_location[n]-chaserGraph.node_div_location[n-1]),
                                                              n,pntSet,pntIndexRange,param,
                                                              std::ref(nodeExtArrSet[threadIdx]),std::ref(extResultSet[threadIdx])));
                    spawnTime += timerSmall.stop();
                }
                for(int threadIdx =0 ; threadIdx < nThread ; threadIdx++)
                    workerThread[threadIdx].join();
                Timer timerInsertion;
                // combine result
                int nNewNode = 0;
                int nNewEdge = 0;
                LayerExtensionResult extResult;
                for (int threadIdx =0; threadIdx < nThread ; threadIdx++){
                    extResult = extResult + extResultSet[threadIdx];
                    nNewEdge += extResultSet[threadIdx].nEdgesFromPrevious;
                    nNewNode += extResultSet[threadIdx].nFeasibleAndConnectedNodes;

                    // insert nodes of this thread
                    for (int j = 0; j < nodeExtArrSet[threadIdx].size() ; j++){
                        int pntIdx = nodeExtArrSet[threadIdx][j].pntIdx;
                        Point cur_node_pnt = candidateNodes.points[pntIdx];

                        int v = node_insert_idx; // index of this vertex
                        chaserGraph.nodes.coeffRef(0, node_insert_idx) = n;
                        chaserGraph.nodes.coeffRef(1, node_insert_idx) = pntIdx;
                        chaserGraph.nodes.coeffRef(2,node_insert_idx) = cur_node_pnt.x;
                        chaserGraph.nodes.coeffRef(3,node_insert_idx) = cur_node_pnt.y;
                        chaserGraph.nodes.coeffRef(4,node_insert_idx) = cur_node_pnt.z;
                        chaserGraph.nodes.coeffRef(5,node_insert_idx) =  v;

                        node_insert_idx++;

                        // register edge heading to this vertex
                        vector<Edge> edgeToThisVertex = nodeExtArrSet[threadIdx][j].connectedEdges;
                        for (int k  = 0; k <edgeToThisVertex.size() ; k++  ){
                            int u = edgeToThisVertex[k].u;
                            chaserGraph.edges(0,edge_insert_idx) = n;
                            chaserGraph.edges(1,edge_insert_idx) = u;
                            chaserGraph.edges(2,edge_insert_idx) = v;
                            chaserGraph.edges(3,edge_insert_idx) = edgeToThisVertex[k].w;
                            edge_insert_idx++;
                        }
                    }
                }

                if (nNewEdge> 0) {
                    if (n < chaserGraph.N )
                        chaserGraph.node_div_location[n+1] = chaserGraph.node_div_location[n] + nNewNode;
                    else {
                        chaserGraph.N_node = chaserGraph.node_div_location[n] + nNewNode;
                    }
                    if (n < chaserGraph.N)
                        chaserGraph.edge_div_location[n] = chaserGraph.edge_div_location[n-1] + nNewEdge;
                    else
                        chaserGraph.N_edge = chaserGraph.edge_div_location[n-1] + nNewEdge;
                }
//                ROS_INFO("timer for insertion = %f",timerInsertion.stop());

                reporter.push_back(extResult);
                if (not extResult.isSuccess){
                    ROS_WARN_STREAM("Preplanner: " <<": Layer was not connected from " << n-1 << " to " << n << ". Aborting preplanning.");
                    isGraphConnect = false;
                    break;
                }
            }

            state.graphSolveSuccess = false;
            if (isGraphConnect) {
                Timer timerSolve;
                state.graphSolveSuccess = chaserGraph.solve();
                double elapse = timerSolve.stop();
                reporter.elapseSolve = elapse;
            }

            /**
             * Update state and vis
             */
            mutex_.lock();
            visSet.candidateNodesPath.clear();
            reporter.elapseConstruction = tGraphGen.stop();
            visSet.candidateNodesPath = chaserGraph.getPCLPath(param.worldFrameId,reporter.N);
            state.graphConstructionSuccess = isGraphConnect;

            if (state.graphSolveSuccess){
                // waypoints
                Path solution;
                for (int n = 0; n <= state.nLastQueryStep; n++) {
                    int idx = chaserGraph.optimal_node_idx_seq(n);
                    Eigen::Vector3f curNode = chaserGraph.nodes.block(2,idx,3,1);
                    solution.append(Point(curNode));
                }
                state.solutionPath = solution;
                visSet.graphSolution = solution.toNavPath(param.worldFrameId);

                // bearing vector
                visSet.bearingMarker.markers.clear();
                for (int m =0 ;m < param.nTarget ; m++){
                    Path targetPathWithInitial;
                    targetPathWithInitial.points.push_back(state.curTargets.points[m]);
                    for(int n = 0 ; n < state.nLastQueryStep ; n++){
                        targetPathWithInitial.points.push_back(state.futureTargetPoints[m].traj.points[n]);
                    }
                    visualization_msgs::MarkerArray curBearing =
                            solution.get_bearing(visSet.bearingBase, targetPathWithInitial);

                    for (int n = 0; n <= state.nLastQueryStep ; n++){
                        auto marker = curBearing.markers[n];
                        marker.id = n;
                        marker.header.frame_id = param.worldFrameId;
                        marker.ns = "target_" + to_string(m);
                        marker.pose.orientation.w = 1.0;
                        visSet.bearingMarker.markers.push_back(marker);
                    }

                    // clearing
                    for (int n = state.nLastQueryStep + 1; n <=param.nMaxStep ; n++ ){
                        visualization_msgs::Marker eraser;
                        eraser.pose.orientation.w= 1.0;
                        eraser.header.frame_id = param.worldFrameId;
                        eraser.type = visualization_msgs::Marker::DELETE;
                        eraser.ns = "target_" + to_string(m);
                        eraser.id = n;
                        visSet.bearingMarker.markers.push_back(eraser);
                    }
                }
            }

            mutex_.unlock();
            reporter.report(); // report current graph connection status
            return isGraphConnect;
        }

        void Preplanner::endSession(PlanningOutput &planningOutput) {

            if (state.graphSolveSuccess){
                planningOutput.waypoints.traj.points = state.solutionPath.points;
                planningOutput.waypoints.traj.ts = state.timeKnotLocal;
                planningOutput.waypoints.refTime = state.tLastTrigger;
                planningOutput.report = reporter;
                planningOutput.targetPoints = state.totalTargetPoints;

                // corridor generation

            }else{
                ROS_ERROR("Preplanner: no graph solution exist but endSession requested");
            }
        }



    } // ns: preplanner
} // ns: dual_chaser


