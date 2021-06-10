//
// Created by jbs on 21. 6. 9..
//
#include <dual_chaser/Preplanner.h>

namespace dual_chaser{
    namespace preplanner{
        Preplanner::Preplanner(octomap_server::EdtOctomapServer* edtServerPtr) :nh("~") {
            state.edtServerPtr = edtServerPtr;
            initROS();
            asyncSpinnerPtr->start();
        }
        void Preplanner::initROS() {
            /**
             * 1. Parameter parsing
             */
            nh.param("preplanner/n_max_step",param.nMaxStep,4);
            nh.param("preplanner/max_bearing",param.maxBearing,float(M_PI/2.0));
            nh.param("preplanner/score_field_resolution",param.resolution,float(0.3));
            nh.param("preplanner/score_field_node_stride",param.graphNodeStride,3);
            nh.param("preplanner/score_field_margin_xy",param.margin_xy,float(4.0));
            nh.param("preplanner/score_field_margin_z",param.margin_z,float(2.5));
            nh.param("preplanner/min_height",param.minZ,-10000.0f);
            nh.param("preplanner/max_height",param.maxZ,10000.0f);
            nh.param("preplanner/des_dist",param.desShotDist,float(3.5));
            nh.param("preplanner/score_field_max_connect",param.maxDist,float(4.5));
            nh.param("preplanner/n_thread",param.nThread,6);

            nh.param("preplanner/weight/distance",param.w_dist,float(1.0));
            nh.param("preplanner/collision_clearance",param.collisionEps,float(0.5));
            nh.param("preplanner/target_collision_clearance",param.targetCollisionEps,float(0.8));
            nh.param("preplanner/occlusion_clearance",param.occlusionEps,float(0.1));
            nh.param("preplanner/weight/bearing",param.w_bearing,float(0.3));
            nh.param("preplanner/weight/rel_distance",param.w_des_rel_dist,float(0.5));
            nh.param("preplanner/weight/visibility",param.w_visibility,float(1.0));
            nh.param("preplanner/weight/init_vel_dir",param.w_init_vel_dir,float(0.5));
            nh.param("preplanner/target_occlusion_ellipse/x",param.ellipsoidScale(0),double(0.5));
            nh.param("preplanner/target_occlusion_ellipse/y",param.ellipsoidScale(1),double(0.5));
            nh.param("preplanner/target_occlusion_ellipse/z",param.ellipsoidScale(2),double(0.5));

            nh.param("visualization/target_collision_volume/r",param.targetCollisionVolumeColor.r,float(0.2));
            nh.param("visualization/target_collision_volume/g",param.targetCollisionVolumeColor.g,float(0.2));
            nh.param("visualization/target_collision_volume/b",param.targetCollisionVolumeColor.b,float(0.8));
            nh.param("visualization/target_collision_volume/a",param.targetCollisionVolumeColor.a,float(0.7));


            /**
             * 2. Communication register
             */

            string topicPrefix = "preplanner";
            pubSet.targetCollisionVolume = nh.advertise<visualization_msgs::Marker>(topicPrefix + "/target_collision_volume",1);

            for(int m = 0 ; m < param.nTarget; m++) {
                string topicPrefixTarget = topicPrefix + "/target_" + to_string(m) ;
                ros::Publisher publisher = nh.advertise<nav_msgs::Path>(topicPrefixTarget + "/target_path" , 1);
                pubSet.targetPathSet.push_back(publisher);
            }

            for (int n = 0 ; n < param.nMaxStep ; n++) {
                string topicPrefixStep = topicPrefix + "/step_" + to_string(n) ;
                ros::Publisher publisher = nh.advertise<pclIntensity>(topicPrefixStep + "/vsf_path" + to_string(n), 1);
                pubSet.vsfPclPath.push_back(publisher);
            }

            callbackQueue.clear();
            nhTimer.setCallbackQueue(&callbackQueue);
            timerCaller  = nhTimer.createTimer(ros::Duration(0.05), &Preplanner::asyncTimerCallback, this);
            asyncSpinnerPtr = new ros::AsyncSpinner(1, &callbackQueue);



            /**
             * 3. visualizer init
             */
            visSet.markerTargetCollision.header.frame_id = param.worldFrameId;
            visSet.markerTargetCollision.type = visualization_msgs::Marker::SPHERE_LIST;
            visSet.markerTargetCollision.scale.x = 2*param.targetCollisionEps;
            visSet.markerTargetCollision.scale.y = 2*param.targetCollisionEps;
            visSet.markerTargetCollision.scale.z = 2*param.targetCollisionEps;
            visSet.markerTargetCollision.color = param.targetCollisionVolumeColor;
            visSet.markerTargetCollision.pose.orientation.w = 1.0;
            visSet.markerTargetCollision.points.resize(param.nTarget);
            visSet.pathTargetPath.resize(param.nTarget);

            /**
             * 4. State initialization
             */
            state.vsf_path = new MultiVisibilityScoreField[param.nMaxStep];
            visSet.vsfPclPath.resize(param.nMaxStep);


        }

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

            vector<TrajStamped> futureTargetPoints;
            for (int m = 0; m < nTarget; m++) {
                const PredictionOutput &prediction = planningInput.predictionOutput[m];
                TrajStamped traj; // (X1,X2,...,XN)
                traj.refTime = triggerTime;
                for (int n = 0; n < N; n++) {
                    float tLocal = (n + 1) * dt;
                    ros::Time tGlobal = traj.refTime + ros::Duration(tLocal);
                    traj.traj.ts.push_back(tLocal);
                    traj.traj.points.push_back(prediction.eval(tGlobal));
                }
                futureTargetPoints.push_back(traj);
            }

            // 2. Upload state and visualization of current planning inputs (state -> visualizer)
            state.futureTargetPoints = futureTargetPoints;
            state.curTargets = planningInput.targetInit;
            state.isInitSession = true;
            state.nLastQueryStep = N;

            mutex_.lock();
            for (int m = 0; m < nTarget; m++) {
                visSet.markerTargetCollision.points[m] = state.curTargets.points[m].toGeometry();
                visSet.pathTargetPath[m] = state.futureTargetPoints[m].traj.toNavPath(param.worldFrameId);
            }
            mutex_.unlock();

            return true;
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
                                                       param.ellipsoidScale );
                }
                // determine size of field
                ScoreFieldParam fieldParam(param.resolution, state.targetSetPath[n].points,
                                           param.margin_xy, param.margin_z, param.minZ, param.maxZ, isValidField);
                if (not isValidField) {
                    ROS_ERROR("Preplanner: grid field of target %d found invalid. Aborting VSF creation.",n);
                    return false;
                }
                fieldParam.nThreadTraverse = param.nThread;
                fieldParam.extraObstacles = extraObstacles;
                vsfParamPath.push_back(fieldParam);
            }

            // 2. Construct VSF
            state.edtServerPtr->getLocker().lock();
            for (int n = 0 ; n < N; n++) {
                state.vsf_path[n].setup(state.edtServerPtr, param.nTarget); // update edf information
                isOk = isOk and state.vsf_path[n].compute(state.targetSetPath[n].points,vsfParamPath[n]);
            }
            state.edtServerPtr->getLocker().unlock();

            // Upload visualization
             if (isOk) {
                 mutex_.lock();
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
                 mutex_.unlock();

                 // report
                 for (int n =0; n < state.nLastQueryStep ; n++)
                     state.vsf_path[n].report();

             }else
                 ROS_ERROR("Preplanner: vsf chain computation failed. Not updating visualization.");
            state.vsfSuccess = isOk;
            return isOk;
        }

        bool Preplanner::inSession() {
            bool isOk = true;
            isOk = isOk and createVsfPath();

            return isOk;
        }

        bool Preplanner::plan(const PlanningInput &planningInput, PlanningOutput &planningOutput) {
            if (initSession(planningInput)) {
                return inSession();
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

                // product of init session (e.g prediction )
                if (state.isInitSession) {
                    visSet.markerTargetCollision.header.stamp = curTime;
                    pubSet.targetCollisionVolume.publish(visSet.markerTargetCollision);
                    for (int m = 0; m < param.nTarget; m++) {
                        visSet.pathTargetPath[m].header.stamp = curTime;
                        pubSet.targetPathSet[m].publish(visSet.pathTargetPath[m]);
                    }
                }

                // product of vsf path
                if (state.vsfSuccess){
                    for (int n = 0 ; n< state.nLastQueryStep ; n++){
                        visSet.vsfPclPath[n].header.stamp = pcl_conversions::toPCL(curTime);
                        pubSet.vsfPclPath[n].publish(visSet.vsfPclPath [n]);
                    }
                }

                mutex_.unlock();
            }else
                ROS_DEBUG("Preplanner: publish locked by plan thread");
        }

    } // ns: preplanner
} // ns: dual_chaser


