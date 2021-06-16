//
// Created by jbs on 20. 12. 12..
//

#ifndef MULTI_CHASER_SCOREFIELD_H
#define MULTI_CHASER_SCOREFIELD_H

#include "chasing_utils/Utils.h"

#define N_2D_DIRECTION 12
#define N_3D_DIRECTION 8
#define ERROR_SCORE -99

using namespace chasing_utils;
typedef pcl::PointCloud<pcl::PointXYZI> pclIntensity;

namespace dual_chaser {

    pclIntensity getFieldIntensity(const DynamicEDTOctomap& dynamicEdtOctomap );

    struct FieldParam{
        Point origin;
        float resolution;
        float lx;
        float ly;
        float lz;
        float min_z = -1e+5; // default = no effect
        FieldParam(){};

    };

    struct ScoreFieldParam{
        FieldParam fieldParam;
        int nThreadTraverse = 4;
        vector<EllipsoidNoRot> extraObstacles;

        ScoreFieldParam(){};
        ScoreFieldParam(double resolution,const vector<Point>& targets,double margin_xy,
                        double margin_z_up, double margin_z_down,
                        double minZ,double maxZ,bool & isValidFeild){
            float xmin = numeric_limits<float>::max();
            float ymin = numeric_limits<float>::max();
            float zmin = numeric_limits<float>::max();

            float xmax = -numeric_limits<float>::max();
            float ymax = -numeric_limits<float>::max();
            float zmax = -numeric_limits<float>::max();

            int M = targets.size();
            for(auto pnt : targets){
                xmin = min(pnt.x,xmin);
                ymin = min(pnt.y,ymin);
                zmin = min(pnt.z,zmin);
                xmax = max(pnt.x,xmax);
                ymax = max(pnt.y,ymax);
                zmax = max(pnt.z,zmax);
            }

            fieldParam.origin.x = xmin - margin_xy;
            fieldParam.origin.y = ymin - margin_xy;
            fieldParam.origin.z = max(zmin - margin_z_down,minZ);
            fieldParam.lx = xmax - xmin + 2*margin_xy;
            fieldParam.ly = ymax - ymin + 2*margin_xy;
            fieldParam.lz = min(zmax + margin_z_up -fieldParam.origin.z,maxZ - fieldParam.origin.z );

            if (fieldParam.lx <= 0 or fieldParam.ly <= 0 or fieldParam.lz <= 0 ){
                isValidFeild = false;

            }else
                isValidFeild = true;

            fieldParam.resolution = resolution;
            fieldParam.min_z = minZ;
        }
        void operator=(const ScoreFieldParam& param_){
            fieldParam = param_.fieldParam;
            nThreadTraverse = param_.nThreadTraverse;
            extraObstacles.clear();
            for (auto obst: param_.extraObstacles){
                extraObstacles.push_back(obst);
            }
        }

    };


    visualization_msgs::Marker getEDFVolume(Point chaerInit, const vector<ScoreFieldParam>& param_path,
            octomap::point3d &min_, octomap::point3d & max_);
    class ScoreFieldBase {

    private:
        FieldParam fieldParam;

    protected:
        bool isFieldInit = false;
        int Nx,Ny,Nz;
        float *** data = NULL;

        ind3 pnt2ind(Point pnt) const ;
        Point ind2pnt(ind3 ind) const;

        bool isInRange(int ind_,int dim) const ;
        bool isInRange(ind3 ind) const ;
        bool isInRange(Point pnt) const ;

        ScoreFieldBase(){};
        ScoreFieldBase(FieldParam param);
    public:

        pclIntensity getFieldIntensity(double slice_leveel,double eps = 0) const ;
        virtual void init(FieldParam param);
        float eval(ind3 ind) const {
            if (isInRange(ind) and isFieldInit){
                return data[ind.x][ind.y][ind.z];
            } else {
                return ERROR_SCORE;
            }
        }
        void getPntAndValue (int nStride,PointSet& pnts,vector<float>& vals);
        void getPnt (int nStride,PointSet& pnts);
        void getPnt (int nStride,const vector<EllipsoidNoRot>& hollowing,PointSet& pnts);
        void getPnt (int stride, float lengthMin[3] , float lengthMax[3],const vector<EllipsoidNoRot>& hollowing ,  PointSet& pnts);
        float eval(Point pnt) const {
            return eval(pnt2ind(pnt));
        }
        int size() const {return Nx*Ny*Nz;};
        void getLengths(float& lx, float& ly, float& lz) const {lx = fieldParam.lx , ly = fieldParam.ly ; lz = fieldParam.lz; }
        virtual ~ScoreFieldBase();
    };

    class SafetyScoreField: public ScoreFieldBase{
    private:
        octomap_server::EdtOctomapServer* edf_ptr;

    public:
        void updateEDT();
        SafetyScoreField(octomap_server::EdtOctomapServer* edt_ptr,FieldParam param):
        ScoreFieldBase(param){
                edf_ptr = (edt_ptr);
                updateEDT();
        };


    };

    class VisibilityScoreField : public ScoreFieldBase{

        int dirBorderX[12] = {-1,1,1,-1,0,0,0,0,-1,1,1,-1};
        int dirBorderY[12] = {-1,-1,1,1,-1,1,1,-1,0,0,0,0};
        int dirBorderZ[12] = {0,0,0,0,-1,-1,1,1,-1,-1,1,1};

        // 3d traverse direction
        int dirSectionX[8] = {-1,-1, 1,1, 1,1, -1,-1 };
        int dirSectionY[8] = {-1,-1, -1,-1, 1,1, 1,1 };
        int dirSectionZ[8] = {-1,1, -1,1, -1,1, -1,1 };

        int sweepingDirX[20] = {-1,1,1,-1,0,0,0,0,-1,1,1,-1,   -1,-1, 1,1, 1,1, -1,-1 };
        int sweepingDirY[20] = {-1,-1,1,1,-1,1,1,-1,0,0,0,0,   -1,-1, -1,-1, 1,1, 1,1 };
        int sweepingDirZ[20] = {0,0,0,0,-1,-1,1,1,-1,-1,1,1,   -1,1, -1,1, -1,1, -1,1 };

        struct State{
            Point lastQueriedTargetPoint;
            bool isEDT = false;
            double elapseMem = -1;
            double elapseComp = -1;
            vector<int> notMe;
        };

    private:
        State state;
        ScoreFieldParam scoreFieldParam;
        octomap_server::EdtOctomapServer* edf_ptr;
        void traverse(ind3 targetInd, int startDirIdx , int endDirIdx,bool allowBorder = false);
        ind3 n_go(ind3 dir);

    public:
        VisibilityScoreField() {};
        VisibilityScoreField(octomap_server::EdtOctomapServer* edt_ptr,ScoreFieldParam param):
        ScoreFieldBase(param.fieldParam),scoreFieldParam(param){
            edf_ptr = (edt_ptr);
        };
        VisibilityScoreField(octomap_server::EdtOctomapServer* edt_ptr):edf_ptr(edt_ptr) {state.isEDT = true;};
        void setEDF(octomap_server::EdtOctomapServer* edt_ptr) {edf_ptr = edt_ptr; state.isEDT = true; }
        void init(ScoreFieldParam param);
        bool compute(Point target);
        ~VisibilityScoreField(){ cout << "Destructing vsf ... " << endl;  };
        void report();
    };

    class MultiVisibilityScoreField : public ScoreFieldBase {
        struct State{
            int nLastQueryTarget = 0;
            vector<Point> targetPoint;
            double elapseFusion = -1;
        };
    private:
        State state;
        VisibilityScoreField* vsf_set;
        float getFusion(ind3 ind){
            double score = 1 ;
            for (int n = 0 ; n < state.nLastQueryTarget ; n++){
                score*= vsf_set[n].eval(ind);
            }
            return score;
        }

    public:
        MultiVisibilityScoreField(octomap_server::EdtOctomapServer* edt_ptr,int nMaxTarget) {
            vsf_set = new VisibilityScoreField[nMaxTarget];
            for (int n = 0 ; n < nMaxTarget; n++)
                vsf_set[n].setEDF(edt_ptr);
        };
        MultiVisibilityScoreField() {};
        void setup(octomap_server::EdtOctomapServer * edt_ptr, int nMaxTarget);
        bool compute (vector<Point> targetPoints,ScoreFieldParam param);
        void report ();
    };

}
#endif //MULTI_CHASER_SCOREFIELD_H
