//
// Created by jbs on 20. 12. 18..
//

#include <dual_chaser/ScoreField.h>

using namespace  dual_chaser;
using namespace std;

visualization_msgs::Marker dual_chaser::getEDFVolume(Point chaserInit, const vector<ScoreFieldParam>& param_path, octomap::point3d &min_, octomap::point3d & max_){
    visualization_msgs::Marker cube;
    float xmin = chaserInit.x;
    float ymin = chaserInit.y;
    float zmin = chaserInit.z;

    float xmax = chaserInit.x;
    float ymax = chaserInit.y;
    float zmax = chaserInit.z;


    float padding = 0.5;
    Point pnt;
    for (auto param: param_path){
        // lower corner
        pnt = param.fieldParam.origin;
        xmin = min(pnt.x,xmin);
        ymin = min(pnt.y,ymin);
        zmin = min(pnt.z,zmin);
        xmax = max(pnt.x,xmax);
        ymax = max(pnt.y,ymax);
        zmax = max(pnt.z,zmax);
        // upper corner
        pnt.x = pnt.x + param.fieldParam.lx;
        pnt.y = pnt.y + param.fieldParam.ly;
        pnt.z = pnt.z + param.fieldParam.lz;
        xmin = min(pnt.x,xmin);
        ymin = min(pnt.y,ymin);
        zmin = min(pnt.z,zmin);
        xmax = max(pnt.x,xmax);
        ymax = max(pnt.y,ymax);
        zmax = max(pnt.z,zmax);
    }
    min_.x() = xmin - padding;
    min_.y() = ymin - padding;
    min_.z() = zmin - padding;
    max_.x() = xmax + padding;
    max_.y() = ymax + padding;
    max_.z() = zmax + padding;

    cube.scale.x = max_.x() - min_.x();
    cube.scale.y = max_.y() - min_.y();
    cube.scale.z = max_.z() - min_.z();

    cube.pose.position.x = (max_.x() + min_.x())/2.0;
    cube.pose.position.y = (max_.y() + min_.y())/2.0;
    cube.pose.position.z = (max_.z() + min_.z())/2.0;

    cube.pose.orientation.w = 1.0;
    return cube;
}

ind3 ScoreFieldBase::pnt2ind(Point pnt) const {
    return ind3(floor((pnt.x-fieldParam.origin.x)/fieldParam.resolution),
                floor((pnt.y-fieldParam.origin.y)/fieldParam.resolution),
                floor((pnt.z-fieldParam.origin.z)/fieldParam.resolution));
}

Point ScoreFieldBase::ind2pnt(ind3 ind ) const {

    assert(isInRange(ind.x,0) and isInRange(ind.y,1) and isInRange(ind.z,2) and "ind2pnt index out of range");
    Point pnt;
    pnt.x = fieldParam.origin.x + fieldParam.resolution * ind.x;
    pnt.y = fieldParam.origin.y + fieldParam.resolution * ind.y;
    pnt.z = fieldParam.origin.z + fieldParam.resolution * ind.z;
    return pnt;
}

bool ScoreFieldBase::isInRange(int ind_, int dim) const  {
    switch (dim) {
        case 0:
            return (ind_ >= 0 and ind_ < Nx);
        case 1:
            return (ind_ >= 0 and ind_ < Ny);
        case 2:
            return (ind_ >= 0 and ind_ < Nz);
        default:
            return -1;
    }
}

bool ScoreFieldBase::isInRange(ind3 ind ) const {
    return (ind.x >= 0 and ind.x < Nx and ind.y >= 0 and ind.y < Ny and ind.z >= 0 and ind.z < Nz );
}

bool ScoreFieldBase::isInRange(Point pnt) const {
    return pnt.x >= fieldParam.origin.x  and pnt.x <= fieldParam.origin.x + fieldParam.lx and
            pnt.y >= fieldParam.origin.y and pnt.y <= fieldParam.origin.y + fieldParam.ly and
            pnt.z >=  fieldParam.origin.z and pnt.z <= fieldParam.origin.z + fieldParam.lz;

}

void ScoreFieldBase::getPntAndValue(int nStride, PointSet& pnts, vector<float>& vals) {
    pnts.points.clear();
    vals.clear();
    if (isFieldInit) {
        for (int nx = 0; nx < Nx ; nx+= nStride)
            for (int ny  = 0 ; ny < Ny ; ny+=nStride)
                for (int nz  = 0 ; nz < Nz ; nz+=nStride){
                    pnts.points.push_back(ind2pnt(ind3(nx,ny,nz)));
                    vals.push_back(data[nx][ny][nz]);
                }
    }
}

void ScoreFieldBase::getPnt(int nStride, PointSet& pnts)  {
    pnts.points.clear();
    int nTotal =(floor((Nx-1)/float(nStride))+1) *
            (floor((Ny-1)/float(nStride))+1)*
            (floor((Nz-1)/float(nStride))+1);
    pnts.points.resize(nTotal);
    int nInsert =0 ;
    if (isFieldInit)
        for (int nx = 0; nx < Nx ; nx+= nStride)
            for (int ny  = 0 ; ny < Ny ; ny+=nStride)
                for (int nz  = 0 ; nz < Nz ; nz+=nStride) {
                    pnts.points[nInsert] = ind2pnt(ind3(nx, ny, nz));
                    nInsert++;
                }
}

/**
 * Sample points outside hollowing ellipsoid
 * @param nStride stride of vsf original resolution
 * @param hollowing
 * @param pnts
 */
void ScoreFieldBase::getPnt(int nStride, const vector<EllipsoidNoRot>& hollowing, PointSet& pnts) {
    pnts.points.clear();
    int nTotal =(floor((Nx-1)/float(nStride))+1) *
                (floor((Ny-1)/float(nStride))+1)*
                (floor((Nz-1)/float(nStride))+1);
    pnts.points.resize(nTotal);
    int nInsert =0 ;
    if (isFieldInit)
        for (int nx = 0; nx < Nx ; nx+= nStride)
            for (int ny  = 0 ; ny < Ny ; ny+=nStride)
                for (int nz  = 0 ; nz < Nz ; nz+=nStride) {
                    Point pnt = ind2pnt(ind3(nx, ny, nz));
                    bool isInclude = false;

                    for (const auto & ellipse : hollowing){
                        if (ellipse.evalDist(pnt) <= 0 ){
                            isInclude = true;
                            break;
                        }
                    }
                    if (not isInclude){
                        pnts.points[nInsert] = pnt ;
                        nInsert++;
                    }
                }
}

/**
 * sample points X s.t angle(X-vantangePoint , sensorPoint-vantangePoint) <= angleDiffMax
 * This function was designed to discrepancy between planned view angle and the current sensoring data
 * @param nStride
 * @param hollowing
 * @param vantangePoint
 * @param sensorPoint
 * @param angleDiffMax
 * @param pnts
 */
void ScoreFieldBase::getPnt(int nStride, const vector<EllipsoidNoRot> &hollowing, Point vantangePoint,
                            Point sensorPoint, float angleDiffMax, PointSet &pnts) {

    pnts.points.clear();
    int nTotal =(floor((Nx-1)/float(nStride))+1) *
                (floor((Ny-1)/float(nStride))+1)*
                (floor((Nz-1)/float(nStride))+1);
    pnts.points.resize(nTotal);
    int nInsert =0 ;
    if (isFieldInit)
        for (int nx = 0; nx < Nx ; nx+= nStride)
            for (int ny  = 0 ; ny < Ny ; ny+=nStride)
                for (int nz  = 0 ; nz < Nz ; nz+=nStride) {
                    Point pnt = ind2pnt(ind3(nx, ny, nz));

                    // outside hollowing ellipse
                    bool isInclude = false;
                    for (const auto & ellipse : hollowing){
                        if (ellipse.evalDist(pnt) <= 0 ){
                            isInclude = true;
                            break;
                        }
                    }
                    if (not isInclude){

                        // angle constraint ?
                        Point v1 =  (pnt-vantangePoint);
                        Point v2 = (sensorPoint-vantangePoint);
                        float angleDiff = acos(v1.dot(v2) / v1.norm() / v2.norm());
                        if (angleDiff < angleDiffMax) {
                            pnts.points[nInsert] = pnt;
                            nInsert++;
                        }
                    }
                }
}






/**
 * point sampler (center - lengthMin) ~ (center + lengthMax) / not origin
 * @param stride
 * @param lengthMin [m] 0<= <= (lx,ly,lz)/2
 * @param lengthMax [m]
 * @param pnts
 */
void ScoreFieldBase::getPnt(int stride, float *lengthMin, float *lengthMax, const vector<EllipsoidNoRot>& hollowing ,  PointSet &pnts) {
    pnts.points.clear();
    Point centerPnt = fieldParam.origin + Point(fieldParam.lx, fieldParam.ly , fieldParam.lz)/2.0;
    ind3 centerCell = pnt2ind(centerPnt);
    int iMin = floor(lengthMin[0] / fieldParam.resolution);
    int jMin = floor(lengthMin[1] / fieldParam.resolution);
    int kMin = floor(lengthMin[2] / fieldParam.resolution);

    int iMax = floor(lengthMax[0] / fieldParam.resolution);
    int jMax = floor(lengthMax[1] / fieldParam.resolution);
    int kMax = floor(lengthMax[2] / fieldParam.resolution);

    for (int i = iMin ; i < iMax ; i+= stride)
        for (int j = jMin ; j < jMax ; j+= stride)
            for (int k = kMin ; k < kMax ; k+= stride) {

                for (int sig1 = 0 ; sig1 < 2 ; sig1++)
                    for (int sig2 = 0 ; sig2 < 2 ; sig2++)
                        for (int sig3 = 0 ; sig3 < 2 ; sig3++) {

                            ind3 currentCell(centerCell.x + i * (sig1 == 0 ? -1 : 1),
                                             centerCell.y + j * (sig2 == 0 ? -1 : 1),
                                             centerCell.z + k * (sig3 == 0 ? -1 : 1));

                            Point pnt = ind2pnt(currentCell);

                            bool isInclude = false;
                            for (const auto & ellipse : hollowing){
                                if (ellipse.evalDist(pnt) <= 0 ){
                                    isInclude = true;
                                    break;
                                }
                            }

                            if (not isInclude)
                                pnts.points.push_back( pnt) ;

                        }

            }
}





pclIntensity ScoreFieldBase::getFieldIntensity(double slice_level,double eps) const {
    pclIntensity pcl;

    if (slice_level >= fieldParam.origin.z and slice_level <= fieldParam.origin.z + fieldParam.lz )    {
       int nz =  floor((slice_level - fieldParam.origin.z)/fieldParam.resolution);
       for (int nx = 0 ; nx <Nx ; nx ++)
           for (int ny = 0 ; ny < Ny ; ny++ ){
               pcl::PointXYZI pnt;
               pnt.x = ind2pnt(ind3(nx,ny,nz)).x;
               pnt.y = ind2pnt(ind3(nx,ny,nz)).y;
               pnt.z = ind2pnt(ind3(nx,ny,nz)).z;
               pnt.intensity = data[nx][ny][nz];


               if (pnt.intensity>eps)
                   pcl.push_back(pnt);
           }

    }else{
        cout << "[ " << typeid(*this).name() << "]" << " required slice level = " << slice_level
        << " range of field = " << fieldParam.origin.z << " ~ "  << fieldParam.origin.z + fieldParam.lz << endl;

    }

    return pcl;

}

ScoreFieldBase::ScoreFieldBase(FieldParam param):fieldParam(param){

        isFieldInit = true;
        Nx = floor(param.lx/param.resolution);
        Ny = floor(param.ly/param.resolution);
        Nz = floor(param.lz/param.resolution);

        data = new float**[Nx];
        for (int x=0;x<Nx;x++){
            data[x] = new float*[Ny];
            for (int y=0;y<Ny;y++)
                data[x][y] = new float[Nz];
        }
}

void ScoreFieldBase::init(FieldParam param) {

    Timer t;
    if (isFieldInit){
        for (int x=0;x<Nx;x++){
            for (int y=0;y<Ny;y++)
                delete[] data[x][y] ;
            delete[] data[x];
        }
        delete[] data;
    }
    fieldParam = param;
    isFieldInit = true;
    Nx = floor(param.lx/param.resolution);
    Ny = floor(param.ly/param.resolution);
    Nz = floor(param.lz/param.resolution);

    data = new float**[Nx];
    for (int x=0;x<Nx;x++){
        data[x] = new float*[Ny];
        for (int y=0;y<Ny;y++)
            data[x][y] = new float[Nz];
    }

}

ScoreFieldBase::~ScoreFieldBase() {
    std::cout << "Destructing field base" << std::endl;

    if (data != NULL){
        for (int x=0;x<Nx;x++){
            for (int y=0;y<Ny;y++)
                delete[] data[x][y] ;
            delete[] data[x];
        }
        delete[] data;
    }else{

        std::cout << "Nothing to destroy" << std::endl;
    }


}

void SafetyScoreField::updateEDT() {

    for (int nx = 0 ; nx <Nx ; nx ++)
        for (int ny = 0 ; ny < Ny ; ny++ )
            for (int nz = 0 ; nz < Nz ; nz++){
                auto pnt = ind2pnt(ind3(nx,ny,nz));
                octomap::point3d pntOcto(pnt.x,pnt.y,pnt.z);
                data[nx][ny][nz] = edf_ptr->getDistance(pntOcto);
            }


}


ind3 VisibilityScoreField::n_go(ind3 dir){
    int dx = dir.x;
    int dy = dir.y;
    int dz = dir.z;
    int lx,ly,lz;
    int iq,jq,kq;
    ind3 targetInd = pnt2ind(state.lastQueriedTargetPoint);
    iq = targetInd.x;
    jq = targetInd.y;
    kq = targetInd.z;

    if (dx == 0){
        lx = 1;
    } else if (dx ==1){
        lx = Nx-iq ;
    } else
        lx = iq+1;

    if (dy == 0){
        ly = 1;
    } else if (dy ==1){
        ly = Ny-jq ;
    } else
        ly = jq+1;

    if (dz == 0){
        lz = 1;
    } else if (dz ==1){
        lz = Nz-kq ;
    } else
        lz = kq+1;

    return ind3(lx,ly,lz);
}
void VisibilityScoreField::init(ScoreFieldParam param) {
    Timer t;
    scoreFieldParam = param;
    ScoreFieldBase::init(param.fieldParam);
    state.elapseMem =t.stop();
}

bool VisibilityScoreField::compute(Point targetPnt) {
    Timer timer;
    // precheck
    if (not isInRange(targetPnt)){
        ROS_ERROR("[VisibilityScoreField] query target [%f, %f, %f] out of bound ([%f, %f, %f] - [%f, %f, %f]) "
               ".. aborting vsf computing\n"  , targetPnt.x,targetPnt.y , targetPnt.z,
               scoreFieldParam.fieldParam.origin.x,
               scoreFieldParam.fieldParam.origin.y,
               scoreFieldParam.fieldParam.origin.z,
               scoreFieldParam.fieldParam.origin.x +scoreFieldParam.fieldParam.lx ,
               scoreFieldParam.fieldParam.origin.y + scoreFieldParam.fieldParam.ly,
               scoreFieldParam.fieldParam.origin.z + scoreFieldParam.fieldParam.lz
               );
        return false;
    }

    // update state
    ind3 targetInd = pnt2ind(targetPnt);
    state.lastQueriedTargetPoint = targetPnt;
    state.notMe.clear();
    int i = 0 ;
    for (auto ellispoid : scoreFieldParam.extraObstacles){
        if (ellispoid.evalDist(targetPnt ) > 0){
            state.notMe.push_back(i);
        }
        i++;
    }

    // reset
    for (int i = 0; i < Nx ; i ++)
        for (int j = 0; j < Ny ; j ++)
            for (int k = 0; k < Nz ; k ++)
                data[i][j][k] = 0;
    data[targetInd.x][targetInd.y][targetInd.z] =
            edf_ptr->getDistance(octomap::point3d(targetPnt.x,targetPnt.y,targetPnt.z));

    // single-thread 2d traverse
    traverse(targetInd,0,N_2D_DIRECTION,true);

    // multi-thread 3d section traverse
    vector<std::thread> workerThread(scoreFieldParam.nThreadTraverse);
    int startDirIdx = N_2D_DIRECTION;
    int increment =ceil(N_3D_DIRECTION/float(scoreFieldParam.nThreadTraverse));
    for (int i =0 ; i < scoreFieldParam.nThreadTraverse ; i++){
        int endDirIdx = min(startDirIdx + increment,N_2D_DIRECTION+N_3D_DIRECTION);
            workerThread[i] = thread(&VisibilityScoreField::traverse,this,
                targetInd,startDirIdx,endDirIdx,false);
        startDirIdx = endDirIdx;
    }

    for(int i =0 ; i < scoreFieldParam.nThreadTraverse ; i++){
        workerThread[i].join();
    }
    state.elapseComp = timer.stop();
    return true;
}

void VisibilityScoreField::traverse(ind3 targetInd,
        int startDirIdx, int endDirIdx,bool allowBorder) {

//    printf ("vsf: traversing from %d to %d direction...\n" ,startDirIdx,endDirIdx);
    int offset = (int)(not allowBorder);
    int i,j,k;
    int iq,jq,kq;
    iq = targetInd.x;
    jq = targetInd.y;
    kq = targetInd.z;

    int neighborX[2] = {0,0};
    int neighborY[2] = {0,0};
    int neighborZ[2] = {0,0};

    for (int d = startDirIdx ; d< endDirIdx ; d++){
        ind3 dir (sweepingDirX[d],sweepingDirY[d],sweepingDirZ[d]);
        ind3 dirGo = n_go(dir);
        int lx = dirGo.x, dx = dir.x;
        int ly = dirGo.y, dy = dir.y;
        int lz = dirGo.z, dz = dir.z;
        int insertIdx = 0 ;
        for (int ix = offset ; ix < lx ; ix++)
            for (int iy = offset ; iy < ly ; iy++)
                for (int iz = offset ; iz < lz ; iz++){
                    if (insertIdx> -int(not allowBorder)){
                        // index
                        i = iq+dx*(ix);
                        j = jq+dy*(iy);
                        k = kq+dz*(iz);

                        // coordinate
                        auto pnt = ind2pnt(ind3(i,j,k));
                        octomap::point3d curPnt(pnt.x,pnt.y,pnt.z);

                        Eigen::Vector3f toTarget(iq-i,jq-j,kq-k);
                        toTarget.normalize();
                        // index (i,j,k) directions combination
                        neighborX[1] = (int) sign(iq-i);
                        neighborY[1] = (int) sign(jq-j);
                        neighborZ[1] = (int) sign(kq-k);


                        // get score of neighbors on raycast end
                        int neighborInd = 0 ;
                        float RayEndVisScore = 0;
                        float weight;
                        float weightSum = 0;
                        for (int ni = 0 ; ni <= abs(neighborX[1]); ni ++)
                            for (int nj = 0 ; nj <= abs(neighborY[1]); nj ++)
                                for (int nk = 0 ; nk <= abs(neighborZ[1]); nk ++){
                                    if (neighborInd > 0 ){
                                        Eigen::Vector3i toNeighbor(neighborX[ni],neighborY[nj],neighborZ[nk] );
                                        weight = toTarget.transpose()*(toNeighbor.cast<float>()/toNeighbor.norm());
                                        RayEndVisScore += weight * data[i + neighborX[ni]][j+neighborY[nj]][k+neighborZ[nk]];
                                        weightSum += weight;
                                    }
                                    neighborInd++;
                                }
                        // consider other obj for occlusion
                        double distFromOthers = numeric_limits<double>::max() ;
                        for (auto idx : state.notMe){
                            distFromOthers = min (distFromOthers,scoreFieldParam.extraObstacles[idx].evalDist(pnt));
                        }
                        RayEndVisScore/=weightSum; double visScore = min(RayEndVisScore,min(edf_ptr->getDistance(curPnt),(float)distFromOthers));
                        data[i][j][k] = visScore;
                    }
                    insertIdx++;
                }
    }
//    printf ("vsf: finished traversing from %d to %d direction\n" ,startDirIdx,endDirIdx);


}
void VisibilityScoreField::report() {
    Point origin = scoreFieldParam.fieldParam.origin;
    Point upperCorner = origin;
    upperCorner.x += scoreFieldParam.fieldParam.lx;
    upperCorner.y += scoreFieldParam.fieldParam.ly;
    upperCorner.z += scoreFieldParam.fieldParam.lz;

    printf("vsf : [Nx,Ny,Nx] = [%d, %d, %d], lower corner = [%f, %f, %f],  upper corner = [%f, %f, %f] | tMem = %f | tComp = %f\n",
            Nx,Ny,Nz,origin.x,origin.y,origin.z,
            upperCorner.x,upperCorner.y,upperCorner.z
            ,state.elapseMem,state.elapseComp);
}

bool MultiVisibilityScoreField::compute(vector<Point> targetPoints, ScoreFieldParam param) {
    bool isOk = true;

    int Ntarget = targetPoints.size();

    ScoreFieldBase::init(param.fieldParam);

    // update state
    state.nLastQueryTarget = Ntarget;
    state.targetPoint = targetPoints;


    // compute subfield
    for (int n = 0 ; n < Ntarget ; n++) {
        vsf_set[n].init(param);
        isOk = isOk and vsf_set[n].compute(targetPoints[n]);
    }
    Timer t;
    // update data
    for (int nx = 0 ; nx < Nx ; nx++)
        for (int ny = 0 ; ny < Ny ; ny++)
            for (int nz = 0 ; nz < Nz ; nz++){
                data[nx][ny][nz] = getFusion(ind3(nx,ny,nz));
            }
    // report
    state.elapseFusion = t.stop();
    return isOk;
}

void MultiVisibilityScoreField::setup(octomap_server::EdtOctomapServer *edt_ptr, int nMaxTarget) {
    vsf_set = new VisibilityScoreField[nMaxTarget];
    for (int n = 0 ; n < nMaxTarget; n++)
        vsf_set[n].setEDF(edt_ptr);
}

void MultiVisibilityScoreField::report(){
    cout << "=============MULTI VSF REPORT===============" << endl;
    for (int m= 0 ; m < state.nLastQueryTarget ; m++){
        vsf_set[m].report();
    }
    printf("elapseFusion = %f\n",state.elapseFusion);
}

