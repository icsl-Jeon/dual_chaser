//
// Created by jbs on 21. 6. 9..
//
#include <dual_chaser/Wrapper.h>

int main(int argc, char** argv){
    ros::init(argc,argv,"dual_chaser");
    dual_chaser::Wrapper chaser;

    chaser.run();

    return 0;
}








