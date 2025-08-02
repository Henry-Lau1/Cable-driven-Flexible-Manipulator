#include <rxb_taskspace_closeloop/rxb_taskspace_closeloop.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "rxb_taskspace_closeloop_node");
    ros::NodeHandle nh;

    RXB_TaskSpace_CloseLoop::SingleSegController SSCtrl(nh);

    while(ros::ok()){
      ;
    }
    ros::shutdown();

    return 0;
}
