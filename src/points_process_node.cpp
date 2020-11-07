#include "points_process_core.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "points_process");

    ros::NodeHandle nh("~");

    PclTestCore core(nh);
    return 0;
}

