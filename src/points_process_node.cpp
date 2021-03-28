#include "points_process_core.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "points_process");

    ros::NodeHandle nh("~");
    
    omp_set_num_threads(4);

    Processor pro(nh);
    
    return 0;
}

