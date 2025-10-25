#include "../include/clear.hpp"
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "clearit");
    pclfilter::clear clear_cloud;
    int way = 2;
    if (way == 1)
    {
        clear_cloud.init_odom();
    }
    else if (way == 2)
    {
        clear_cloud.init_basemap();
    }
    else
    {
        clear_cloud.init_mapwithodom();
    }
    ros::spin();
    return 0;
}
