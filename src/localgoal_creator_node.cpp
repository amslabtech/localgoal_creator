#include "localgoal_creator/localgoal_creator.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_goal_creator");
    LocalGoalCreator localgoal_creator;
    localgoal_creator.process();
    return 0;
}
