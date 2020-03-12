#include "include/getLP.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "getLP");

  ros::NodeHandle nh;

  PclGetLP core(nh);
  return 0;
}
