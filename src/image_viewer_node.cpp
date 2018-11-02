#include "image_viewer.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_viewer");
  image_viewer OBJECT;
  
  ros::spin();
  return 0;
}
