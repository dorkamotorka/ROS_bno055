# ros_bno055
ROS Driver for IMU BNO055

  git clone --single-branch --branch kinetic-devel https://github.com/tp4348/ros_bno055.git

  cd .. 

  sudo apt-get install -y
  
  sudo apt-get install libi2c-dev # for I2C SMBUS functions

  rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y

  catkin_make

  source devel/setup.bash

  rospack profile
