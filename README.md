ROS MPU6050 Node
================

Publishes IMU DMP sensor data from an MPU6050 connected to an I2C bus on a Raspberry Pi.

Installation
------------

First install [I2Cdevlib](https://github.com/jrowberg/i2cdevlib):

    sudo mkdir -p /usr/share/arduino/libraries
    cd /usr/share/arduino/libraries
    sudo git clone https://github.com/chrisspen/i2cdevlib.git

Note the fork should be used for now, since jrowberg's i2cdevlib has several outstanding bugs and is unmaintained.

Then install [Bcm2835](http://www.airspayce.com/mikem/bcm2835/index.html):

    cd /tmp
    wget http://www.airspayce.com/mikem/bcm2835/bcm2835-1.50.tar.gz
    tar zxvf bcm2835-1.50.tar.gz
    cd bcm2835-1.50
    ./configure
    make
    make check
    sudo make install
    
Then clone the project into your ROS workspace via:

    git clone https://github.com/chrisspen/ros_mpu6050_node.git
    
And then compile it:

    catkin_make --pkg ros_mpu6050_node

Usage
-----

Since the Raspbian kernel and BCM2835 driver restrict I2C access to only the root user, you must launch the node as root like:

    sudo bash -c "source /your/ros/path/setup.bash; roslaunch ros_mpu6050_node mpu6050.launch"

Assuming the device is properly wired, it should report "DMP ready!". Now you should be able to see the IMU stream with:

    rostopic echo /imu/data

Similar Projects
----------------

    https://github.com/brNX/ros_mpu6050_node
    
    https://github.com/matpalm/ros-mpu6050-node
