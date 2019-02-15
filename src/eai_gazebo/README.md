#Eai gazebo package how to use

## Tutoial

How-to-build your world:

      http://gazebosim.org/tutorials?cat=guided_b&tut=guided_b3

## Quick Start

Gazebo:

      roslaunch eai_gazebo eai_world.launch (world_file:=YOUR_OWN_GAZEBO_WORLD.world)
      
Gmapping:

      roslaunch eai_gazebo gmapping_demo.launch

Teleop:

      roslaunch dashgo_tools teleop_twist_keyboard.launch sim:=true

      
publish topics:

      /odom
      /mobile_base/sensors/imu_data
      /scan
      /tf
      /tf_static
      
subscribe topics:

      /cmd_vel_mux/input/navi       # for navigator cmd_vel
      /cmd_vel_mux/input/teleop     # for teleop cmd_vel
