1. Terminal:
    roslaunch jackal_gazebo jackal_world.launch config:=front_laser

2. Terminal:
    roslaunch jackal_navigation gmapping.launch

3. Terminal:
    roslaunch jackal_viz view_robot.launch config:=gmapping

4. Terminal:
     roslaunch jackal_navigation move_base.launch


3. Terminal:
    rosrun greedy_pioneer getOccupancyGridMap


4. im RViz Marker setzen!

note: 
muss ge-sourced sein:

    source catkin_ws/devel/setup.bash  

kann nicht permanent sourcen weil im bashrc schon navigation_ ge-sourced ist und das sonst nicht geht.

