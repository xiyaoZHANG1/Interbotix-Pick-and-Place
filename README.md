# Interbotix-Pick-and-Place
I created a program to help a robot arm (PX100) pick up and move objects automatically. The same code works for both the virtual robot and the real.

# Utilisation
How to Run 

   
   
    # Terminal 1
    source /opt/ros/galactic/setup.bash
    source ~/interbotix_ws/install/setup.bash
    
    # pour robot virtuel
    ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=px100 use_sim:=true use_rviz:=false
    
    # pour robot reel
    ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=px100 use_sim:=false

    # Terminal 2
    cd ~/interbotix_ws
    python3 final.py
    
    
