# IR-Project
Intelligent Robotics project - UniPD - 2023/2024


For start the assignment 1 follow these instructions:

- Open a terminal and go to `catkin_ws` folder
- Run the command `catkin build`
- Run the command `roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=robotics_library` on the first terminal
- Open another terminal and run the command `roslaunch tiago_iaslab_simulation navigation.launch`
- Open the third terminal go to `catkin_ws` folder and run `rosrun assignment_1 goal_receiver`
- Open the fourth terminal go again to `catkin_ws` folder and run `rosrun assignment_1 goal_sender`
- Now you can add on the terminal where you run `rosrun assignment_1 goal_sender`:
    - the coordinate point `X` (integer of float are booth accepted)
    - the coordinate point `Y` (integer of float are booth accepted)
    - the orientation (integer of float are booth accepted)
    - Select `y` if you want to see the real time feedback or `n` if you are interested only in the final result.
- To stop the program enter `q` end press `ENTER` on the terminal where you run `rosrun assignment_1 goal_sender`
