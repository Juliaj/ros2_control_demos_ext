Ackermann Bot Demo
==================

This demo is to investigate the idea of using unpowered wheels for odometry calculation, see details in `ros-controls/ros2_controllers#1109 <https://github.com/ros-controls/ros2_controllers/issues/1109>`_.
The issue with the ackermann bot is that the wheels are powered and the slip is not taken into account.

Run the demo
-----------

Start the bot with the following command:

.. code-block:: bash

    ros2 launch ros2_control_demo_ext_example_1 gazebo_default.launch.py


You should see output similar to the following:

.. code-block:: bash

    [INFO] [1719021501.931733100] [gazebo_ros]: Connecting to gazebo
    [INFO] [1719021501.931753000] [gazebo_ros]: Connected to gazebo[python3-1] [INFO] [1745381327.499124865] [ackermannbot_demo_helper_node]: Starting timed run for 120.0 seconds
    [python3-1] [INFO] [1745381329.499933614] [ackermannbot_demo_helper_node]: Starting movement with linear=1.5, angular=0.8
    [python3-1] [INFO] [1745381329.500910309] [ackermannbot_demo_helper_node]: Published initial movement command: linear=1.5, angular=0.8
    [python3-1] [INFO] [1745381329.502232074] [ackermannbot_demo_helper_node]: Initial position set: x=0.00, y=0.00
    [python3-1] [INFO] [1745381330.275318283] [ackermannbot_demo_helper_node]: Distances - Straight: 1.05m, Arc: 1.05m
    [python3-1] [INFO] [1745381331.593000963] [ackermannbot_demo_helper_node]: Distances - Straight: 3.00m, Arc: 3.00m
    [python3-1] [INFO] [1745381332.302481949] [ackermannbot_demo_helper_node]: Distances - Straight: 4.05m, Arc: 4.06m
    [python3-1] [INFO] [1745381333.620742027] [ackermannbot_demo_helper_node]: Distances - Straight: 6.00m, Arc: 6.01m
    [python3-1] [INFO] [1745381334.329710452] [ackermannbot_demo_helper_node]: Distances - Straight: 7.05m, Arc: 7.06m
    [python3-1] [INFO] [1745381335.647632012] [ackermannbot_demo_helper_node]: Distances - Straight: 9.00m, Arc: 9.01m
    [python3-1] [INFO] [1745381336.357239405] [ackermannbot_demo_helper_node]: Distances - Straight: 10.05m, Arc: 10.06m

If there is any slip detected, you may see something like the following:

.. code-block:: bash

    [python3-1] [INFO] [1745381499.039564950] [ackermannbot_demo_helper_node]: Wheel spin slip detected! Ratio: x.xx (wheels moved x.xx m, robot moved x.xx m)

The slip ratio is greater than 1.0, which means that the wheels have spun more than the robot has moved.

To drive the bot on regular ground, use the following command:

.. code-block:: bash

    ros2 launch ros2_control_demo_ext_example_1 demo_test.launch.py ground_mu:=0.8 ground_mu2:=0.8 ground_slip1:=0.05 ground_slip2:=0.05

To drive the bot on slippery ground, use the following command:

.. code-block:: bash

    ros2 launch ros2_control_demo_ext_example_1 demo_test.launch.py ground_mu:=0.4 ground_mu2:=0.4 ground_slip1:=0.3 ground_slip2:=0.3


To drive the bot on very slippery ground, use the following command:

.. code-block:: bash

    ros2 launch ros2_control_demo_ext_example_1 demo_test.launch.py ground_mu:=0.05 ground_mu2:=0.05 ground_slip1:=0.9 ground_slip2:=0.9




