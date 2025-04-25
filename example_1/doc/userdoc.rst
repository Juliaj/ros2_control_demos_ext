Ackermann Bot Demo
==================

This demo is to investigate the idea of using unpowered wheels for odometry calculation, see details in `ros-controls/ros2_controllers#1109 <https://github.com/ros-controls/ros2_controllers/issues/1109>`_.
The issue with the ackermann bot is that the wheels are powered and the slip is not taken into account.


Simulate the slipping wheels
-------------------------------
To create the slipping wheels, we add `surface` parameters to the test_ackermann_drive.xacro file for the rear wheel links under collision section.

The rear wheels are defined in the following code snippet:

.. code-block:: xml

   <collision name="rear_left_wheel_collision">
        <geometry>
          <cylinder>
            <radius>0.3</radius>
            <length>0.08</length>
          </cylinder>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.2</mu>
              <slip1>0.8</slip1>
              <slip2>0.9</slip2>
            </ode>
          </friction>
        </surface>
    </collision>

Note: 

In Gazebo's ODE physics engine, `slip1` and `mu` serve different but related purposes in friction modeling:

1. **mu (μ) - Friction Coefficient**  
   ```xml
   <ode>
     <mu>1.0</mu>       <!-- Primary friction coefficient -->
     <mu2>1.0</mu2>     <!-- Secondary friction coefficient -->
   </ode>
   ```
   - Determines the "grip" between surfaces
   - Higher μ = more friction/resistance to sliding
   - Typical range: 0.0 (ice) to 1.5 (rubber on concrete)
   - Governs the maximum friction force before sliding occurs

2. **slip1/slip2 - Slip Coefficients**  
   ```xml
   <ode>
     <slip1>0.5</slip1>  <!-- Longitudinal slip -->
     <slip2>0.5</slip2>  <!-- Lateral slip -->
   </ode>
   ```
   - Control how much sliding occurs AFTER friction is overcome
   - Higher values = more slippage during motion
   - Range: 0.0 (no slip) to 1.0 (full slip)

**Key relationship**:  
`mu` determines IF slipping occurs, while `slip1/slip2` determine HOW MUCH slipping occurs once in motion. They work together to model:  
1. Static friction (mu prevents initial movement)  
2. Dynamic friction (slip parameters control motion once moving)

For high slipperiness,we choose the following values:
- μ=0.1-0.3 (ice, oil spills)
- μ=0.3-0.5 (wet roads, loose gravel)


Run the demo
-----------

Start the bot with the following command:

.. code-block:: bash

    ros2 launch ros2_control_demo_ext_example_1 gazebo_default.launch.py

In a different terminal, run the following command to drive the bot in `8` shape:

.. code-block:: bash

    ros2 launch ros2_control_demo_ext_example_1 demo_test.launch.py

In another window, start the plotjuggler to visualize the odometry:

.. code-block:: bash

    ros2 run plotjuggler plotjuggler

Results
-------
Run 1: 

Configuration:
```xml
<ode>
  <mu>0.2</mu>
  <slip1>0.8</slip1>
  <slip2>0.9</slip2>
</ode>
```
Observation:
- The slipping is significant and the movement in Gazebo doesn't match the odometry info from the plotjuggler.

Video:
.. image:: ../images/slip_test_1.gif


