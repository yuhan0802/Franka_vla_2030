franka_mobile example_controllers
==========================

This package contains the controllers that can be used as a reference for TMRv0.2. 

Mobile Cartesian Velocity Example
^^^^^^^^^^^^^^^^^^^^^^^^^^

This controller subscribes to a topic that publishes cartesian velocities and forwards them to the 
robot after going through a rate limiter.

.. code-block:: shell

    ros2 launch franka_bringup example.launch.py controller_names:=mobile_cartesian_velocity_example_controller
