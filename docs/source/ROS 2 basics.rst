ROS 2 basics
=====

.. _ros2_basics:

What is ROS 2?
--------------

- ROS 2 (Robotic Operating System) is a software platform for developing robotics applications, also known as a robotics software development kit (SDK).

- ROS 2 software ecosystem is divided into three categories:

   - Middleware: Referred to as the plumbing, the ROS 2 middleware encompasses communication among components, from network APIs to message parsers. By "Middleware" it refers to an intermediate layer between the various components of robotics (sensors, actuators, controllers) and higher-level software modules.

   .. image:: images/ROS2middleware.png
      :alt: ROS2 middleware graph.

   - Algorithms: ROS 2 provides many of the algorithms commonly used when building robotics applications, e.g. perception, SLAM, planning, and beyond.
   
   - Developer tools: ROS 2 includes a suite of commandline and graphical tools for configuration, launch, introspection, visualization, debugging, simulation, and logging. There is also a large suite of tools for source management, build processes, and distribution.

- ROS 2 is open source with a large community around it. From students and hobbyists to multinational corporations and government agencies, people and organizations of all stripes keep the ROS project going.

- Finally, the "2" in this ROS 2 course, it obviously indicates that there was, and in fact there is still, a ROS 1 version whose support will go until 2025 (check the latest ROS distro here: http://wiki.ros.org/Distributions). While ROS 1 solves many of the complexity issues inherent to robotics, it struggles to consistently deliver data over lossy links (like WiFi or satellite links), has a single point of failure, and does not have any built-in security mechanisms. In order to address these challenges, a new generation of ROS was redeisgned from the ground, that is ROS 2.

The above information was extracted and based from the following resources:

- https://www.science.org/doi/abs/10.1126/scirobotics.abm6074
- https://www.theconstruct.ai/wp-content/uploads/2021/06/ROS2-Developers-Guide-2021.pdf

Quick summary of ROS: https://vimeo.com/639235111


If coming from ROS 1 
------------

The main differences that one should consider when switching from ROS 1 to ROS 2 are:

- The ROS node called ``master``, which needed to be executed prior any other node, in ROS 1, does not exist anymore in ROS 2.
- The launch files in ROS 1 could not be coded in python scripts, these had to be ``.xml`` extension launch files. Now, in ROS 2 launch files support ``.xml``, ``.yaml`` and ``.py`` extensions.
- In ROS 1, the libraries to use C++ and python code were ``rospy`` and ``roscpp`` respectively. These were completely independent from each other. Whereas in ROS 2 the ``rclcpp`` and ``rclpy`` libraries (the first one for C++ code and the later for python code) have one unique base library called ``rcl``.
- Introduction of QoS (Quality of Service) to ROS 2. By configuring QoS settings, developers can tailor ROS 2 communication to meet the specific requirements of their robotic systems, balancing factors such as latency, throughput, and fault tolerance.
- In ROS 1, the build tool were ``catkin_make`` or ``catkin_build``, while in ROS 2, ``ament`` is the building system with ``colcon`` as the command line tool. 
- ROS 2 uses DDS (Data Distribution Service) as the network protocol for all communication that happens internally. It provides the security guarantees as well as the reliability needed to maintain good communication in areas with weak or lossy connections. While ROS 1 uses a custom TCP/UDP protocol that missed a lot of the security and reliability guarantees.

Check this link for more insights on the transition from ROS 1 to ROS 2: https://roboticsbackend.com/ros1-vs-ros2-practical-overview/#Why_ROS2_and_not_keep_ROS1