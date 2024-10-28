Creating custom msg and srv. C++
==========================

.. _custom msg and srv cpp:


In previous sections, predefined messages and service types were used. Recall the ``String`` message type in `the publisher and subscriber example`_ or the ``AddTwoInts`` service in the `service and client examples`_. These types of interfaces already existed and were ready to be used. In this section, custom messages and services types will be created and applied into program examples under the C++ programming language.

.. _`the publisher and subscriber example`: https://ros2course.readthedocs.io/en/latest/Writing%20publisher%20and%20subscriber%20nodes.%20C%2B%2B.html#writing-publisher-and-subscriber-nodes-c
.. _`service and client examples`: https://ros2course.readthedocs.io/en/latest/Writing%20service%20and%20client.%20C%2B%2B.html#writing-service-and-client-c


Setup for working with custom msg and srv
------------------------

In this :ref:`previous section<Creating custom msg and srv. Python/Setup for working with custom msg and srv>` it was already created a custom msg and srv: the ``Sphere.msg`` msg and the ``AddThreeInts.srv`` srv, that belong to the ``tutorial_interfaces`` package. These two custom interfaces will be used along this section of the course. 

Testing the Sphere custom msg in a C++ package
-----------------------
Make sure to be in a `brand new terminal`_ window and no ROS command is currently running. 

.. _`brand new terminal`: https://ros2course.readthedocs.io/en/latest/Installation%20and%20software%20setup.html#running-a-docker-container


Create a new python package,  this package should be contained in the ``ros2_ws`` workspace, within its ``/src`` folder. The name provided to this new package will be ``testing_interfaces_cpp``. 

For more reference on package creation consult the `package creation`_ section.

.. _package creation: https://ros2course.readthedocs.io/en/latest/Configuring%20environment.html#creating-and-configuring-a-package

.. code-block:: console

   ros2 pkg create --build-type ament_cmake --license Apache-2.0 testing_interfaces_cpp --dependencies rclcpp tutorial_interfaces

The ``--dependencies`` argument will automatically add the necessary dependency lines to ``package.xml`` file. In this case, ``tutorial_interfaces`` is the package that includes the ``Sphere.msg`` file that is needed for this test.

The code
~~~~~~~~~~~~~~~~

Next, inside ``testing_interfaces_cpp/src`` create a C++ script, name it ``sphere_publisher.cpp``. 

Copy this content into the new C++ script. 

.. code-block:: cpp

   #include <chrono>
   #include <memory>

   #include "rclcpp/rclcpp.hpp"
   #include "tutorial_interfaces/msg/Sphere.hpp"                                            // CHANGE

   using namespace std::chrono_literals;

   class SpherePublisher : public rclcpp::Node
   {
      public:
      SpherePublisher()
      : Node("sphere_publisher"), count_(0)
      {
         publisher_ = this->create_publisher<tutorial_interfaces::msg::Sphere>("sphere_topic", 10);  // CHANGE
         timer_ = this->create_wall_timer(
            500ms, std::bind(&SpherePublisher::timer_callback, this));
      }

      private:
      void timer_callback()
      {
         auto message = tutorial_interfaces::msg::Sphere();                                   // CHANGE
         message.center.x = this->count_;
         message.center.y = 1.0; 
         message.center.z = 2.0;
         message.radius = 10.0;

         RCLCPP_INFO_STREAM(this->get_logger(), "Publishing sphere params " \
         "(x, y, z, radius): x = " << message.center.x << ", y = " \
            << message.center.y << ", z = " << message.center.z << \
            ", radius = " << message.radius);    // CHANGE
         publisher_->publish(message);
         this->count_++;
      }

      rclcpp::TimerBase::SharedPtr timer_;
      rclcpp::Publisher<tutorial_interfaces::msg::Sphere>::SharedPtr publisher_;             // CHANGE
      size_t count_;
   };

   int main(int argc, char * argv[])
   {
      rclcpp::init(argc, argv);
      rclcpp::spin(std::make_shared<SpherePublisher>());
      rclcpp::shutdown();
      return 0;
   }

Notice that this code is very similar to the publisher script that was studied `publisher script that was studied previously`_.

.. _`publisher script that was studied previously`: https://ros2course.readthedocs.io/en/latest/Writing%20publisher%20and%20subscriber%20nodes.%20C%2B%2B.html#publisher-node-in-c

Check the important changes in this script.

.. code-block:: cpp

   #include "tutorial_interfaces/msg/Sphere.hpp"                                      
   ...
   publisher_ = this->create_publisher<tutorial_interfaces::msg::Sphere>("sphere_topic", 10);     
   ...
   void timer_callback()
   {
      auto message = tutorial_interfaces::msg::Sphere();                                  
      message.center.x = this->count_;
      message.center.y = 1.0; 
      message.center.z = 2.0;
      message.radius = 10.0;

      RCLCPP_INFO_STREAM(this->get_logger(), "Publishing sphere params " \
      "(x, y, z, radius): x = " << message.center.x << ", y = " \
         << message.center.y << ", z = " << message.center.z << \
         ", radius = " << message.radius);    // CHANGE
      publisher_->publish(message);
      this->count_++;
   }


- It is important to correctly import the required libraries. Importing the custom message definition of ``Sphere``.
- The publisher node will now publish different type of messages and will also publish to a different topic. The topic name could have stayed the same, but it is better to name the topics accordingly.
- Finally, the callback function, instead of directly publishing a string message, it is necessary to fill every parameter that is needed for the new message type. 

Next, create another node, a listener node for this publisher. Inside ``testing_interfaces_cpp/src`` create a C++ script, name it ``sphere_listener.cpp``. 

Copy this content into the new C++ script. 

.. code-block:: cpp

   #include <functional>
   #include <memory>

   #include "rclcpp/rclcpp.hpp"
   #include "tutorial_interfaces/msg/Sphere.hpp"                                       // CHANGE

   using std::placeholders::_1;

   class SphereListener : public rclcpp::Node
   {
      public:
      SphereListener()
      : Node("sphere_listener")
      {
         subscription_ = this->create_subscription<tutorial_interfaces::msg::Sphere>(    // CHANGE
            "sphere_topic", 10, std::bind(&SphereListener::topic_callback, this, _1));
      }

      private:
      void topic_callback(const tutorial_interfaces::msg::Sphere & msg) const  // CHANGE
      {
         RCLCPP_INFO_STREAM(this->get_logger(), "I heard" \
         ": x = " << msg.center.x << ", y = " \
            << msg.center.y << ", z = " << msg.center.z << \
            ", radius = " << msg.radius);    // CHANGE
      }
      rclcpp::Subscription<tutorial_interfaces::msg::Sphere>::SharedPtr subscription_;  // CHANGE
   };

   int main(int argc, char * argv[])
   {
      rclcpp::init(argc, argv);
      rclcpp::spin(std::make_shared<SphereListener>());
      rclcpp::shutdown();
      return 0;
   }

The code is very similar to the listener script that was studied `listener script that was studied previously`_.

.. _`listener script that was studied previously`: https://ros2course.readthedocs.io/en/latest/Writing%20publisher%20and%20subscriber%20nodes.%20C%2B%2B.html#subscriber-node-in-cpp

Again, the relevant changes here, have to do with dealing with the appropriate topic name and message type. 

Dependencies and CMakeLists
~~~~~~~~~~~~~~~~

Once, these two C++ scripts are ready, it is necessary to add the required dependencies in the ``package.xml`` file, which was already added when creating this package. See that in the ``package.xml`` file it is present the tags: ``<depend>rclcpp</depend>`` and ``<depend>tutorial_interfaces</depend>``.

Next, add the following in the ``CMakeLists.txt`` file:

.. code-block:: console

   add_executable(sphere_publisher src/sphere_publisher.cpp)
   ament_target_dependencies(sphere_publisher rclcpp tutorial_interfaces)    

   add_executable(sphere_listener src/sphere_listener.cpp)
   ament_target_dependencies(sphere_listener rclcpp tutorial_interfaces)    

   install(TARGETS
   sphere_publisher
   sphere_listener
   DESTINATION lib/${PROJECT_NAME})


Build and run the custom msg
~~~~~~~~~~~~~~~~

Build the package with either of these commands:

.. code-block:: console

   colcon build
   colcon build --packages-select testing_interfaces_cpp

Source the setup file:

.. code-block:: console
   
   source install/setup.bash

And run the ``sphere_publisher`` node that was recently created. 

.. code-block:: console
   
   ros2 run testing_interfaces_cpp sphere_publisher

The result should be like the following:

.. code-block:: console
   
   [INFO] [1712745603.801777360] [sphere_publisher]: Publishing sphere params (x, y, z, radius): x = 0, y = 1, z = 2, radius = 10
   [INFO] [1712745604.301748381] [sphere_publisher]: Publishing sphere params (x, y, z, radius): x = 1, y = 1, z = 2, radius = 10
   [INFO] [1712745604.801799750] [sphere_publisher]: Publishing sphere params (x, y, z, radius): x = 2, y = 1, z = 2, radius = 10
   ...

`Open a new terminal`_ and execute the ``sphere_listener`` node:

.. _open a new terminal: https://ros2course.readthedocs.io/en/latest/Installation%20and%20software%20setup.html#opening-a-new-terminal-for-the-docker-container

.. code-block:: console
   
   ros2 run testing_interfaces_cpp sphere_listener

The expected result is:

.. code-block:: console
   
   [INFO] [1712745636.802284213] [sphere_listener]: I heard: x = 66, y = 1, z = 2, radius = 10
   [INFO] [1712745637.302150919] [sphere_listener]: I heard: x = 67, y = 1, z = 2, radius = 10
   [INFO] [1712745637.802143924] [sphere_listener]: I heard: x = 68, y = 1, z = 2, radius = 10
   ...

Finally, it can also be checked the echo of the messages arriving to the desired topic. `Open a new terminal`_ and execute:

.. code-block:: console
   
   ros2 topic echo /sphere_topic

The expected result is:

.. code-block:: console
   
   center:
      x: 132.0
      y: 1.0
      z: 2.0
   radius: 10.0
   ---
   center:
      x: 133.0
      y: 1.0
      z: 2.0
   radius: 10.0
   ---
   ...

At this point, it can be seen that the custom message ``Sphere.msg`` that was created is being used successfully.

Testing the AddThreeInts custom srv in a C++ package
-----------------------

This example will be worked in the ``testing_interfaces_cpp`` package.

Make sure to be in a `brand new terminal`_ window and no ROS command is currently running. 


The code
~~~~~~~~~~~~~~~~

Inside ``testing_interfaces_cpp/src`` create a C++ script, name it ``add_service_node.cpp``. 

Copy this content into the new python script. 

.. code-block:: cpp

   #include "rclcpp/rclcpp.hpp"
   #include "tutorial_interfaces/srv/add_three_ints.hpp"                                        

   #include <memory>

   void add(const std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Request> request,     
            std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Response>       response)  
   {
      response->sum = request->a + request->b + request->c;                                      
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld" " c: %ld",  
                     request->a, request->b, request->c);                                         
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
   }

   int main(int argc, char **argv)
   {
      rclcpp::init(argc, argv);

      std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_server");   

      rclcpp::Service<tutorial_interfaces::srv::AddThreeInts>::SharedPtr service =               
         node->create_service<tutorial_interfaces::srv::AddThreeInts>("add_three_ints",  &add);   

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add three ints.");                     

      rclcpp::spin(node);
      rclcpp::shutdown();
   }

Notice that this code is very similar to the `service script that was studied previously`_.

.. _`service script that was studied previously`: https://ros2course.readthedocs.io/en/latest/Writing%20service%20and%20client.%20C%2B%2B.html#writing-the-service-node-c

Check the important changes in this script.

.. code-block:: cpp

   #include "tutorial_interfaces/srv/add_three_ints.hpp"  
   ...
   rclcpp::Service<tutorial_interfaces::srv::AddThreeInts>::SharedPtr service =               
         node->create_service<tutorial_interfaces::srv::AddThreeInts>("add_three_ints",  &add);
   ...
   void add(const std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Request> request,     
            std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Response>       response)  
   {
      response->sum = request->a + request->b + request->c;                                      
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld" " c: %ld",  
                     request->a, request->b, request->c);                                         
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
   }

- It is important to correctly import the required service. In this case, notice that ``add_three_ints.hpp`` is being imported when the actual created service was named ``AddThreeInts.srv`` (See the `definition of the service`_ section to recall about the name of the service). If ``#include "tutorial_interfaces/srv/AddThreeInts.hpp"``  were to be imported, a compilation error would have arisen stating:

.. _`definition of the service`: https://ros2course.readthedocs.io/en/latest/Creating%20custom%20msg%20and%20srv.%20Python.html#service-definition

.. code-block:: console
   
   fatal error: tutorial_interfaces/srv/AddThreeInts.hpp: No such file or directory

This happens because in ROS 2, the naming convention for service files (.srv) is usually converted to snake_case when generating corresponding C++ code. So, a service file named ``AddThreeInts.srv``, when generating C++ code, it will typically be converted to ``add_three_ints.hpp``.

- The service node will be of type ``AddThreeInts``, and the service name of name: ``add_three_ints``. 
- Finally, the callback function, instead of adding two values it will summ the three parameters in the request section of the service. 

Next, create a client node for this service. Inside ``testing_interfaces_cpp/src`` create a C++ script, name it ``add_client_node.cpp``. 

Copy this content into the new cpp script. 

.. code-block:: cpp

   #include "rclcpp/rclcpp.hpp"
   #include "tutorial_interfaces/srv/add_three_ints.hpp"                                       // CHANGE

   #include <chrono>
   #include <cstdlib>
   #include <memory>

   using namespace std::chrono_literals;

   int main(int argc, char **argv)
   {
      rclcpp::init(argc, argv);

      if (argc != 4) { // CHANGE
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_three_ints_client X Y Z");      // CHANGE
            return 1;
      }

      std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_client");  // CHANGE
      rclcpp::Client<tutorial_interfaces::srv::AddThreeInts>::SharedPtr client =                // CHANGE
         node->create_client<tutorial_interfaces::srv::AddThreeInts>("add_three_ints");          // CHANGE

      auto request = std::make_shared<tutorial_interfaces::srv::AddThreeInts::Request>();       // CHANGE
      request->a = atoll(argv[1]);
      request->b = atoll(argv[2]);
      request->c = atoll(argv[3]);                                                              // CHANGE

      while (!client->wait_for_service(1s)) {
         if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
         }
         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }

      auto result = client->async_send_request(request);
      // Wait for the result.
      if (rclcpp::spin_until_future_complete(node, result) ==
         rclcpp::FutureReturnCode::SUCCESS)
      {
         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
      } else {
         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_three_ints");    // CHANGE
      }

      rclcpp::shutdown();
      return 0;
   }

The code is very similar to the client node that was studied `service client script that was studied previously`_.

.. _`service client script that was studied previously`: https://ros2course.readthedocs.io/en/latest/Writing%20service%20and%20client.%20C%2B%2B.html#client-node-in-c

Again, the relevant changes here, have to do with dealing with the appropriate import of the required library, the service name and service type. 

Dependencies and CMakeLists file
~~~~~~~~~~~~~~~~

Once, these two C++ scripts are ready, it is necessary to add the required dependencies in the ``package.xml`` file, which was already added when creating this package. See that in the ``package.xml`` file it is present the tags: ``<depend>rclcpp</depend>`` and ``<depend>tutorial_interfaces</depend>``.

Next, add the following to the ``CMakeLists.txt`` file:

.. code-block:: console

   ...
   add_executable(add_service_node src/add_service_node.cpp)
   ament_target_dependencies(add_service_node rclcpp tutorial_interfaces) 

   add_executable(add_client_node src/add_client_node.cpp)
   ament_target_dependencies(add_client_node rclcpp tutorial_interfaces) 
   ...
   install(TARGETS
      ...
      add_service_node
      add_client_node
      DESTINATION lib/${PROJECT_NAME})

Considering the changes for the custom msg as well, the final ``CMakeLists.txt`` file should look like this:

.. code-block:: console

   cmake_minimum_required(VERSION 3.8)
   project(testing_interfaces_cpp)

   if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      add_compile_options(-Wall -Wextra -Wpedantic)
   endif()

   # find dependencies
   find_package(ament_cmake REQUIRED)
   find_package(rclcpp REQUIRED)
   find_package(tutorial_interfaces REQUIRED)

   add_executable(sphere_publisher src/sphere_publisher.cpp)
   ament_target_dependencies(sphere_publisher rclcpp tutorial_interfaces)    

   add_executable(sphere_listener src/sphere_listener.cpp)
   ament_target_dependencies(sphere_listener rclcpp tutorial_interfaces)    

   add_executable(add_service_node src/add_service_node.cpp)
   ament_target_dependencies(add_service_node rclcpp tutorial_interfaces) 

   add_executable(add_client_node src/add_client_node.cpp)
   ament_target_dependencies(add_client_node rclcpp tutorial_interfaces) 

   install(TARGETS
      sphere_publisher
      sphere_listener
      add_service_node
      add_client_node
      DESTINATION lib/${PROJECT_NAME})

   if(BUILD_TESTING)
      find_package(ament_lint_auto REQUIRED)
      # the following line skips the linter which checks for copyrights
      # comment the line when a copyright and license is added to all source files
      set(ament_cmake_copyright_FOUND TRUE)
      # the following line skips cpplint (only works in a git repo)
      # comment the line when this package is in a git repo and when
      # a copyright and license is added to all source files
      set(ament_cmake_cpplint_FOUND TRUE)
      ament_lint_auto_find_test_dependencies()
   endif()

   ament_package()

Build and run the custom srv
~~~~~~~~~~~~~~~~

Build the package with either of these commands:

.. code-block:: console

   colcon build
   colcon build --packages-select testing_interfaces_cpp

Source the setup file:

.. code-block:: console
   
   source install/setup.bash

And run the ``add_service_node`` node that was recently created. 

.. code-block:: console
   
   ros2 run testing_interfaces_cpp add_service_node

As a result, this will be shown in the terminal, meaning that the service is ready to be consumed. 

.. code-block:: console

   [INFO] [1712746785.956178405] [rclcpp]: Ready to add three ints.

`Open a new terminal`_ and execute the ``add_client_node`` node:

.. code-block:: console
   
   ros2 run testing_interfaces_cpp add_client_node 8 9 5

The expected result is:

.. code-block:: console
   
   [INFO] [1712746812.713518561] [rclcpp]: Sum: 22

Finally, the ``add_three_ints service`` can also be called from the terminal directly, without the necessity of coding a client node. `Open a new terminal`_ and execute:

.. code-block:: console
   
   ros2 service call /add_three_ints tutorial_interfaces/srv/AddThreeInts "{a: 1, b: 2, c: 9}"

The expected result is:

.. code-block:: console
   
   requester: making request: tutorial_interfaces.srv.AddThreeInts_Request(a=1, b=2, c=9)

   response:
   tutorial_interfaces.srv.AddThreeInts_Response(sum=12)

At this point, it can be seen that the custom service ``AddThreeInts.srv`` that was created is being used successfully.

Testing a custom msg inisde the same package
-----------------------

The previous examples had the custom msg and srv created in a different package from where these are tested. Recall that the custom msg and srv were created in ``tutorial_interfaces`` but are tested in the ``testing_interfaces_cpp`` package.

In this part, a custom msg will be created in a package of name: ``more_interfaces`` and inside this very same package, a node will be created that makes use of the custom msg. It will be seen that there are some minor differences when using a msg generated in the same package. 

The process below is similar to the one `studied previously`_.

.. _`studied previously`: https://ros2course.readthedocs.io/en/latest/Creating%20custom%20msg%20and%20srv.%20Python.html#testing-the-sphere-custom-msg-in-a-python-package

Create a new package
~~~~~~~~~~~~~~~~

`Open a new terminal`_ and make sure that no ROS commands are currently running. 

Create a new package. This package should be contained in the ``ros2_ws`` workspace, within its ``/src`` folder. The name provided to this new package will be ``more_interfaces``.

.. code-block:: console

   ros2 pkg create --build-type ament_cmake --license Apache-2.0 more_interfaces

Create a new custom msg
~~~~~~~~~~~~~~~~

Next, create the folder: ``msg`` inside ``ros2_ws/src/more_interfaces``. This is where the new messages types will be stored.

Inside ``more_interfaces/msg`` create a new file named ``AddressBook.msg``. Edit the content of ``AddressBook.msg`` to include:

.. code-block:: console

   uint8 PHONE_TYPE_HOME=0
   uint8 PHONE_TYPE_WORK=1
   uint8 PHONE_TYPE_MOBILE=2

   string first_name
   string last_name
   string phone_number
   uint8 phone_type

Note that it is possible to set default values for fields within a message definition. 

Build the msg file
~~~~~~~~~~~~~~~~

To make sure that the msg file is turned into source code for C++ and Python, the following should be added in the ``more_interfaces/package.xml`` file:

.. code-block:: console

   <buildtool_depend>rosidl_default_generators</buildtool_depend>
   <exec_depend>rosidl_default_runtime</exec_depend>
   <member_of_group>rosidl_interface_packages</member_of_group>

- ``rosidl_default_generators`` is a package in ROS 2 that provides default code generators for ROS message and service types. It is part of the ROS 2 build system and is used to generate C++ and Python code from ROS 2 message and service definitions. The ``<buildtool_depend>`` specifies a dependency on a build tool needed to build the package.
- ``<exec_depend>`` is a runtime or execution-stage dependency. ``rosidl_default_runtime`` is a ROS 2 package that provides runtime libraries necessary for working with ROS 2 messages and services.
- The ``<member_of_group>`` tag specifies that the package is a member of a particular group.  In this case, ``<member_of_group>rosidl_interface_packages</member_of_group>`` indicates that the package is part of the ``rosidl_interface_packages`` group. The ``rosidl_interface_packages`` group typically includes packages that define ROS interfaces, such as messages, services, and action definitions. These packages contain ``.msg``, ``.srv``, and ``.action`` files that define the structure and behavior of messages, services, and actions used in ROS 2 communication.

Now, regarding the ``CMakeLists.txt`` file, the following should be added just below the ``find_package(ament_cmake REQUIRED)`` line:

.. code-block:: console

   find_package(rosidl_default_generators REQUIRED)
   set(msg_files
      "msg/AddressBook.msg"
   )
   rosidl_generate_interfaces(${PROJECT_NAME}
      ${msg_files}
   )

- The ``find_package(...)`` command finds the package that generates message code from msg/srv files.
- The ``set(...)`` command declares a list of messages that is to be generated.- The ``rosidl_generate_interfaces(...)`` command generates the messages.

Open a `brand new terminal`_, make sure that no other ROS 2 command is currently running, navigate to the workspace directory and execute:

.. code-block:: console

   colcon build --packages-select more_interfaces

Now, source the setup file:

.. code-block:: console
   
   source install/setup.bash

For more reference on sourcing the setup file, see `sourcing the setup file`_.

.. _sourcing the setup file: https://ros2course.readthedocs.io/en/latest/Configuring%20environment.html#workspace-sourcing

Next, to check that the custom message is correctly created, run:

.. code-block:: console
   
   ros2 interface show more_interfaces/msg/AddressBook

The otuput should be: 

.. code-block:: console
   
   uint8 PHONE_TYPE_HOME=0
   uint8 PHONE_TYPE_WORK=1
   uint8 PHONE_TYPE_MOBILE=2

   string first_name
   string last_name
   string phone_number
   uint8 phone_type

At this point the custom msg is created and ready to be used.

The cpp code in the same package
~~~~~~~~~~~~~~

Inside ``more_interfaces/src`` create a C++ script, name it ``publish_address_book.cpp``. 

Copy this content into the new C++ script. 

.. code-block:: cpp

   #include <chrono>
   #include <memory>

   #include "rclcpp/rclcpp.hpp"
   #include "more_interfaces/msg/address_book.hpp"

   using namespace std::chrono_literals;

   class AddressBookPublisher : public rclcpp::Node
   {
   public:
   AddressBookPublisher()
   : Node("address_book_publisher")
   {
      address_book_publisher_ =
         this->create_publisher<more_interfaces::msg::AddressBook>("address_book", 10);

      auto publish_msg = [this]() -> void {
         auto message = more_interfaces::msg::AddressBook();

         message.first_name = "John";
         message.last_name = "Doe";
         message.phone_number = "1234567890";
         message.phone_type = message.PHONE_TYPE_MOBILE;

         std::cout << "Publishing Contact\nFirst:" << message.first_name <<
            "  Last:" << message.last_name << std::endl;

         this->address_book_publisher_->publish(message);
         };
      timer_ = this->create_wall_timer(1s, publish_msg);
   }

   private:
      rclcpp::Publisher<more_interfaces::msg::AddressBook>::SharedPtr address_book_publisher_;
      rclcpp::TimerBase::SharedPtr timer_;
   };


   int main(int argc, char * argv[])
   {
      rclcpp::init(argc, argv);
      rclcpp::spin(std::make_shared<AddressBookPublisher>());
      rclcpp::shutdown();

      return 0;
   }

The code consists on these parts:

- Library imports. Notice specially the ``address_book.hpp`` header that is imported. As explained in `this section`_  the srv and msg files are usually converted to snake_case when generating corresponding C++ code.

.. _`this section`: https://ros2course.readthedocs.io/en/latest/Creating%20custom%20msg%20and%20srv.%20C%2B%2B.html#id1

.. code-block:: cpp

   #include <chrono>
   #include <memory>

   #include "rclcpp/rclcpp.hpp"
   #include "more_interfaces/msg/address_book.hpp"

   using namespace std::chrono_literals;


- Creating a node and an ``AddressBook`` publisher.

.. code-block:: cpp

   class AddressBookPublisher : public rclcpp::Node
   {
   public:
   AddressBookPublisher()
   : Node("address_book_publisher")
   {
      address_book_publisher_ =
         this->create_publisher<more_interfaces::msg::AddressBook>("address_book");

- Create a callback to publish the messages periodically.

.. code-block:: cpp

   auto publish_msg = [this]() -> void {
      auto message = more_interfaces::msg::AddressBook();

      message.first_name = "John";
      message.last_name = "Doe";
      message.phone_number = "1234567890";
      message.phone_type = message.PHONE_TYPE_MOBILE;

      std::cout << "Publishing Contact\nFirst:" << message.first_name <<
         "  Last:" << message.last_name << std::endl;

      this->address_book_publisher_->publish(message);
   };

- Create a 1 second timer to call the ``publish_msg`` callback function every second.

.. code-block:: cpp

   timer_ = this->create_wall_timer(1s, publish_msg);

Build the publisher
~~~~~~~~~~~~~~

Add the following to the ``CMakeLists.txt`` file, just below the ``find_package(rosidl_default_generators REQUIRED)`` command:

.. code-block:: console

   find_package(rclcpp REQUIRED)

   add_executable(publish_address_book src/publish_address_book.cpp)
   ament_target_dependencies(publish_address_book rclcpp)

   install(TARGETS
      publish_address_book
     DESTINATION lib/${PROJECT_NAME})

In order to use the messages generated in the same package it is needed to use the following CMake code, add this just below the ``rosidl_generate_interfaces(...
)`` command:

.. code-block:: console

   rosidl_get_typesupport_target(cpp_typesupport_target
   ${PROJECT_NAME} rosidl_typesupport_cpp)

   target_link_libraries(publish_address_book "${cpp_typesupport_target}")

This CMake code is only required when interfaces are to used in the same package as they are defined.

At the end, this ``CMakeLists.txt`` file should look like the following:

.. code-block:: txt

   cmake_minimum_required(VERSION 3.8)
   project(more_interfaces)

   if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      add_compile_options(-Wall -Wextra -Wpedantic)
   endif()

   # find dependencies
   find_package(ament_cmake REQUIRED)
   find_package(rosidl_default_generators REQUIRED)

   find_package(rclcpp REQUIRED)

   add_executable(publish_address_book src/publish_address_book.cpp)
   ament_target_dependencies(publish_address_book rclcpp)

   install(TARGETS
      publish_address_book
      DESTINATION lib/${PROJECT_NAME})

   set(msg_files
      "msg/AddressBook.msg"
   )

   rosidl_generate_interfaces(${PROJECT_NAME}
      ${msg_files}
   )

   rosidl_get_typesupport_target(cpp_typesupport_target
      ${PROJECT_NAME} rosidl_typesupport_cpp)

   target_link_libraries(publish_address_book "${cpp_typesupport_target}")

   # uncomment the following section in order to fill in
   # further dependencies manually.
   # find_package(<dependency> REQUIRED)

   if(BUILD_TESTING)
      find_package(ament_lint_auto REQUIRED)
      # the following line skips the linter which checks for copyrights
      # comment the line when a copyright and license is added to all source files
      set(ament_cmake_copyright_FOUND TRUE)
      # the following line skips cpplint (only works in a git repo)
      # comment the line when this package is in a git repo and when
      # a copyright and license is added to all source files
      set(ament_cmake_cpplint_FOUND TRUE)
      ament_lint_auto_find_test_dependencies()
   endif()

   ament_package()

Run the publisher
~~~~~~~~~~~~~~

Open a `brand new terminal`_, make sure that no other ROS 2 command is currently running, navigate to the workspace directory and execute:

.. code-block:: console
   
   colcon build --packages-select more_interfaces


Now, source the setup file:

.. code-block:: console
   
   source install/setup.bash

For more reference on sourcing the setup file, see `sourcing the setup file`_.

And run the publisher node that was recently created. 

.. code-block:: console
   
   ros2 run more_interfaces publish_address_book

As a result, the following messages will be displayed in the terminal:

.. code-block:: console
   
   Publishing Contact
   First:John  Last:Doe
   Publishing Contact
   First:John  Last:Doe
   ...


Finally, it can also be checked the echo of the messages arriving to the desired topic. `Open a new terminal`_ and execute:

.. code-block:: console
   
   ros2 topic echo /address_book

The expected result is:

.. code-block:: console
   
   first_name: John
   last_name: Doe
   phone_number: '1234567890'
   phone_type: 2
   ---
   first_name: John
   last_name: Doe
   phone_number: '1234567890'
   phone_type: 2
   ---
   ...

At this point, it can be seen that the custom message ``AddressBook.msg`` that was created is being used successfully within the same package in which it was defined.