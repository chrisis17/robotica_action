# action_example
This package is for example of ros nodes action server and client in python and C++.

Instructions to run the C++ nodes, python nodes don't need compilation.

```
$ cd
$ cd catkin_ws
$ catkin_make --only-pkg-with-deps action_example
```
Once the package is build, you can run the nodes written in C++.

```
$ rosrun action_example action_server
$ rosrun action_example action_client
```
