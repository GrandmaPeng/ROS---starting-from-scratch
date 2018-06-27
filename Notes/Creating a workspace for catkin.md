**catkin**: ROS中用于创建功能和编译功能包的命令

# Creating a workspace
    $ mkdir -p ~/catkin_ws/src
    $ cd ~/catkin_ws/
    $ catkin_make
    
- 第一次run `catkin_make` 会在src文件夹中生成CMakeLists.txt
- 执行完后生成了build和devel文件夹，devel中有setup.*sh文件。source任意一个其中的文件会把这个workspace覆盖当前其他workspace
 ```
$ source devel/setup.bash
```

