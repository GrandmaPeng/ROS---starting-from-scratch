# rosed
**rosed** is a part of rosbash. It allows you to edit a file in the package directly by the package name.

    # rosed [package_name] [filename]
```
$ rosed roscpp Logger.msg
```
The example shows how to edit **Logger.msg** file in the **roscpp** package. 
# Editor
The default editor is vim. You can choose anthor editor by modifying ~/.bashrc:
   
    export EDITOR='gedit'


