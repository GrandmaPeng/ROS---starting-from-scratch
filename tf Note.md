# tf Command
## tf_echo
View the transformation between two coordinate frames
```
tf_echo <source_frame> <targer_frame>
```
## view_frames
Generates pdf file to view the information of the whole tf tree
```
$ rosrun tf view_frames
$ evince frames.pdf
```
## tf_monitor
Prints all the frames information in the tf tree.
- ```
    tf_monitor
    ```
    All frames information
- ```
    tf_monitor <source_frame> <target_frame>
    ```
    Information between two certain frames.

```Net delay avg```: tf can only transform between turtles 8 ms ago instead of "now".

