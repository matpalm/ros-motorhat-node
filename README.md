[ros](http://www.ros.org/) node wrapper for the [Adafruit DC & Stepper Motor HAT](https://www.adafruit.com/products/2348).

![Adafruit motor hat](motor_hat.jpg)

note: no code for the Stepper Motor control, just the DCs.

subscribes to [std_msg::Int16MultiArray](http://docs.ros.org/jade/api/std_msgs/html/msg/Int16MultiArray.html)
messages expecting 4 values (one for each motor) between -255 (full reverse) to 255 (full forward). A value of 0 denotes turning
off motor. 

node sends [0,0,0,0] (i.e. stop all) on exit.

````
# example sending of command.
rostopic pub -1 /cmd std_msgs/Int16MultiArray "{data:[1,2,3,4]}"
````

TODOS
* no support for stepper control.
* integrate conversion of 4 values to a twist directly in this code (?)

