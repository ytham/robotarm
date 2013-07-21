# Robotic Arm controlled with Leap #
Created by Yu Jiang Tham

YouTube Link: http://www.youtube.com/watch?v=DnWJ3Q_PA5Y

This is the node.js code for my robotic arm that's controlled by an Arduino and a Leap Motion device.  Libraries used are leapjs and johnny-five.  The code needs to be modified in the lengths of the robot arm parts (in terms of Leap Motion space).  Additionally, care must be taken to place the servo in such a way that the joints allow for a full range of motion.

Servos used:
Base: HiTec HS-422
Shoulder: HiTec HS-755HB
Elbow: RadioShack "Standard Servo"
End Effector: HiTec HS-81
