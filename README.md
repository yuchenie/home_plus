# home+ Hardware Controls
## Serial Inputs
In order to control the positions of the base and arm joints, the user should send a message through the serial port connected to the Arduino via USB cable. The interface is designed to receive serial messages either through the Serial Monitor in the Arduino IDE, or through an external script configured to send messages through the appropriate serial port on the user's device. The serial message should adhere to the following format: "%f %f %f %f %f %f %f %f %f %f". The serial message should be a series of ten floats, with each float separated by a single space. Sequentially, each float represents the absolute position* of the following:
1. The position (cm) of the base in the x-axis. The positive x-axis, from the perspective of the base center, is in the direction of the arm.
2. The position (cm) of the base in the y-axis. The positive y-axis, from the perspective of the base center, is in the direction to the right of the arm.
3. The orientation (degrees) of the base. A positive translation in the orientation axis, viewing from the top, is clockwise.
4. The configuration of the gripper. This is a value between 0 and 100, with 0 mapping to fully closed and 100 mapping to fully opened.
5. The configuration of the "hand" servo, rotating the end effector around the axis through the center of the "wrist" joint. This is a value between 0 and 100.
6. The configuration of the "wrist" servo, rotating the end effector around the axis through the length of the "forearm" segment. This is a value between 0 and 100.
7. The configuration of the "elbow" servo, rotating the end effector around the "elbow" joint. This is a value between 0 and 100.
8. The configuration of the "shoulder" servo, rotating the end effector around the "shoulder" joint, which is mounted to the top of the center linear actuator. This is a value between 0 and 100.
9. The height (cm) of the linear actuator within the frame. The assumption is made that the linear actuator begins at the fully extended position (450 cm), because the robot lacks an absolute encoder for the height of the linear actuator. The input should be a value between 0 and 450.
10. The height (cm) of the linear actuator that translates the height of the frame. The assumption is made that the linear actuator begins at the fully retracted position (0 cm), because the robot lacks an absolute encoder for the height of the linear actuator. The input should be a value between 0 and 175.
* The absolute position of the base assumes that the origin---the position at which (x,y,θ) = (0,0,0)---is the position at which the Arduino is powered on.
