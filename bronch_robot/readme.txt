enabling motors:

open service for enabling motors in a terminal:

rosrun bronch_robot enable_motors_server.py

call the client for enabling motors in another terminal:

rosrun bronch_robot enable_motors_client.py 1

if the client is called again motors will be disabled


homing motors:

To home motors, enable them first, then run the following command:

rosrun bronch_robot home_robot.py

The motors will be automatically disabled after homing.


