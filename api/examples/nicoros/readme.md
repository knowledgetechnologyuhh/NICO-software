# ROS examples

The example nodes for the nicoros package need to be within a ROS package to be executed. Therefore they are also located in the nicoros package and can be run the same way, e.g.:

```bash
rosrun nicoros OptoforceExample.py
```

For these examples to work, the corresponding node also needs to be running.

This readme file serves as an overview over the different ROS nodes and their respective examples:

|Node|Example node|
|----|:-------:|
|Motion.py|yesno_example.py|
|Optoforce.py|OptoforceExample.py|
|AudioStream.py|StreamReceiverExample.py|
|Vision.py|ImageSubscriberExample.py|
|FaceExpression.py|FaceExample.py|
