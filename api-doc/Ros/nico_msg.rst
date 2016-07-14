nico_msg
********

The **nico_msg** package contains common messages used in different parts of the API.

Message name convention
#######################

The message name contains the type of the parameter. The type is shortened as following:

+-------------------------------------------------+------------------------------+
| Type                                            | Short name                   |
+=================================================+==============================+
| float64                                         | f                            |
+-------------------------------------------------+------------------------------+
| int32                                           | i                            |
+-------------------------------------------------+------------------------------+
| string                                          | s                            |
+-------------------------------------------------+------------------------------+

A message with the parameters:
 * string
 * float64
 * int32

will be called *nico_msg/sfi*. The parameter name is called **paramX**, where **X** is the position of the parameter.
 

Service name convention
#######################

The naming of the services should follow the following rules (based on the standard ROS API):
 * Use CamelCase
 * The name should describe the purpose of the service