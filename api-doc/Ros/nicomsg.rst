nicomsg
********

The **nicomsg** package contains common messages used in different parts of the API.

Message name convention
#######################

The message name contains the type of the parameter. The type is shortened as following:

+-------------------------------------------------+------------------------------+
| Type                                            | Short name                   |
+=================================================+==============================+
| Header                                          | h                            |
+-------------------------------------------------+------------------------------+
| float64                                         | f                            |
+-------------------------------------------------+------------------------------+
| int32                                           | i                            |
+-------------------------------------------------+------------------------------+
| string                                          | s                            |
+-------------------------------------------------+------------------------------+
| array                                           | a<type>a                     |
+-------------------------------------------------+------------------------------+

A message with the parameters:
 * string
 * float64
 * int32

will be called *nicomsg/sfi.msg*. The parameter name is called **paramX**, where **X** is the position of the parameter.

If an array is used, the type of its content should be surrounded by two a's (e.g. *asa.msg* for a string array parameter)

Service name convention
#######################

The naming of the services should follow the following rules (based on the standard ROS API):
 * Use CamelCase
 * The name should describe the purpose of the service
