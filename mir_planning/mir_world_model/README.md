# A class that represents the world model of the atwork arena including the robot and the objects.

## Requirements - WorldModel class description

### Objects
 - Workstations object - Can be N workstations in the arena - should be dynamic
   - Workstation name - string: WS01-WSN, RT01-RTN, CV01-CVN, SH01-SHN
   - Workstation type - can be of type: Normal, Shelf, Cavity, Rotating table
   - Workstation height - double: 0.0 - 1.0 (meters)
   - Object Ids - vector of object Ids on the workstation
 - Atwork object - Can be N objects in the arena - should be dynamic
   - Object name - string
   - Object Id - unique idenfier
   - Object pose - geometry_msgs::Pose
   - Workstation name - name of the workstation the object is on
   - Object added time - ros::Time
### Methods 
- addWorkstation - adds a workstation to the world model
- removeWorkstation - Removes a workstation from the world model
- addObjectToWorkstation - adds an object to a workstation
- removeObjectFromWorkstation - removes an object from a workstation
- getWorkstationObjectIds - gets IDs of all objects on a workstation
- getWorkstationObjects - gets names of all objects on a workstation
- getAllWorkstations - gets all workstation objects
- getWorkstationHeight - gets the height of a workstation