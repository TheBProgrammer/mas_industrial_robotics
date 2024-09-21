/**
 * @brief A class that represents the world model of the atwork arena including
 * the robot and the objects.
 */

#ifndef WORLD_MODEL_HPP
#define WORLD_MODEL_HPP

#include <ctime>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <cmath>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "rclcpp/time.hpp"

#include "mir_interfaces/msg/object_list.hpp"
#include "mir_interfaces/msg/object.hpp"
#include "mir_interfaces/msg/workstation.hpp"

class WorldModel
{
public:
  WorldModel();

  virtual ~WorldModel();

private:
  // ============================ Members ============================
  std::map<std::string, mir_interfaces::msg::Workstation> workstations_;

public:
  typedef std::vector<mir_interfaces::msg::Object> ObjectVector;
  // ============================ Methods ============================

  /**
   * @brief Adds a workstation to the world model.
   *
   * @param name Name of the workstation.
   * @param type Type of the workstation.
   * @param height Height of the workstation.
   */
  void addWorkstation(
    const std::string &name, 
    const std::string &type, 
    const double &height);

  /**
   * @brief Removes a workstation from the world model.
   * 
   * @param name Name of the workstation.
   */
  void removeWorkstation(
    const std::string &name);

  /**
   * @brief adds an object to a workstation
   * 
   * @param object_list list of objects including workstation_name, pose, id and name
  */
  void addObjectToWorkstation(
    const mir_interfaces::msg::ObjectList::SharedPtr object_list);

  /**
   * @brief removes an object from a workstation
   * 
   * @param workstation_name name of the workstation
   * @param object_name name of the object
  */
  void removeObjectFromWorkstation(
    const std::string &workstation_name,
    const std::string &object_name);

  /**
   * @brief gets all objects on a workstation
   * 
   * @param workstation_name name of the workstation
  */
  void getAllObjects(
    const std::string &workstation_name, ObjectVector &objects);

  /**
   * @brief gets an object from a workstation by name
   *        required for picking up object
   * 
   * @param workstation_name name of the workstation
   * @param object_name name of the object
   * @param object object to be returned
  */
  void getWorkstationObject(
    const std::string &workstation_name,
    const std::string &object_name,
    mir_interfaces::msg::Object &object);

  /**
   * @brief gets all workstations
  */
  void getAllWorkstations(
    std::vector<mir_interfaces::msg::Workstation> &workstations);

  /**
   * @brief gets the height of a workstation
   * 
   * @param workstation_name name of the workstation
  */
  void getWorkstationHeight(
    const std::string &workstation_name, double &height);

  /**
   * @brief removes duplicate objects from two lists based on their distance
   * if distance is smaller than 0.02m, the object from the secondary list is removed
   * 
   * @param objectlist_primary primary list of objects. This list will be modified.
   * @param objectlist_secondary secondary list of objects
   */
  void filterDuplicatesByDistance(
    ObjectVector &objectlist_primary,
    ObjectVector &objectlist_secondary);

  /**
   * @brief removes duplicate objects from two lists based on their distance
   * if distance is smaller than 0.02m, the object from the secondary list is removed
   * 
   * @param objectlist_combined combined list of objects. This list will be modified.
   * @param objectlist_primary primary list of objects
   * @param objectlist_secondary secondary list of objects
   */
  void filterDuplicatesByDistance(
    ObjectVector &objectlist_combined,
    const ObjectVector &objectlist_primary,
    const ObjectVector &objectlist_secondary);
};

#endif // WORLD_MODEL_HPP