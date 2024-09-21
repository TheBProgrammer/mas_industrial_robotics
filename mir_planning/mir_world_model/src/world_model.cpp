/**
 * @brief A class that represents the world model of the atwork arena including
 * the robot and the objects.
 *
 */

#include <mir_world_model/world_model.hpp>

WorldModel::WorldModel()
{
  std::cout << "WorldModel created" << std::endl;
}

WorldModel::~WorldModel() {}

// ============================ Methods ============================
void 
WorldModel::addWorkstation(
  const std::string &name, 
  const std::string &type, 
  const double &height)
{
  mir_interfaces::msg::Workstation workstation;
  workstation.name = name;
  workstation.type = type;
  workstation.height = height;
  workstations_[name] = workstation; // add workstation to map
}

void 
WorldModel::removeWorkstation(
  const std::string &name)
{
  workstations_.erase(name);
}

void WorldModel::addObjectToWorkstation(
    const mir_interfaces::msg::ObjectList::SharedPtr object_list)
{
    ObjectVector objects_from_rgb;
    ObjectVector objects_from_pcl;

    std::string &workstation_name = object_list->workstation_name;
    ObjectVector &workstation_objects = workstations_[workstation_name].objects;

    getAllObjects(workstation_name, workstation_objects);

    for (const auto &object : object_list->objects)
    {
        // add object to respective source list
        if (object.database_id > 99)
        {
            objects_from_rgb.push_back(object);
        }
        else
        {
            objects_from_pcl.push_back(object);
        }
    }

    ObjectVector objects_combined;

    if (!objects_from_rgb.empty() && !objects_from_pcl.empty())
    {
        filterDuplicatesByDistance(objects_combined, objects_from_rgb, objects_from_pcl);
    }
    else if (objects_from_pcl.empty() && !objects_from_rgb.empty())
    {
        objects_combined = objects_from_rgb;
    }
    else if (objects_from_rgb.empty() && !objects_from_pcl.empty())
    {
        objects_combined = objects_from_pcl;
    }
    else
    {
        throw std::runtime_error("No objects found in object list");
    }

    // find duplicates between workstation_objects and objects_combined
    filterDuplicatesByDistance(workstation_objects, objects_combined);
}


void 
WorldModel::removeObjectFromWorkstation(
  const std::string &workstation_name,
  const std::string &object_name)
{ 
  // remove object from workstation
  ObjectVector &workstation_objects = workstations_[workstation_name].objects;
  for (auto it = workstation_objects.begin(); it != workstation_objects.end(); ++it)
  {
    if (it->name == object_name)
    {
      workstation_objects.erase(it);
      break;
    }
  }
}


void
WorldModel::getAllObjects(
  const std::string &workstation_name, ObjectVector &objects)
{
  objects = workstations_[workstation_name].objects;
}

void 
WorldModel::getWorkstationObject(
    const std::string &workstation_name,
    const std::string &object_name,
    mir_interfaces::msg::Object &object)
{
  ObjectVector &workstation_objects = workstations_[workstation_name].objects;
  for (auto it = workstation_objects.begin(); it != workstation_objects.end(); ++it)
  {
    if (it->name == object_name)
    {
      object = *it;
      workstation_objects.erase(it);
      break;
    }
  }
}

void 
WorldModel::getAllWorkstations(std::vector<mir_interfaces::msg::Workstation> &workstations)
{
  for (auto const& workstation : workstations_)
  {
    workstations.push_back(workstation.second);
  }
}



void 
WorldModel::getWorkstationHeight(
  const std::string &workstation_name, double &height)
{
  height = workstations_[workstation_name].height;
}

void 
WorldModel::filterDuplicatesByDistance(
  ObjectVector &objectlist_combined,
  const ObjectVector &objectlist_primary,
  const ObjectVector &objectlist_secondary)
{
  objectlist_combined.clear();
  for (auto const& object_secondary : objectlist_secondary)
  {
    bool duplicate = false;
    for (auto const& object_primary : objectlist_primary)
    {
      float distance = sqrt(
        pow(object_primary.pose.pose.position.x - object_secondary.pose.pose.position.x, 2) +
        pow(object_primary.pose.pose.position.y - object_secondary.pose.pose.position.y, 2));

      if (distance < 0.02)
      {
        duplicate = true;
        break;
      }

    }
    if (!duplicate)
    {
      objectlist_combined.push_back(object_secondary);
    }
  }
  // return the objectlist_combined + objectlist_primary
  objectlist_combined.insert(
    objectlist_combined.end(), 
    objectlist_primary.begin(), 
    objectlist_primary.end());

}

void 
WorldModel::filterDuplicatesByDistance(
  ObjectVector &objectlist_primary,
  ObjectVector &objectlist_secondary)
{
  //log
  std::cout << "Filtering duplicates by distance - 2param ver" << std::endl;

  // write code to iterate through the objectlist_secondary and remove duplicates from objectlist_primary
  for (auto it = objectlist_secondary.begin(); it != objectlist_secondary.end(); ++it)
  {
    for (auto it = objectlist_primary.begin(); it != objectlist_primary.end(); ++it)
    {
      float distance = sqrt(
        pow(it->pose.pose.position.x - it->pose.pose.position.x, 2) +
        pow(it->pose.pose.position.y - it->pose.pose.position.y, 2));

      if (distance < 0.02)
      {
        //log
        std::cout << "Duplicate found" << std::endl;
        objectlist_primary.erase(it);
        break;
      }
    }
  }

  //add all objects from objectlist_secondary to objectlist_primary
  objectlist_primary.insert(
    objectlist_primary.end(), 
    objectlist_secondary.begin(), 
    objectlist_secondary.end());
}