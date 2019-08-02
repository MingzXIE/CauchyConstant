#include "EdgeIDManager.hpp"
#include <iostream>

using namespace ::std;

map<int,int> EdgeIDManager::s_ID;

mutex EdgeIDManager::s_mutex;

int EdgeIDManager::issueID(int idGroup)
{
  lock_guard<mutex> lock(s_mutex);
  
  if (s_ID.find(idGroup) == s_ID.end())
    {
      return -1;
    }
  int newID = s_ID[idGroup];
  s_ID[idGroup] = newID + 1;
  //  ROS_DEBUG_STREAM_NAMED("EdgeIDManager", __FUNCTION__ <<  ": Issuing Edge ID " << newID << " on ID group " << idGroup);
  return newID;
}

int EdgeIDManager::getNumberOfIssuedIDs(int idGroup)
{
  lock_guard<mutex> lock(s_mutex);
  if (s_ID.find(idGroup) == s_ID.end())
    {
      return -1;
    }
  return s_ID[idGroup];
}

bool EdgeIDManager::createIDGroup(int newIDGroup, int startIDNumber)
{
  lock_guard<mutex> lock(s_mutex);

  //  ROS_DEBUG_STREAM_NAMED("EdgeIDManager", __FUNCTION__ <<  ": creating newIDGroup " << newIDGroup << " with startIDNumber " << startIDNumber);

  if (startIDNumber < 0)
    {
      return false;
    }

    if (s_ID.find(newIDGroup) != s_ID.end())
    {
      return false;
    }
    s_ID[newIDGroup] = startIDNumber;
    return true;
}

// Magic gubbins to start the ID manager off with default values

struct StartUpEdgeIDManager
{
  StartUpEdgeIDManager()
  {
    EdgeIDManager::createIDGroup(0, 0);
  }
};

static StartUpEdgeIDManager s_startUpEdgeIDManagerProxy;
