#ifndef __EDGE_ID_MANAGER_HPP__
#define __EDGE_ID_MANAGER_HPP__

#include <map>
#include <mutex>

/**
 * \class EdgeIDManager
 * 
 * \brief Manages issuing of edge IDs for the graph optimiser
 *
 * g2o requires that each edge be assigned a unique ID. This class
 * centralised the creation and assignment of IDs. It is actually
 * based on a simple counter which increments. We make it possible
 * to assign a set of "ID groups". Each ID group can have its own
 * start value, and increments from there.
 */
class EdgeIDManager
{
public:

  /**
   * Issue a new ID. Each ID increases monotonically.
   *
   * \param idGroup The ID group that the ID is to be assigned from. The default, 0, is the default ID group
   *
   * \return The newly assigned ID. This is -1 if the idGroup does not exist.
   */
  static int issueID(int idGroup = 0);

  /**
   * Return how many IDs have been assigned. This does not assign
   * a new edge ID.
   *
   * \param idGroup The ID group that the ID is to be assigned from. The default, 0, is the default ID group
   *
   * \return The number of assigned edge IDs, -1 if the idGroup does not exist.
   */
  static int getNumberOfIssuedIDs(int idGroup = 0);

  /**
   * Create a new ID group
   *
   * \return The number of assigned edge IDs.
   */
  static bool createIDGroup(int newIDGroup, int startIDNumber = -1);

  /**
   * Return the start ID of the ID group
   *
   * \return The number of assigned edge IDs.
   */
  //static int getStartID(int idGroup = 0);

private:
  /**
   * The private constructor
   */
  EdgeIDManager();

  /**
   * The map used to store the count of assigned IDs
   */
  static std::map<int, int> s_ID;

  /**
   * The mutex used to ensure thread safety.
   */
  static std::mutex s_mutex;
};

#endif // __EDGE_ID_MANAGER_HPP__
