#ifndef VALHALLA_BALDR_GRAPHID_H_
#define VALHALLA_BALDR_GRAPHID_H_

namespace valhalla{
namespace baldr{

/**
 * Identifier of a node or an edge within the graph. Includes a hierarchy
 * level and a unique identifier within the level.
 * @author  David W. Nesbitt
 */
class GraphId {
 public:
  /**public:
   * Default constructor
   */
  GraphId();

  /**
   * Constructor.
   * @param  level  Hierarchy level
   * @param  id     Unique identifier within the level.
   */
  GraphId(const unsigned int level, const unsigned int id);

  /**
   * Copy constructor.
   * @param  g   GraphId to copy
   */
  GraphId(const GraphId& g);

  /**
   * Gets the hierarchy level.
   * @return   Returns the level.
   */
  unsigned int Level() const;

  /**
   * Gets the identifier within the hierarchy level.
   * @return   Returns the unique identifier within the level.
   */
  unsigned int Id() const;

  /**
   * Test if this is a valid graph element. Invalid elements have id == 0.
   * @return   Returns true if valid, false if not.
   */
  bool IsValid() const;

  /**
   * Convenience method to set individual graph Id elements.
   * @param  level  Hierarchy level
   * @param  id     Unique identifier within the level
   */
  void Set(const unsigned int level, const unsigned int id);

  /**
   * Post increments the id.
   */
  void operator ++(int);

  /**
   * Less than operator for sorting.
   * @param  other  Other graph Id for comparison.
   * @return  Returns true if this GraphId is less than the other.
   */
  bool operator < (const GraphId& other) const;

 protected:
  struct Fields {
    unsigned int id    : 29;
    unsigned int level : 3;
  };
  Fields graphid_;
};

}
}

#endif // VALHALLA_BALDR_GRAPHID_H_
