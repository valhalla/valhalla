#ifndef __graphid_h__
#define __graphid_h__

namespace valhalla{
namespace baldr{

/**
 * Identifier of a node or an edge within the graph. Includes a hierarchy
 * and a unique identifier within the hierarchy.
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
   * @param  hierarchy   hierarchy ID
   * @param  id         Unique identifier within the hierarchy.
   */
  GraphId(const unsigned int hierarchy, const unsigned int id);

  /**
   * Copy constructor.
   * @param  g   GraphId to copy
   */
  GraphId(const GraphId& g);

  /**
   * Gets the hierarchy number.
   * @return   Returns the hierarchy.
   */
  unsigned int Hierarchy() const;

  /**
   * Gets the identifier within the hierarchy.
   * @return   Returns the unique identifier within the hierarchy.
   */
  unsigned int Id() const;

  /**
   * Test if this is a valid graph element. Invalid elements have id == 0.
   * @return   Returns true if valid, false if not.
   */
  bool IsValid() const;

  /**
   * Convenience method to set individual greph Id elements.
   * @param  hierarchy  hierarchy ID
   * @param  id         Unique identifier within the hierarchy
   */
  void Set(const unsigned int hierarchy, const unsigned int id);

  /**
   * Post increments the id.
   */
  void operator ++(int);

  // TODO - do we need equality operator or does C++ take care of this?

 protected:
  struct Fields {
    unsigned int id        : 29;
    unsigned int hierarchy : 3;
  };
  Fields graphid;
};

}
}

#endif // __graphid_h__
