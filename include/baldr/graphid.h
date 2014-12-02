#ifndef __graphid_h__
#define __graphid_h__

// Maximum of 8 (0-7) graph hierarchies are supported.
constexpr unsigned int kMaxGraphHierarchy = 7;

// Maximum unique identifier within a graph hierarchy (~536 million)
constexpr unsigned int kMaxGraphId = 536870911;

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
  GraphId() {
    // TODO - better way to initialize?
    Set(0, 0);
  }

  /**
   * Constructor.
   * @param  hierarchy   hierarchy ID
   * @param  id         Unique identifier within the hierarchy.
   */
  GraphId(const unsigned int hierarchy, const unsigned int id) {
    Set(hierarchy, id);
  }

  /**
   * Copy constructor.
   * @param  g   GraphId to copy
   */
  GraphId(const GraphId& g) {
    Set(g.Hierarchy(), g.Id());
  }

  /**
   * Gets the hierarchy number.
   * @return   Returns the hierarchy.
   */
  unsigned int Hierarchy() const {
    return graphid.hierarchy;
  }

  /**
   * Gets the identifier within the hierarchy.
   * @return   Returns the unique identifier within the hierarchy.
   */
  unsigned int Id() const {
    return graphid.id;
  }

  /**
   * Test if this is a valid graph element. Invalid elements have id == 0.
   * @return   Returns true if valid, false if not.
   */
  bool IsValid() const {
    return (graphid.id > 0);
  }

  /**
   * Convenience method to set individual greph Id elements.
   * @param  hierarchy  hierarchy ID
   * @param  id         Unique identifier within the hierarchy
   */
  void Set(const unsigned int hierarchy, const unsigned int id) {
    graphid.id = (id < kMaxGraphId ) ? id : 0;
    graphid.hierarchy = (hierarchy < kMaxGraphHierarchy) ? hierarchy : 0;
  }

  /**
   * Post increments the id.
   */
  void operator ++(int) {
    graphid.id++;
  }

  // TODO - do we need equality operator or does C++ take care of this?

 protected:
  struct Fields {
    unsigned int id        : 29;
    unsigned int hierarchy : 3;
  };
  Fields graphid;
};

#endif // __graphid_h__
