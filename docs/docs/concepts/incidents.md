# Incidents

Valhalla supports loading incidents in the form of pbf format valhalla-shaped tiles. That means that we have a proto definition which describes the tile format in such a way that code can be generated to access the data but also that the geograph area a tile covers matches those of the tiles that are natively supported in valhalla. Very briefly, the incident format has a vector of incident locations objects each of which specify the location along an edge in the corresponding graph tile where the incident occurs (begins/ends). There is also a vector of metadata which is indexed by the incident location information. Here's some pseudo code to give you the gist:

```c++
struct incident_tile{
  struct locations {
    uint32_t edge_index;   // the index of the edge in the corresponding graph tile
    float start_pct;       // the percent along the edge the incident starts
    float end_pct;         // the percent along the edge the incident ends
    size_t metadata_index; // the index into the metadata vector describing the incident
  };

  struct metadata {
    enum struct event_type:uint8_t {
      CONSTRUCTION = 0,
      // ... many more
    };
    std::string description;
    //... many more see the proto file
  };

  // locations of incidents along edges, sorted by edge id
  std::vector<location> locations;
  // metadata to go along with each incident
  std::vector<metadata> metadatas;
};

```

## Runtime support

If your request enables the `incidents` attribute filter and both incident tiles are available and the library is configured to use them, then incidents along paths will be attached to `TripLeg`s as they are created. These are then serialized into json as their own top level object.

## Tile access

Much like graph tile, incident tiles are loaded from a configured directory. Since the tiles are not fixed size (protobuf and unknown number of incidens per tile) the idea here is that we will need to continually refresh the tiles from the directory. There are a couple of assumptions that we are making:

* its unlikely that incidents change frequently
* its unlikely that there are large numbers of incidents
* incidents will not affect routing algorithms computations

The graphreader will then call a singleton whose job it is to give access to the available incident tiles from the directory. Access is used in triplegbuilder to associate incidents to the pbf route leg. The way these incidents are accessed is via binary search to find the corresponding incident for the edge index. We also mark a bit in the speed record for the edge to say that a given edge should have an incident. Upon seeing this bit set to true in the graphreader we will go look up the incident from the incident tile via the singleton.

## How are incidents refreshed

This deserves an explanation because its the trickier part. The model we use here is a class with a private constructor and one public static function. That function statically initializes a singleton instance of this class. The class's constructor spawns a thread that runs in the background. That thread is responsible for monitoring the filesystem for new incidents. When the thread is first started, the singleton waits for it to initialize (load all the incidents that are available).

The thread and the singleton instance communicate over the thread barrier by sharing a state object. The bulk of communications use atomic booleans (which are hopefully lock free on most implementations). You'll notice a mutex in there as well. The mutex is only used for synchronization of the tile cache, which is an `unordered_map`. Synchronization is needed because the `unordered_map` may reallocate when new tiles are added to it. Thus this synchronization is also conditional. When the tileset is static, ie. when its a memory mapped tar, we do not need to use the mutex to synchronize and instead preallocate all the slots needed for the cache. The state object shared between the threads is wrapped in a `shared_ptr`, from the code comments:

```c++
// we use a shared_ptr to wrap the state between the watcher thread and the main threads singleton
// instance. this gives the responsibility to the last living thread to deallocate the state object.
// if we didn't do this, and the watcher thread were still running when the singleton instance got
// destructed, then the watcher would be making use of a deallocated state object. this way, if the
// watcher is last to die it owns the lifetime of the state and if the singleton is the last to die
// it owns the lifetime of the state. note that we still need to use atomics inside the state as
// only the shared_ptr itself is thread safe, not the thing it points to
```

There are two modes for the incident loading singleton, one which does directory scans (`mjolnir.incident_dir` in the config), which on a modern ssd where changes are happening to the incident directory, takes 15 seconds for a planets worth of incident tiles. The second mode is a memory mapped log file which tells the timestamp when an incident tile was last changed rather than using mtime of the files on the filesystem. This can be configured with the `mjolnir.incident_log` config option and takes generally subsecond on modern ssds to complete for updates since it doesnt need to scan the whole directory.

Since there is only one thread (per process) who is in charge of updating incidents we need to be worried about the health of this thread. There is one other configuration options to do with the healthiness of this thread. This config option is called `mjolnir.max_incident_loading_latency` and controls how long a round of incident updates can take before we log an error that the update was latent.
