#Map Roulette Geojson Generation
To generate the geojson, you'll need to detect the cases, gather the required data, and define how the geojson should be created.

##Detection
You will have to design an algorithm that can detect the cases that you are looking for. Once you are able to detect the cases, you now need to capture the useful data.

For an example of existing detection please see the function [bool IsUnroutableNode](https://github.com/valhalla/mjolnir/blob/master/src/mjolnir/valhalla_build_statistics.cc#L190) in `valhalla_build_statistics.cc`.

##Gathering the Data
At a minimum, you will need some sort of geojson feature to display on the Map Roulette interface. This can be a point or linestring or a combination of them. At some point in the data capture you must capture a lat-long pair (PointLL) that is unique for each case you detect. This will be used later as the identifier in the geojson. You will need to modify the [`roulettedata`](https://github.com/valhalla/mjolnir/blob/master/src/mjolnir/statistics.cc#L304) struct to hold your new data.

##Geojson
