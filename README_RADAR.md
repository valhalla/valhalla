
# README Radar



Read me for Radar Fork.
Radar extends Valhalla in two main ways for live traffic.



1. Building various files needed for live traffic. Some of these are already built into Valhalla but we made some modifications
  a. Modified ways_to_edges which builds ways_to_edges.db. Now it includes shortcuts, direction, and length of edge
  b. Added traffic_tile_offset which builds tile offsets from a traffic.tar

1. Building a NAPI extension that is used by https://github.com/radarlabs/valhalla-traffic-builder to ingest traffic into the traffic.tar



## Compiling
To Build locally follow along to https://valhalla.github.io/valhalla/building/

You can pull files prebuilt tile.tar/index.tar files from S3 instead of building new ones

### Compiling NAPI
If you want to modify and test the NAPI code you will also need node js and NAPI
```bash
# Assuming you have installed node from nvm
node --version
npm install -g node-addon-api
export NODE_VERSION=v20.19.0 # or your node version


# will build to ./build
cmake -B build -DCMAKE_BUILD_TYPE=Release -DENABLE_NAPI_BINDINGS=ON
make -C build -j$(sysctl -n hw.physicalcpu)
sudo make -C build install
```
This will build  `node_bindings.node` to `/build`. See https://github.com/radarlabs/valhalla-traffic-builder for details on how to use


## Used by Radar
**Running Valhalla**
```


```

**Generating Ways to Edges**
```
```

**Ingesting Historic Traffic**
```
```


**Generating Traffic.tar**
```

```

**Generating traffic tile offset**
```
```

**Other useful tools**
```
# Find tile offset

# Re-add back in modifying traffic.tar without NAPI

```
