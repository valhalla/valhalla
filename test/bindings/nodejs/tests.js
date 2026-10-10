const test = require('node:test');
const assert = require('node:assert').strict;
const path = require('node:path');
// NODE_PATH is set to the build directory containing this file
const valhalla = require('valhalla_node.node');
const fs = require('node:fs');
const config = fs.readFileSync(path.join(__dirname, 'valhalla.json'), 'utf8');


function hasCyrillic(text) {
  return /[\u0400-\u04FF]/.test(text);
}

test('variables', () => {
  assert.ok(valhalla.VALHALLA_VERSION, 'VALHALLA_VERSION is not defined');
  assert.ok(valhalla.ValhallaError, 'ValhallaError is not exported');
});

test('ValhallaError', async () => {
  const actor = new valhalla.Actor(config);

  const query = {
    locations: [
      { lat: 0.0, lon: 0.0 },
      { lat: 0.1, lon: 0.1 }
    ],
    costing: "auto"
  };

  try {
    await actor.route(JSON.stringify(query));
    assert.fail('Expected ValhallaError to be thrown');
  } catch (e) {
    // Should be a ValhallaError instance
    assert.ok(e instanceof valhalla.ValhallaError, `Expected ValhallaError, got ${e.constructor.name}`);
    // Should also be an Error instance (prototype chain)
    assert.ok(e instanceof Error, 'ValhallaError should be instanceof Error');
    // Structured fields from valhalla_exception_t
    assert.equal(e.code, 171);
    assert.equal(e.httpCode, 400);
    assert.equal(e.message, 'No suitable edges near location');
    assert.equal(e.httpMessage, 'Bad Request');
  }
});

test('actor', async(t) => {

  const actor = new valhalla.Actor(config);

  await t.test('route', async () => {
    const query = {
      locations: [
        { lat: 52.08813, lon: 5.03231 },
        { lat: 52.09987, lon: 5.14913 }
      ],
      costing: "bicycle",
      directions_options: { language: "bg-BG" }
    };

    const result = await actor.route(JSON.stringify(query));
    const route = JSON.parse(result);

    assert.ok('trip' in route);
    assert.ok('units' in route.trip);
    assert.equal(route.trip.units, 'kilometers');
    assert.ok('summary' in route.trip);
    assert.ok('length' in route.trip.summary);
    assert.ok(route.trip.summary.length > 0.7);
    assert.ok('legs' in route.trip);
    assert.ok(route.trip.legs.length > 0);
    assert.ok('maneuvers' in route.trip.legs[0]);
    assert.ok(route.trip.legs[0].maneuvers.length > 0);
    assert.ok('instruction' in route.trip.legs[0].maneuvers[0]);
    assert.ok(hasCyrillic(route.trip.legs[0].maneuvers[0].instruction));
  });

  await t.test('isochrone', async () => {
    const query = {
      locations: [
        { lat: 52.08813, lon: 5.03231 }
      ],
      costing: "pedestrian",
      contours: [
        { time: 1 },
        { time: 5 },
        { distance: 1 },
        { distance: 5 }
      ],
      show_locations: true
    };

    const result = await actor.isochrone(JSON.stringify(query));
    const iso = JSON.parse(result);

    // 4 isochrones and 2 point layers
    assert.equal(iso.features.length, 6);
  });

  await t.test('tile', async () => {
    // Utrecht center tile coordinates (52.08778°N, 5.13142°E at zoom 14)
    const query = {
      'tile': {
        z: 14,
        x: 8425,
        y: 5405
      }
    };

    const buf = await actor.tile(JSON.stringify(query));
    
    // Verify it's a Buffer
    assert.ok(Buffer.isBuffer(buf), 'tile() should return a Buffer');
    
    // Verify reasonable size (at least 100 bytes, typically KB range for MVT)
    assert.ok(
      buf.length >= 100, 
      `Tile buffer should be at least 100 bytes, got ${buf.length}`
    );
  });

  // we utilize NodeJS's thread pool to process requests in parallel, this test verifies there are no race conditions
  await t.test('100 parallel identical route requests', async () => {
    const query = {
      locations: [
        { lat: 52.08813, lon: 5.03231 },
        { lat: 52.09987, lon: 5.14913 }
      ],
      costing: "bicycle",
      directions_options: { language: "en-US" }
    };

    const queryString = JSON.stringify(query);
    
    // Send 100 identical requests in parallel
    const promises = Array.from({ length: 100 }, () => actor.route(queryString));
    const results = await Promise.all(promises);

    // Parse all results
    const parsedResults = results.map(result => JSON.parse(result));

    // Verify all results are the same by comparing with the first result
    const firstResult = parsedResults[0];
    
    for (let i = 1; i < parsedResults.length; i++) {
      // Compare the entire result as JSON string for strict equality
      assert.equal(JSON.stringify(parsedResults[i]), JSON.stringify(firstResult),
        `Result ${i} differs from first result`);
    }
  });
});
test('actor endpoints', async (t) => {
  const actor = new valhalla.Actor(config);
  const a = { lat: 52.08813, lon: 5.03231 };
  const b = { lat: 52.09987, lon: 5.14913 };
  const c = { lat: 52.0938, lon: 5.1194 };
  const route = JSON.parse(await actor.route(JSON.stringify({ locations: [a, b], costing: 'auto' })));
  const shape = route.trip.legs[0].shape;

  // method -> [request, key expected in the response]
  const cases = {
    locate: [{ locations: [a], costing: 'auto' }, null],
    matrix: [{ sources: [a], targets: [b, c], costing: 'auto' }, 'sources_to_targets'],
    optimizedRoute: [{ locations: [a, c, b], costing: 'auto' }, 'trip'],
    traceRoute: [{ encoded_polyline: shape, costing: 'auto', shape_match: 'edge_walk' }, 'trip'],
    traceAttributes: [{ encoded_polyline: shape, costing: 'auto', shape_match: 'edge_walk' }, 'edges'],
    height: [{ shape: [a, b] }, 'height'],
    transitAvailable: [{ locations: [{ ...a, radius: 1000 }] }, null],
    expansion: [{ locations: [a, b], costing: 'auto', action: 'route' }, 'features'],
    centroid: [{ locations: [a, b], costing: 'auto' }, 'trip'],
    status: [{}, 'version'],
  };
  for (const [method, [query, key]] of Object.entries(cases)) {
    await t.test(method, async () => {
      const res = JSON.parse(await actor[method](JSON.stringify(query)));
      if (key === null) {
        assert.ok(Array.isArray(res));
        assert.equal(res.length, query.locations.length);
      } else {
        assert.ok(key in res, `${key} missing from ${method} response`);
      }
    });
  }

  await t.test('binary result', async () => {
    const query = JSON.stringify({ locations: [a, b], costing: 'auto', format: 'pbf' });
    const json = await actor.route(query);
    assert.equal(typeof json, 'string');
    const buf = await actor.route(query, true);
    assert.ok(Buffer.isBuffer(buf));
    assert.ok(buf.length > 0);
  });

  await t.test('argument validation', () => {
    assert.throws(() => new valhalla.Actor(), TypeError);
    assert.throws(() => new valhalla.Actor('{not json'), /Failed to parse config/);
    assert.throws(() => actor.route(), TypeError);
    assert.throws(() => actor.route({}), TypeError);
    assert.throws(() => actor.tile(), TypeError);
  });
});

test('GraphId', () => {
  const { GraphId } = valhalla;

  const gid = new GraphId(421920, 2, 20);
  assert.equal(gid.tileid(), 421920);
  assert.equal(gid.level(), 2);
  assert.equal(gid.id(), 20);
  assert.equal(gid.value, 674464002);
  assert.ok(gid.is_valid());
  assert.equal(new GraphId().is_valid(), false);

  // every constructor yields the same id
  assert.ok(new GraphId('2/421920/20').equals(gid));
  assert.ok(new GraphId(674464002).equals(gid));
  assert.ok(new GraphId(674464002n).equals(gid));

  assert.ok(gid.tile_base().equals(new GraphId(421920, 2, 0)));
  assert.equal(gid.tile_value(), new GraphId(421920, 2, 0).value);
  assert.ok(gid.add(2).equals(new GraphId(421920, 2, 22)));
  assert.equal(gid.id(), 20);

  assert.equal(gid.equals(new GraphId(421920, 2, 21)), false);
  assert.equal(gid.equals({}), false);
  assert.equal(gid.equals(), false);

  assert.equal(gid.toString(), '2/421920/20');
  assert.deepEqual(JSON.parse(JSON.stringify(gid)),
    { level: 2, tileid: 421920, id: 20, value: 674464002 });

  assert.throws(() => new GraphId(true), TypeError);
  assert.throws(() => new GraphId(2n ** 64n), /Value too large/);
  assert.throws(() => new GraphId('1', 2, 3), TypeError);
  assert.throws(() => new GraphId(1, 2), TypeError);
  assert.throws(() => new GraphId('invalid'), Error);
  assert.throws(() => gid.add('1'), TypeError);
});

test('tile helpers', () => {
  const { GraphId } = valhalla;

  const gid = valhalla.getTileIdFromLonLat(2, [5.03231, 52.08813]);
  assert.equal(gid.level(), 2);
  assert.equal(gid.id(), 0);
  const [lon, lat] = valhalla.getTileBaseLonLat(gid);
  assert.ok(lon <= 5.03231 && lon > 5.03231 - 0.25);
  assert.ok(lat <= 52.08813 && lat > 52.08813 - 0.25);
  assert.ok(valhalla.getTileIdFromLonLat(2, [lon, lat]).equals(gid));

  assert.throws(() => valhalla.getTileBaseLonLat('2/0/0'), TypeError);
  assert.throws(() => valhalla.getTileBaseLonLat(new GraphId(0, 5, 0)), Error);
  assert.throws(() => valhalla.getTileIdFromLonLat(2, '5,52'), TypeError);
  assert.throws(() => valhalla.getTileIdFromLonLat(2, [5]), TypeError);
  assert.throws(() => valhalla.getTileIdFromLonLat(5, [5, 52]), /hierarchy levels/);
  assert.throws(() => valhalla.getTileIdFromLonLat(2, [200, 0]), /Invalid coordinate/);

  const bbox = [4.9, 52.0, 5.2, 52.2];
  const all = valhalla.getTileIdsFromBbox(...bbox);
  const local = valhalla.getTileIdsFromBbox(...bbox, [2]);
  assert.ok(local.length > 0);
  assert.ok(local.every((t) => t.level() === 2));
  assert.ok(all.length > local.length);
  assert.ok(local.some((t) => t.equals(gid)));
  assert.throws(() => valhalla.getTileIdsFromBbox(1, 2), TypeError);
  assert.throws(() => valhalla.getTileIdsFromBbox(200, 0, 201, 1), /Invalid coordinate/);
  assert.throws(() => valhalla.getTileIdsFromBbox(...bbox, [5]), /hierarchy levels/);

  // a ring around the same bbox covers the same level 2 tiles
  const ring = [[4.9, 52.0], [5.2, 52.0], [5.2, 52.2], [4.9, 52.2]];
  const fromRing = valhalla.getTileIdsFromRing(ring, [2]);
  assert.deepEqual(fromRing.map(String).sort(), local.map(String).sort());
  assert.equal(valhalla.getTileIdsFromRing(ring).length, all.length);
  assert.throws(() => valhalla.getTileIdsFromRing('ring'), TypeError);
  assert.throws(() => valhalla.getTileIdsFromRing([[0, 0], 1, [1, 1]]), TypeError);
  assert.throws(() => valhalla.getTileIdsFromRing([[0, 0], [1], [1, 1]]), TypeError);
  assert.throws(() => valhalla.getTileIdsFromRing([[0, 0], [1, 1]]), /at least 3/);
  assert.throws(() => valhalla.getTileIdsFromRing([[200, 0], [0, 0], [0, 1]]), /Invalid coordinate/);
});
