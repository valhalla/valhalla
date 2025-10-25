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
});