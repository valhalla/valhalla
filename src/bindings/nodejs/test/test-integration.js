const { describe, test } = require('node:test');
const assert = require('node:assert');
const valhalla = require('@valhallajs/valhallajs');

describe('Valhalla Integration Tests', () => {
  test('should generate routes using built tiles', async () => {
    const configFile = process.env.VALHALLA_CONFIG_FILE;
    if (!configFile) {
      throw new Error('VALHALLA_CONFIG_FILE environment variable is required');
    }
    const actor = await valhalla.Actor.fromConfigFile(configFile);
    assert.ok(actor, 'Actor should be initialized');

    const routeRequest = {
      locations: [
        { lat: 51.572386, lon: -0.139549 },
        { lat: 51.570000, lon: -0.138000 }
      ],
      costing: "auto",
      directions_options: {
        units: "kilometers"
      }
    };

    const result = await actor.route(routeRequest);
    assert.ok(result, 'Route result should be returned');
    assert.ok(result.trip, 'Route result should contain trip');
    assert.ok(result.trip.legs, 'Trip should contain legs');
    assert.ok(result.trip.legs.length > 0, 'Trip should have at least one leg');

    const trip = result.trip;
    const leg = trip.legs[0];

    assert.ok(leg.summary.length > 0, 'Route distance should be greater than 0');
  });

  test('expansion with pbf format returns Buffer', async () => {
    const configFile = process.env.VALHALLA_CONFIG_FILE;
    if (!configFile) {
      throw new Error('VALHALLA_CONFIG_FILE environment variable is required');
    }
    const actor = await valhalla.Actor.fromConfigFile(configFile);

    const expansionRequest = {
      locations: [
        { lat: 51.572386, lon: -0.139549 }
      ],
      costing: "auto",
      action: "isochrone",
      contours: [{ "time": 60 }],
      format: "pbf"
    };

    const result = await actor.expansion(expansionRequest);
    assert.ok(Buffer.isBuffer(result), 'PBF format should return a Buffer');
    assert.ok(result.length > 0, 'Buffer should not be empty');
  });

  test('isochrone with pbf format returns Buffer', async () => {
    const configFile = process.env.VALHALLA_CONFIG_FILE;
    if (!configFile) {
      throw new Error('VALHALLA_CONFIG_FILE environment variable is required');
    }
    const actor = await valhalla.Actor.fromConfigFile(configFile);

    const isochroneRequest = {
      locations: [
        { lat: 51.572386, lon: -0.139549 }
      ],
      costing: "auto",
      action: "isochrone",
      contours: [{ "time": 60 }],
      format: "pbf"
    };

    const result = await actor.isochrone(isochroneRequest);
    assert.ok(Buffer.isBuffer(result), 'PBF format should return a Buffer');
    assert.ok(result.length > 0, 'Buffer should not be empty');
  });

  test('expansion with json format returns object', async () => {
    const configFile = process.env.VALHALLA_CONFIG_FILE;
    if (!configFile) {
      throw new Error('VALHALLA_CONFIG_FILE environment variable is required');
    }
    const actor = await valhalla.Actor.fromConfigFile(configFile);

    const expansionRequest = {
      locations: [
        { lat: 51.572386, lon: -0.139549 }
      ],
      costing: "auto",
      action: "isochrone",
      contours: [{ "time": 60 }],
      format: "json"
    };

    const result = await actor.expansion(expansionRequest);
    assert.ok(typeof result === 'object', 'JSON format should return an object');
    assert.ok(!Buffer.isBuffer(result), 'JSON format should not return a Buffer');
  });

  test('isochrone with json format returns object', async () => {
    const configFile = process.env.VALHALLA_CONFIG_FILE;
    if (!configFile) {
      throw new Error('VALHALLA_CONFIG_FILE environment variable is required');
    }
    const actor = await valhalla.Actor.fromConfigFile(configFile);

    const isochroneRequest = {
      locations: [
        { lat: 51.572386, lon: -0.139549 }
      ],
      costing: "auto",
      action: "isochrone",
      contours: [{ "time": 60 }],
      format: "json"
    };

    const result = await actor.isochrone(isochroneRequest);
    assert.ok(typeof result === 'object', 'JSON format should return an object');
    assert.ok(!Buffer.isBuffer(result), 'JSON format should not return a Buffer');
  });

  test('route with pbf format returns Buffer', async () => {
    const configFile = process.env.VALHALLA_CONFIG_FILE;
    if (!configFile) {
      throw new Error('VALHALLA_CONFIG_FILE environment variable is required');
    }
    const actor = await valhalla.Actor.fromConfigFile(configFile);

    const routeRequest = {
      locations: [
        { lat: 51.572386, lon: -0.139549 },
        { lat: 51.570000, lon: -0.138000 }
      ],
      costing: "auto",
      format: "pbf"
    };

    const result = await actor.route(routeRequest);
    assert.ok(Buffer.isBuffer(result), 'PBF format should return a Buffer');
    assert.ok(result.length > 0, 'Buffer should not be empty');
  });

  test('route with json format returns object', async () => {
    const configFile = process.env.VALHALLA_CONFIG_FILE;
    if (!configFile) {
      throw new Error('VALHALLA_CONFIG_FILE environment variable is required');
    }
    const actor = await valhalla.Actor.fromConfigFile(configFile);

    const routeRequest = {
      locations: [
        { lat: 51.572386, lon: -0.139549 },
        { lat: 51.570000, lon: -0.138000 }
      ],
      costing: "auto",
      format: "json"
    };

    const result = await actor.route(routeRequest);
    assert.ok(typeof result === 'object', 'JSON format should return an object');
    assert.ok(!Buffer.isBuffer(result), 'JSON format should not return a Buffer');
    assert.ok(result.trip, 'Result should contain trip');
  });

  test('matrix with pbf format returns Buffer', async () => {
    const configFile = process.env.VALHALLA_CONFIG_FILE;
    if (!configFile) {
      throw new Error('VALHALLA_CONFIG_FILE environment variable is required');
    }
    const actor = await valhalla.Actor.fromConfigFile(configFile);

    const matrixRequest = {
      sources: [
        { lat: 51.572386, lon: -0.139549 },
        { lat: 51.570000, lon: -0.138000 }
      ],
      targets: [
        { lat: 51.572386, lon: -0.139549 },
        { lat: 51.570000, lon: -0.138000 }
      ],
      costing: "auto",
      format: "pbf"
    };

    const result = await actor.matrix(matrixRequest);
    assert.ok(Buffer.isBuffer(result), 'PBF format should return a Buffer');
    assert.ok(result.length > 0, 'Buffer should not be empty');
  });

  test('matrix with json format returns object', async () => {
    const configFile = process.env.VALHALLA_CONFIG_FILE;
    if (!configFile) {
      throw new Error('VALHALLA_CONFIG_FILE environment variable is required');
    }
    const actor = await valhalla.Actor.fromConfigFile(configFile);

    const matrixRequest = {
      sources: [
        { lat: 51.572386, lon: -0.139549 },
        { lat: 51.570000, lon: -0.138000 }
      ],
      targets: [
        { lat: 51.572386, lon: -0.139549 },
        { lat: 51.570000, lon: -0.138000 }
      ],
      costing: "auto",
      format: "json"
    };

    const result = await actor.matrix(matrixRequest);
    assert.ok(typeof result === 'object', 'JSON format should return an object');
    assert.ok(!Buffer.isBuffer(result), 'JSON format should not return a Buffer');
    assert.ok(Array.isArray(result.sources_to_targets), 'Result should contain sources_to_targets array');
  });

  test('traceRoute with pbf format returns Buffer', async () => {
    const configFile = process.env.VALHALLA_CONFIG_FILE;
    if (!configFile) {
      throw new Error('VALHALLA_CONFIG_FILE environment variable is required');
    }
    const actor = await valhalla.Actor.fromConfigFile(configFile);

    const traceRouteRequest = {
      shape: [
        { lat: 51.572386, lon: -0.139549 },
        { lat: 51.571000, lon: -0.138500 },
        { lat: 51.570000, lon: -0.138000 }
      ],
      costing: "auto",
      shape_match: "map_snap",
      format: "pbf"
    };

    const result = await actor.traceRoute(traceRouteRequest);
    assert.ok(Buffer.isBuffer(result), 'PBF format should return a Buffer');
    assert.ok(result.length > 0, 'Buffer should not be empty');
  });

  test('traceRoute with json format returns object', async () => {
    const configFile = process.env.VALHALLA_CONFIG_FILE;
    if (!configFile) {
      throw new Error('VALHALLA_CONFIG_FILE environment variable is required');
    }
    const actor = await valhalla.Actor.fromConfigFile(configFile);

    const traceRouteRequest = {
      shape: [
        { lat: 51.572386, lon: -0.139549 },
        { lat: 51.571000, lon: -0.138500 },
        { lat: 51.570000, lon: -0.138000 }
      ],
      costing: "auto",
      shape_match: "map_snap",
      format: "json"
    };

    const result = await actor.traceRoute(traceRouteRequest);
    assert.ok(typeof result === 'object', 'JSON format should return an object');
    assert.ok(!Buffer.isBuffer(result), 'JSON format should not return a Buffer');
    assert.ok(result.trip, 'Result should contain trip');
  });

  test('traceAttributes with pbf format returns Buffer', async () => {
    const configFile = process.env.VALHALLA_CONFIG_FILE;
    if (!configFile) {
      throw new Error('VALHALLA_CONFIG_FILE environment variable is required');
    }
    const actor = await valhalla.Actor.fromConfigFile(configFile);

    const traceAttributesRequest = {
      shape: [
        { lat: 51.572386, lon: -0.139549 },
        { lat: 51.571000, lon: -0.138500 },
        { lat: 51.570000, lon: -0.138000 }
      ],
      costing: "auto",
      shape_match: "map_snap",
      format: "pbf"
    };

    const result = await actor.traceAttributes(traceAttributesRequest);
    assert.ok(Buffer.isBuffer(result), 'PBF format should return a Buffer');
    assert.ok(result.length > 0, 'Buffer should not be empty');
  });

  test('traceAttributes with json format returns object', async () => {
    const configFile = process.env.VALHALLA_CONFIG_FILE;
    if (!configFile) {
      throw new Error('VALHALLA_CONFIG_FILE environment variable is required');
    }
    const actor = await valhalla.Actor.fromConfigFile(configFile);

    const traceAttributesRequest = {
      shape: [
        { lat: 51.572386, lon: -0.139549 },
        { lat: 51.571000, lon: -0.138500 },
        { lat: 51.570000, lon: -0.138000 }
      ],
      costing: "auto",
      shape_match: "map_snap",
      format: "json"
    };

    const result = await actor.traceAttributes(traceAttributesRequest);
    assert.ok(typeof result === 'object', 'JSON format should return an object');
    assert.ok(!Buffer.isBuffer(result), 'JSON format should not return a Buffer');
    assert.ok(result.edges, 'Result should contain edges');
  });
});

