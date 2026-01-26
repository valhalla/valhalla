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
        { lat: 52.08957210411523, lon: 5.1103487316804985 }, // Utrecht Central Station
        { lat: 52.09085032726166, lon: 5.121582587112782 }  // Dom Tower
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
        { lat: 52.08957210411523, lon: 5.1103487316804985 }
      ],
      costing: "auto",
      action: "isochrone",
      contours: [{ time: 15 }],
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
        { lat: 52.08957210411523, lon: 5.1103487316804985 }
      ],
      costing: "auto",
      contours: [{ time: 15 }],
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
        { lat: 52.08957210411523, lon: 5.1103487316804985 }
      ],
      costing: "auto",
      action: "isochrone",
      contours: [{ time: 15 }],
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
        { lat: 52.08957210411523, lon: 5.1103487316804985 }
      ],
      costing: "auto",
      contours: [{ time: 15 }],
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
        { lat: 52.08957210411523, lon: 5.1103487316804985 },
        { lat: 52.09085032726166, lon: 5.121582587112782 }
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
        { lat: 52.08957210411523, lon: 5.1103487316804985 },
        { lat: 52.09085032726166, lon: 5.121582587112782 }
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
        { lat: 52.08957210411523, lon: 5.1103487316804985 },
        { lat: 52.09085032726166, lon: 5.121582587112782 }
      ],
      targets: [
        { lat: 52.08957210411523, lon: 5.1103487316804985 },
        { lat: 52.09085032726166, lon: 5.121582587112782 }
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
        { lat: 52.08957210411523, lon: 5.1103487316804985 },
        { lat: 52.09085032726166, lon: 5.121582587112782 }
      ],
      targets: [
        { lat: 52.08957210411523, lon: 5.1103487316804985 },
        { lat: 52.09085032726166, lon: 5.121582587112782 }
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
        { lat: 52.08957210411523, lon: 5.1103487316804985 },
        { lat: 52.090000, lon: 5.115000 },
        { lat: 52.09085032726166, lon: 5.121582587112782 }
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
        { lat: 52.08957210411523, lon: 5.1103487316804985 },
        { lat: 52.090000, lon: 5.115000 },
        { lat: 52.09085032726166, lon: 5.121582587112782 }
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
        { lat: 52.08957210411523, lon: 5.1103487316804985 },
        { lat: 52.090000, lon: 5.115000 },
        { lat: 52.09085032726166, lon: 5.121582587112782 }
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
        { lat: 52.08957210411523, lon: 5.1103487316804985 },
        { lat: 52.090000, lon: 5.115000 },
        { lat: 52.09085032726166, lon: 5.121582587112782 }
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

