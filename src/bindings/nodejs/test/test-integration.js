const { describe, test } = require('node:test');
const assert = require('node:assert');
const valhalla = require('@valhallajs/valhallajs');

describe('Valhalla Integration Tests', () => {
  test('should generate routes using built tiles', async () => {
    const configFile = process.env.VALHALLA_CONFIG_FILE;  
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
});

