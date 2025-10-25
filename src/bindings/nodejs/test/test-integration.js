const { describe, test } = require('node:test');
const assert = require('node:assert');
const valhalla = require('@valhallajs/valhallajs');

describe('Valhalla Integration Tests', () => {
  test('should generate routes using built tiles', async () => {
    console.log('[INFO] Testing route generation with built tiles...');
    console.log('[INFO] Valhalla version:', valhalla.VALHALLA_VERSION);

    // Load config from file
    const configFile = process.argv[process.argv.length - 1];
    console.log('[INFO] Loading config from:', configFile);
    
    const actor = await valhalla.Actor.fromConfigFile(configFile);
    assert.ok(actor, 'Actor should be initialized');
    console.log('[SUCCESS] Actor initialized successfully');

    // Utrecht, Netherlands coordinates (within the test data area)
    // Route from Utrecht Central Station to Dom Tower
    const routeRequest = {
      locations: [
        { lat: 52.0907, lon: 5.1107 }, // Utrecht Central Station
        { lat: 52.0908, lon: 5.1213 }  // Dom Tower
      ],
      costing: "auto",
      directions_options: {
        units: "kilometers"
      }
    };

    console.log('[INFO] Requesting route...');
    console.log('[INFO] From:', routeRequest.locations[0]);
    console.log('[INFO] To:', routeRequest.locations[1]);

    const result = actor.route(routeRequest);
    
    // Verify result
    assert.ok(result, 'Route result should be returned');
    assert.ok(result.trip, 'Route result should contain trip');
    assert.ok(result.trip.legs, 'Trip should contain legs');
    assert.ok(result.trip.legs.length > 0, 'Trip should have at least one leg');

    const trip = result.trip;
    const leg = trip.legs[0];
    
    console.log('[SUCCESS] Route generated successfully!');
    console.log('[INFO] Route summary:');
    console.log('  - Distance:', leg.summary.length.toFixed(2), 'km');
    console.log('  - Time:', (leg.summary.time / 60).toFixed(2), 'minutes');
    console.log('  - Maneuvers:', leg.maneuvers ? leg.maneuvers.length : 'N/A');
    
    assert.ok(leg.summary.length > 0, 'Route distance should be greater than 0');
    console.log('[SUCCESS] All route tests passed!');
  });
});

