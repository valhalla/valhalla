/**
 * Test script to verify route generation using tiles built by CLI
 * This script loads a Valhalla config and tests that routing works.
 */

async function testRoute() {
  try {
    const valhalla = require('@valhallajs/valhallajs');
    console.log('[INFO] Testing route generation with built tiles...');
    console.log('[INFO] Valhalla version:', valhalla.VALHALLA_VERSION);

    // Load config from file
    const configFile = process.argv[2] || 'valhalla.json';
    console.log('[INFO] Loading config from:', configFile);
    
    const actor = await valhalla.Actor.fromConfigFile(configFile);
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
    if (!result) {
      console.error('[ERROR] No route result returned');
      process.exit(1);
    }

    if (!result.trip) {
      console.error('[ERROR] No trip in route result:', JSON.stringify(result));
      process.exit(1);
    }

    if (!result.trip.legs || result.trip.legs.length === 0) {
      console.error('[ERROR] No legs in route result');
      process.exit(1);
    }

    const trip = result.trip;
    const leg = trip.legs[0];
    
    console.log('[SUCCESS] Route generated successfully!');
    console.log('[INFO] Route summary:');
    console.log('  - Distance:', leg.summary.length.toFixed(2), 'km');
    console.log('  - Time:', (leg.summary.time / 60).toFixed(2), 'minutes');
    console.log('  - Maneuvers:', leg.maneuvers ? leg.maneuvers.length : 'N/A');
    
    if (leg.summary.length <= 0) {
      console.error('[ERROR] Invalid route distance');
      process.exit(1);
    }

    console.log('[SUCCESS] All route tests passed!');
    process.exit(0);

  } catch (error) {
    console.error('[ERROR] Route test failed:', error.message);
    console.error(error.stack);
    process.exit(1);
  }
}

testRoute();

