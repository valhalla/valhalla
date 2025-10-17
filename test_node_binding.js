const valhalla = require('./build/src/bindings/nodejs/lib/valhalla_node.node');
const fs = require('fs');

// Load configuration from file
let config = JSON.parse(fs.readFileSync('./valhalla2.json', 'utf8'));
config.mjolnir.tile_dir = "./build/test/data/utrecht_tiles";
config = JSON.stringify(config);

async function main() {
  console.log('Valhalla version:', valhalla.VALHALLA_VERSION);
  console.log('Creating actor...');

  // Create actor
  const actor = new valhalla.Actor(config);

  console.log('Actor created successfully!');

  // Simple route request in Utrecht
  const routeRequest = JSON.stringify({
    locations: [
      { lat: 52.09110, lon: 5.09806 },
      { lat: 52.09050, lon: 5.12135 }
    ],
    costing: "auto",
    directions_options: {
      units: "kilometers"
    }
  });

  console.log('Requesting route (async)...');

  try {
    // All methods now return Promises - use await
    const result = await actor.route(routeRequest);
    const routeData = JSON.parse(result);

    console.log('\nRoute calculated successfully!');
    console.log('Trip summary:');
    console.log('  Distance:', routeData.trip.summary.length, 'km');
    console.log('  Time:', routeData.trip.summary.time, 'seconds');
    console.log('  Number of legs:', routeData.trip.legs.length);

    if (routeData.trip.legs.length > 0) {
      console.log('\nFirst leg maneuvers:', routeData.trip.legs[0].maneuvers.length);
    }
  } catch (error) {
    console.error('Error:', error.message);
    process.exit(1);
  }
}

// Run the async main function
main().catch((error) => {
  console.error('Unhandled error:', error);
  process.exit(1);
});
