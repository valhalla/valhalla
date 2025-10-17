const valhalla = require('./build/src/bindings/nodejs/lib/valhalla_node.node');

// Configuration using Utrecht tiles
const config = JSON.stringify({
  mjolnir: {
    tile_dir: "./build/test/data/utrecht_tiles",
    logging: {
      type: "std_out",
      color: true
    }
  },
  loki: {
    actions: ["route"],
    logging: {
      type: "std_out",
      color: true
    }
  },
  thor: {
    logging: {
      type: "std_out",
      color: true
    }
  },
  odin: {
    logging: {
      type: "std_out",
      color: true
    }
  },
  meili: {
    auto: {
      search_radius: 50
    },
    default: {
      beta: 3,
      breakage_distance: 2000,
      geometry: false,
      gps_accuracy: 5.0,
      interpolation_distance: 10,
      max_route_distance_factor: 3,
      max_route_time_factor: 3,
      max_search_radius: 100,
      route: true,
      search_radius: 50,
      sigma_z: 4.07,
      turn_penalty_factor: 0
    },
    logging: {
      type: "std_out",
      color: true
    }
  },
  service_limits: {
    auto: {
      max_distance: 5000000.0,
      max_locations: 20,
      max_matrix_distance: 400000.0,
      max_matrix_locations: 50
    }
  }
});

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

console.log('Requesting route...');

try {
  const result = actor.route(routeRequest);
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
}
