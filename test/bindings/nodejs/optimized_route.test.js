const test = require('tape');
var config = require('./fixtures/basic_config');
var Valhalla = require('../../../')(JSON.stringify(config));
var valhalla = new Valhalla(JSON.stringify(config));

test('optimizedRoute: can return info for locations', function(assert) {
  var optimizedRouteRequest = '{"locations":[{"lat":40.546115,"lon":-76.385076},{"lat":40.544232,"lon":-76.385752},{"lat":40.543152,"lon":-76.385862},{"lat":40.543952,"lon":-76.386162}],"costing":"auto","directions_options":{"units":"miles"}}';
  var optimizedRoute = JSON.parse(valhalla.optimizedRoute(optimizedRouteRequest));
  assert.ok(optimizedRoute);
  assert.equal(optimizedRoute.trip.status_message, 'Found route between points', 'has successful status message');
  assert.equal(optimizedRoute.trip.legs.length, 1, 'returns one leg');
  assert.equal(optimizedRoute.trip.legs[0].maneuvers.length, 6, 'returns 6 maneuvers');
  assert.equal(optimizedRoute.trip.locations.length, 4, 'returns back the 4 input locations');
  assert.end();
});

test('optimizedRoute: throws error if can\'t match input to map', function(assert) {
  var badRequest = '{"locations":[{"lat":50,"lon":-76.385076}],"costing":"pedestrian","contours":[{"time":15,"color":"ff0000"}]}';
  assert.throws(() => { valhalla.optimizedRoute(badRequest), /Error: No suitable edges near location/, 'Throws an error when cant match input to map'});
  assert.end();
});

test('optimizedRoute: returns an error if request format is wrong', function(assert) {
  var badRequest = '{"locations":[40.546115,-76.385076], [40.544232,"lon":-76.385752],"costing":"auto"}';
  assert.throws(() => { valhalla.optimizedRoute(badRequest) }, /std::exception/, 'Throws an error when request format is wrong');
  assert.end();
});
