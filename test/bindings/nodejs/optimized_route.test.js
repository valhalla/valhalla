const test = require('tape');
var config = require('./fixtures/basic_config');
var Valhalla = require('../../../')(JSON.stringify(config));
var valhalla = new Valhalla(JSON.stringify(config));

test('optimizedRoute: can return info for locations', function(assert) {
  var optimizedRouteRequest = '{"locations":[{"lat":40.546115,"lon":-76.385076},{"lat":40.544232,"lon":-76.385752},{"lat":40.543152,"lon":-76.385862},{"lat":40.543952,"lon":-76.386162}],"costing":"auto","directions_options":{"units":"miles"}}';
  valhalla.optimizedRoute(optimizedRouteRequest, (err, result) => {
    var optimizedRoute = JSON.parse(result);
    assert.ok(optimizedRoute);
    assert.equal(optimizedRoute.trip.status_message, 'Found route between points', 'has successful status message');
    assert.equal(optimizedRoute.trip.legs.length, 3, 'returns 3 legs');
    assert.equal(optimizedRoute.trip.legs[0].maneuvers.length, 2, 'returns 2 maneuvers');
    assert.equal(optimizedRoute.trip.legs[1].maneuvers.length, 2, 'returns 2 maneuvers');
    assert.equal(optimizedRoute.trip.legs[2].maneuvers.length, 2, 'returns 2 maneuvers');
    assert.equal(optimizedRoute.trip.locations.length, 4, 'returns back the 4 input locations');
    assert.end();
  });
});

test('optimizedRoute: throws error if can\'t match input to map', function(assert) {
  var badRequest = '{"locations":[{"lat":50,"lon":-76.385076}],"costing":"pedestrian","contours":[{"time":15,"color":"ff0000"}]}';
  valhalla.optimizedRoute(badRequest, (err, result) => {
    assert.notOk(result, 'Should not pass back result');
    assert.equal(err.message, '{"error_code":120,"http_code":400,"message":"Insufficient number of locations provided"}', 'Throws an error when can\'t match input to map with correct codes');
    assert.end();;
  });
});

test('optimizedRoute: returns an error if request format is wrong', function(assert) {
  var badRequest = '{"locations":[40.546115,-76.385076], [40.544232,"lon":-76.385752],"costing":"auto"}';
  valhalla.optimizedRoute(badRequest, (err, result) => {
    assert.notOk(result, 'Should not pass back result');
    assert.equal(err.message, '{"error_code":100,"http_code":400,"message":"Failed to parse json request"}', 'Throws an error when request format is wrong');
    assert.end()
  });
});
