const test = require('tape');
var config = require('./fixtures/basic_config');
var Valhalla = require('../../../')(JSON.stringify(config));
var valhalla = new Valhalla(JSON.stringify(config));

test('traceRoute: can return info for locations', function(assert) {
  var traceRouteRequest = '{"shape": [{"lon": -76.38601899147034, "lat": 40.54356410818211},{"lon": -76.38604044914246, "lat": 40.54393914378402},{"lon": -76.38568639755249, "lat": 40.544281565760556},{"lon": -76.38562202453613, "lat": 40.54473812567283},{"lon": -76.38546109199524, "lat": 40.54511315470123},{"lon": -76.38535380363464, "lat": 40.54549633436573},{"lon": -76.38522505760193, "lat": 40.5457816795003},{"lon": -76.38488173484802, "lat": 40.5457816795003},{"lon": -76.38467788696289, "lat": 40.54559416712024},{"lon": -76.38432383537292, "lat": 40.54551263983472},{"lon": -76.38399124145508, "lat": 40.54556970894501},{"lon": -76.38359427452087, "lat": 40.54543926519292}], "costing":"auto","shape_match":"map_snap", "directions_options":{"units":"miles"}}';
  valhalla.traceRoute(traceRouteRequest, (err, result) => {
    var traceRoute = JSON.parse(result);
    assert.ok(traceRoute);
    assert.equal(traceRoute.trip.status_message, 'Found route between points', 'returns successful status message');
    assert.equal(traceRoute.trip.legs.length, 1, '1 leg returned');
    assert.equal(traceRoute.trip.legs[0].maneuvers.length, 3, '3 maneuvers returned');
    assert.equal(traceRoute.trip.legs[0].maneuvers[2].instruction, 'You have arrived at your destination.', 'last maneuver is arrival');
    assert.end();
  });
});

test('traceRoute: throws error if can\'t match input to map', function(assert) {
  var badRequest = '{"shape": [{"lon": 5, "lat": 40.543564},{"lon": 6, "lat": 40.543939},{"lon": 5, "lat": 40.5442815},{"lon": 5, "lat": 40.5447381}],"costing":"auto", "directions_options":{"units":"miles"}}';
  valhalla.traceRoute(badRequest, (err, result) => {
    assert.notOk(result, 'Shouldnt pass back result');
    assert.equal(err.message, '{"error_code":171,"http_code":400,"message":"No suitable edges near location"}', 'Throws an error when cant match input to map with appropriate codes');
    assert.end();
  });
});

test('traceRoute: returns an error if request format is wrong', function(assert) {
  var badRequest = '{"locations":[40.546115,-76.385076], [40.544232,"lon":-76.385752],"costing":"auto"}';
  valhalla.traceRoute(badRequest, (err, result) => {
    assert.notOk(result, 'Shouldnt pass back result');
    assert.equal(err.message, '{"error_code":100,"http_code":400,"message":"Failed to parse json request"}', 'Throws an error when cant match input to map with appropriate codes');
    assert.end();
  });
});
