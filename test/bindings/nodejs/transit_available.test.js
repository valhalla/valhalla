const test = require('tape');
var config = require('./fixtures/basic_config');
var Valhalla = require('../../../')(JSON.stringify(config));
var valhalla = new Valhalla(JSON.stringify(config));

test('transitAvailable: can return info for locations', function(assert) {
  var transitAvailableRequest = '{"locations": [{"lon": -76.386018, "lat": 40.543564, "radius": 20},{"lon": -76.386040, "lat": 40.543939, "radius": 20}]}';
  var transitAvailable = JSON.parse(valhalla.transitAvailable(transitAvailableRequest));
  assert.ok(transitAvailable);
  assert.equal(transitAvailable.length, 2, 'returns two availabilities, one for each point');
  assert.equal(transitAvailable[0].istransit, false, 'no transit found');
  assert.end();
});

test('transitAvailable: returns istransit false if bogus values passed in', function(assert) {
  var badRequest = '{"locations": [{"lon": 500, "lat": 40.543564, "radius": 20},{"lon": 500, "lat": 40.543939, "radius": 20}]}';
  var transitAvailable = JSON.parse(valhalla.transitAvailable(badRequest));
  assert.ok(transitAvailable);
  assert.equal(transitAvailable[0].istransit, false, 'no transit found');
  assert.end();
});

test('transitAvailable: returns an error if request format is wrong', function(assert) {
  var badRequest = '{"locations":[40.546115,-76.385076], [40.544232,"lon":-76.385752],"costing":"auto"}';
  assert.throws(() => { valhalla.transitAvailable(badRequest) }, /std::exception/, 'Throws an error when request format is wrong');
  assert.end();
});
