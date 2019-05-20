const test = require('tape');
var config = require('./fixtures/basic_config');
var Valhalla = require('../../../')(JSON.stringify(config));
var valhalla = new Valhalla(JSON.stringify(config));

test('error thrown if valhalla is required with invalid config', function(assert) {
  assert.throws(() => { var ErrValhalla = require('../../../')(); },/Failed to load logging config/, 'Throws an error if required without config');
  assert.throws(() => { var ErrValhalla = require('../../../')('bogus'); },/Failed to load logging config/, 'Throws an error if required with invalid config');
  assert.throws(() => { var valh = new Valhalla(); }, /Unable to parse config/, 'Throws an error if constructed without config');
  assert.throws(() => { var valh = new Valhalla('bogus'); }, /Unable to parse config/, 'Throws an error if constructed with invalid config');
  assert.end();
});

test('route: request sends clear error if called with undefined', function(assert) {
  assert.throws(() => { valhalla.route() }, /method must be called with string and callback/, 'Throws error when called with no arg');
  assert.throws(() => { valhalla.route(null) }, /method must be called with string and callback/, 'Throws error when called with null');
  assert.throws(() => { valhalla.route(undefined) }, /method must be called with string and callback/, 'Throws error when called with undefined');
  assert.throws(() => { valhalla.route('{}') }, /method must be called with string and callback/, 'Throws error when called with no callback');
  assert.throws(() => { valhalla.route('{}', null) }, /method must be called with string and callback/, 'Throws error when called with null callback');
  assert.throws(() => { valhalla.route('{}', undefined) }, /method must be called with string and callback/, 'Throws error when called with undefined callback');
  assert.end();
});

test('route: can get a route in Hershey', function(assert) {
  var hersheyRequest = '{"locations":[{"lat":40.546115,"lon":-76.385076,"type":"break"}, {"lat":40.544232,"lon":-76.385752,"type":"break"}],"costing":"auto"}';
  valhalla.route(hersheyRequest, (err, resp) => {
    if (err) assert.error(err, 'should not error');
    assert.ok(resp);
    const route = JSON.parse(resp);
    assert.equal(route['trip']['status_message'], 'Found route between points', 'route was found');
    assert.equal(route['trip']['legs'].length, 1, 'Route has one leg');
    assert.equal(route['trip']['legs'][0]['maneuvers'].length, 2, 'Leg has two maneuvers');
    assert.end();
  });
});

test('route: returns an error if no edges found', function(assert) {
  var hersheyRequest = '{"locations":[{"lat":5,"lon":-76.385076,"type":"break"}, {"lat":40.544232,"lon":-76.385752,"type":"break"}],"costing":"auto"}';
  valhalla.route(hersheyRequest, (err, resp) => {
    assert.notOk(resp);
    assert.equal(err.message, '{"error_code":171,"http_code":400,"message":"No suitable edges near location"}', 'throws the appropriate error with http and error codes');
    assert.end()
  });
});

test('route: returns an error if request format is wrong', function(assert) {
  var hersheyRequest = '{"locations":[40.546115,-76.385076], [40.544232,"lon":-76.385752],"costing":"auto"}';
  valhalla.route(hersheyRequest, (err, resp) => {
    assert.notOk(resp, 'no response returned');
    assert.equal(err.message, '{"error_code":100,"http_code":400,"message":"Failed to parse json request"}', 'Throws an error');
    assert.end();
  })
});
