const test = require('tape');
var config = require('./fixtures/basic_config');
var Valhalla = require('../../')(JSON.stringify(config));
var valhalla = new Valhalla(JSON.stringify(config));

test('route: can get a route in Hershey', function(assert) {
  var hersheyRequest = '{"locations":[{"lat":40.546115,"lon":-76.385076,"type":"break"}, {"lat":40.544232,"lon":-76.385752,"type":"break"}],"costing":"auto"}';
  var route = JSON.parse(valhalla.route(hersheyRequest));
  assert.ok(route);
  assert.equal(route['trip']['status_message'], 'Found route between points', 'route was found');
  assert.equal(route['trip']['legs'].length, 1, 'Route has one leg');
  assert.equal(route['trip']['legs'][0]['maneuvers'].length, 2, 'Leg has two maneuvers');
  assert.end();
});

test('route: returns an error if no edges found', function(assert) {
  var hersheyRequest = '{"locations":[{"lat":5,"lon":-76.385076,"type":"break"}, {"lat":40.544232,"lon":-76.385752,"type":"break"}],"costing":"auto"}';
  assert.throws(() => { valhalla.route(hersheyRequest) }, /No suitable edges near location/, 'Throws correct error');
  assert.end();
});

test('route: returns an error if request format is wrong', function(assert) {
  var hersheyRequest = '{"locations":[40.546115,-76.385076], [40.544232,"lon":-76.385752],"costing":"auto"}';
  // TODO: we are not throwing a very useful error in this case - we should track this
  // down and throw something a little more descriptive
  assert.throws(() => { valhalla.route(hersheyRequest) }, /std::exception/, 'Throws an error');
  assert.end();
});

test.onFinish(() => {
    valhalla = null;
    if (global.gc) global.gc();
});
