const test = require('tape');
var config = require('./fixtures/basic_config');
var Valhalla = require('../../../')(JSON.stringify(config));
var valhalla = new Valhalla(JSON.stringify(config));

test('height: can return info for locations', function(assert) {
  var heightRequest = '{"shape":[{"lat":40.546115,"lon":-76.385076},{"lat":40.544232,"lon":-76.385752}]}';
  var height = JSON.parse(valhalla.height(heightRequest));
  assert.ok(height);
  assert.equal(height.shape.length, 2, 'two locations returned');
  assert.equal(height.height.length, 2, 'two heights returned');
  assert.end();
});

test('height: throws error if can\'t match input to map', function(assert) {
  var badRequest = '{shape":[{"lat":5,"lon":-76.504916},{"lat":5,"lon":-76.605259}]}';
  assert.throws(() => { valhalla.height(badRequest), /std::exception/, 'Throws an error when cant match input to map'});
  assert.end();
});

test('height: returns an error if request format is wrong', function(assert) {
  var badRequest = '{"locations":[40.546115,-76.385076], [40.544232,"lon":-76.385752],"costing":"auto"}';
  assert.throws(() => { valhalla.height(badRequest) }, /std::exception/, 'Throws an error when request format is wrong');
  assert.end();
});
