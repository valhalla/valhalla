const test = require('tape');
var config = require('./fixtures/basic_config');
var Valhalla = require('../../../')(JSON.stringify(config));
var valhalla = new Valhalla(JSON.stringify(config));

test('matrix: can return matrix info info for locations', function(assert) {
  var matrixRequest = '{"sources":[{"lat":40.546115,"lon":-76.385076}],"targets":[{"lat":40.544232,"lon":-76.385752},{"lat":40.54424,"lon":-76.38574}],"costing":"pedestrian"}';
  var matrix = JSON.parse(valhalla.matrix(matrixRequest));
  assert.ok(matrix);
  assert.equal(matrix.sources_to_targets[0].length, 2, 'two sources to targets returned');
  assert.ok(matrix.sources_to_targets[0][0].distance, 'returns distance info');
  assert.ok(matrix.sources_to_targets[0][0].time, 'returns time info');
  assert.equal(matrix.sources_to_targets[0][0].to_index, 0, 'returns to_index');
  assert.equal(matrix.sources_to_targets[0][0].from_index, 0, 'returns from_index');
  assert.end();
});

test('matrix: returns error if no edges found', function(assert) {
  var hersheyRequest = '{"sources":[{"lat":-40.546115,"lon":-76.385076}],"targets":[{"lat":-40.544232,"lon":-76.385752},{"lat":-40.54424,"lon":-76.38574}],"costing":"pedestrian"}';
  assert.throws(() => { valhalla.matrix(hersheyRequest)}, /No suitable edges near location/, 'No edges found');
  assert.end();
});

test('matrix: returns an error if request format is wrong', function(assert) {
  var badRequest = '{"locations":[40.546115,-76.385076], [40.544232,"lon":-76.385752],"costing":"auto"}';
  assert.throws(() => { valhalla.matrix(badRequest) }, /std::exception/, 'Throws an error');
  assert.end();
});
