const test = require('tape');
var config = require('./fixtures/basic_config');
var Valhalla = require('../../../')(JSON.stringify(config));
var valhalla = new Valhalla(JSON.stringify(config));

test('locate: can return info for locations', function(assert) {
  var locateRequest = '{"verbose":true,"locations":[{"lat":40.546115,"lon":-76.385076,"type":"break"}, {"lat":40.544232,"lon":-76.385752,"type":"break"}],"costing":"bicycle","costing_directions_options":{"bicycle":{"bicycle_type":"road"}},"directions_options":{"units":"miles"},"id":"12abc3afe23984fe"}';
  valhalla.locate(locateRequest, (err, result) => {
    var locate = JSON.parse(result);
    assert.error(err, "should not error");
    assert.ok(locate);
    assert.equal(locate.length, 2, 'two locations returned');
    assert.ok(locate[0].edges, 'returns edge info');
    assert.end();
  });
});

test('locate: returns null for edges and nodes if none found', function(assert) {
  var hersheyRequest = '{"locations":[{"lat":5,"lon":-76.385076,"type":"break"}, {"lat":4,"lon":-76.385752,"type":"break"}],"costing":"auto"}';
  valhalla.locate(hersheyRequest, (err, result) => {
    var locate = JSON.parse(result);
    assert.equal(locate[0].edges, null, 'no edges found for first location');
    assert.equal(locate[1].edges, null, 'no edges found for second location');
    assert.equal(locate[0].nodes, null, 'no nodes found for first location');
    assert.equal(locate[1].nodes, null, 'no nodes found for second location');
    assert.end();
  });
});

test('locate: returns an error if request format is wrong', function(assert) {
  var badRequest = '{"locations":[40.546115,-76.385076], [40.544232,"lon":-76.385752],"costing":"auto"}';
  valhalla.locate(badRequest, (err, result) => {
    assert.end();
    assert.notOk(result, 'no result returned');
    assert.equal(err.message, '{"error_code":100,"http_code":400,"message":"Failed to parse json request"}', 'Throws an error');
  });
});
