const test = require('tape');
var config = require('./fixtures/basic_config');
var Valhalla = require('../../../')(JSON.stringify(config));
var valhalla = new Valhalla(JSON.stringify(config));

test('locate: can locate something?', function(assert) {
  var locateRequest = '{"verbose":true,"locations":[{"lat":42.358528,"lon":-83.271400},{"lat":42.996613,"lon":-78.749855}],"costing":"bicycle","costing_options":{"bicycle":{"bicycle_type":"road"}},"directions_options":{"units":"miles"},"id":"12abc3afe23984fe"}';
  var locate = JSON.parse(valhalla.locate(locateRequest));
  assert.ok(locate);
  assert.equal();
  assert.end();
});
