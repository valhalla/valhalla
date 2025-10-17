const test = require('node:test');
const assert = require('node:assert').strict;

const valhalla = require(process.env.VALHALLA_NODE_LIBRARY_PATH);

test('test', () => {
  assert.ok(valhalla.VALHALLA_VERSION, 'VALHALLA_VERSION is not defined');
});