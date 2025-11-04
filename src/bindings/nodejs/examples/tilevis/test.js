#!/usr/bin/env node

const { test } = require('node:test');
const assert = require('node:assert');
const http = require('node:http');

const PORT = process.env.TEST_PORT || 8080;
const HOST = process.env.TEST_HOST || '127.0.0.1';

// Utrecht center tile coordinates (52.08778°N, 5.13142°E at zoom 14)
const TILE_Z = 14;
const TILE_X = 8425;
const TILE_Y = 5405;

function httpRequest(path) {
  return new Promise((resolve, reject) => {
    const options = {
      hostname: HOST,
      port: PORT,
      path: path,
      method: 'GET'
    };

    const req = http.request(options, (res) => {
      const chunks = [];
      res.on('data', (chunk) => chunks.push(chunk));
      res.on('end', () => {
        resolve({
          statusCode: res.statusCode,
          headers: res.headers,
          body: Buffer.concat(chunks)
        });
      });
    });

    req.on('error', reject);
    req.setTimeout(5000, () => {
      req.destroy();
      reject(new Error('Request timeout'));
    });

    req.end();
  });
}

test('HTML endpoint returns map page', async () => {
  const response = await httpRequest('/');
  
  assert.strictEqual(response.statusCode, 200, 'Should return HTTP 200');
  assert.match(
    response.headers['content-type'],
    /text\/html/,
    'Content-Type should be text/html'
  );
  
  const html = response.body.toString('utf8');
  assert.ok(html.includes('map'), 'HTML should contain "map"');
  assert.ok(html.includes('maplibre'), 'HTML should contain "maplibre"');
});

test('Vector tile endpoint returns valid MVT for Utrecht', async () => {
  const tilePath = `/${TILE_Z}/${TILE_X}/${TILE_Y}.mvt`;
  console.log(`  Requesting tile: ${tilePath}`);
  
  const response = await httpRequest(tilePath);
  
  assert.strictEqual(response.statusCode, 200, 'Should return HTTP 200');
  assert.match(
    response.headers['content-type'],
    /application\/vnd\.mapbox-vector-tile/,
    'Content-Type should be application/vnd.mapbox-vector-tile'
  );
  assert.ok(
    response.headers['access-control-allow-origin'],
    'Should have CORS headers'
  );
  const tileSize = response.body.length;
  assert.ok(
    tileSize >= 100,
    `Tile size should be at least 100 bytes, got ${tileSize}`
  );
  const isText = response.body.toString('utf8').match(/^[a-zA-Z0-9\s.,;:!?'"]+$/);
  assert.ok(!isText, 'Tile should be binary data, not plain text');
});

test('Invalid tile coordinates return 400', async () => {
  const response = await httpRequest('/99/999999/999999.mvt');
  
  assert.strictEqual(
    response.statusCode,
    400,
    'Should return HTTP 400 for invalid coordinates'
  );
});


