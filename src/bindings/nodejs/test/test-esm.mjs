import { describe, test } from 'node:test';
import assert from 'node:assert';
import valhalla from '@valhallajs/valhallajs';

describe('Valhalla ESM Tests', () => {
  test('should load Valhalla package successfully', () => {
    assert.ok(valhalla, 'Valhalla package should be loaded');
    const exports = Object.keys(valhalla);
    assert.ok(exports.length > 0, 'Should have at least one export');
  });

  test('should export VALHALLA_VERSION', () => {
    assert.strictEqual(typeof valhalla.VALHALLA_VERSION, 'string');
    assert.ok(valhalla.VALHALLA_VERSION.length > 0, 'Version should not be empty');
  });

  test('should export Actor class', () => {
    assert.strictEqual(typeof valhalla.Actor, 'function', 'Actor should be a function/class');
  });

  test('should export Actor.fromConfigFile static method', () => {
    assert.strictEqual(
      typeof valhalla.Actor.fromConfigFile,
      'function',
      'Actor.fromConfigFile should be a function'
    );
  });
});
