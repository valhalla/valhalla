const { describe, test } = require('node:test');
const assert = require('node:assert');
const { GraphId, getTileBaseLonLat, getTileIdFromLonLat, getTileIdsFromBbox, getTileIdsFromRing } = require('@valhallajs/valhallajs');

describe('GraphId', () => {
  test('should export GraphId class', () => {
    assert.strictEqual(typeof GraphId, 'function');
  });

  describe('constructors', () => {
    test('default constructor creates invalid GraphId', () => {
      const gid = new GraphId();
      assert.strictEqual(gid.is_valid(), false);
    });

    test('construct from tileid, level, id', () => {
      const gid = new GraphId(100, 2, 5);
      assert.strictEqual(gid.tileid(), 100);
      assert.strictEqual(gid.level(), 2);
      assert.strictEqual(gid.id(), 5);
      assert.strictEqual(gid.is_valid(), true);
    });

    test('construct from string', () => {
      const gid = new GraphId('2/100/5');
      assert.strictEqual(gid.tileid(), 100);
      assert.strictEqual(gid.level(), 2);
      assert.strictEqual(gid.id(), 5);
    });

    test('construct from numeric value', () => {
      const original = new GraphId(100, 2, 5);
      const fromValue = new GraphId(original.value);
      assert.strictEqual(fromValue.tileid(), 100);
      assert.strictEqual(fromValue.level(), 2);
      assert.strictEqual(fromValue.id(), 5);
    });

    test('throws on invalid string format', () => {
      assert.throws(() => new GraphId('invalid'), Error);
    });

    test('throws on out-of-range level', () => {
      assert.throws(() => new GraphId(0, 99, 0), Error);
    });

    test('throws on wrong number of arguments', () => {
      assert.throws(() => new GraphId(1, 2), TypeError);
    });
  });

  describe('value property', () => {
    test('value is a number', () => {
      const gid = new GraphId(100, 2, 5);
      assert.strictEqual(typeof gid.value, 'number');
    });

    test('value is read-only', () => {
      const gid = new GraphId(100, 2, 5);
      const original = gid.value;
      gid.value = 999;
      assert.strictEqual(gid.value, original);
    });

    test('default GraphId value matches kInvalidGraphId', () => {
      const gid = new GraphId();
      assert.strictEqual(gid.value, 0x3fffffffffff);
    });
  });

  describe('tile_base', () => {
    test('returns GraphId with id zeroed out', () => {
      const gid = new GraphId(100, 2, 5);
      const base = gid.tile_base();
      assert.strictEqual(base.tileid(), 100);
      assert.strictEqual(base.level(), 2);
      assert.strictEqual(base.id(), 0);
    });
  });

  describe('tile_value', () => {
    test('returns 32-bit tile value', () => {
      const gid = new GraphId(100, 2, 5);
      const base = gid.tile_base();
      assert.strictEqual(gid.tile_value(), base.value);
    });
  });

  describe('add', () => {
    test('advances the id by offset', () => {
      const gid = new GraphId(100, 2, 5);
      const advanced = gid.add(3);
      assert.strictEqual(advanced.tileid(), 100);
      assert.strictEqual(advanced.level(), 2);
      assert.strictEqual(advanced.id(), 8);
    });

    test('does not mutate the original', () => {
      const gid = new GraphId(100, 2, 5);
      gid.add(3);
      assert.strictEqual(gid.id(), 5);
    });
  });

  describe('equals', () => {
    test('returns true for same values', () => {
      const a = new GraphId(100, 2, 5);
      const b = new GraphId(100, 2, 5);
      assert.strictEqual(a.equals(b), true);
    });

    test('returns false for different values', () => {
      const a = new GraphId(100, 2, 5);
      const b = new GraphId(100, 2, 6);
      assert.strictEqual(a.equals(b), false);
    });

    test('returns false for non-GraphId argument', () => {
      const gid = new GraphId(100, 2, 5);
      assert.strictEqual(gid.equals({}), false);
      assert.strictEqual(gid.equals(null), false);
      assert.strictEqual(gid.equals(42), false);
    });
  });

  describe('toString', () => {
    test('returns level/tileid/id format', () => {
      const gid = new GraphId(100, 2, 5);
      assert.strictEqual(gid.toString(), '2/100/5');
    });
  });

  describe('toJSON', () => {
    test('returns plain object with components', () => {
      const gid = new GraphId(100, 2, 5);
      const json = gid.toJSON();
      assert.strictEqual(json.level, 2);
      assert.strictEqual(json.tileid, 100);
      assert.strictEqual(json.id, 5);
      assert.strictEqual(json.value, gid.value);
    });

    test('works with JSON.stringify', () => {
      const gid = new GraphId(100, 2, 5);
      const str = JSON.stringify(gid);
      const parsed = JSON.parse(str);
      assert.strictEqual(parsed.level, 2);
      assert.strictEqual(parsed.tileid, 100);
      assert.strictEqual(parsed.id, 5);
    });
  });

  describe('roundtrip', () => {
    test('string roundtrip preserves values', () => {
      const original = new GraphId(500, 1, 42);
      const str = original.toString();
      const restored = new GraphId(str);
      assert.strictEqual(restored.equals(original), true);
    });

    test('value roundtrip preserves values', () => {
      const original = new GraphId(500, 1, 42);
      const restored = new GraphId(original.value);
      assert.strictEqual(restored.equals(original), true);
    });
  });
});

describe('getTileBaseLonLat', () => {
  test('should export getTileBaseLonLat function', () => {
    assert.strictEqual(typeof getTileBaseLonLat, 'function');
  });

  test('returns [lon, lat] array for a valid GraphId', () => {
    const gid = getTileIdFromLonLat(2, [13.4, 52.5]);
    const result = getTileBaseLonLat(gid);
    assert.ok(Array.isArray(result));
    assert.strictEqual(result.length, 2);
    assert.strictEqual(typeof result[0], 'number');
    assert.strictEqual(typeof result[1], 'number');
  });

  test('base lon/lat is within reasonable range', () => {
    const gid = getTileIdFromLonLat(2, [13.4, 52.5]);
    const [lon, lat] = getTileBaseLonLat(gid);
    // base should be at or below the queried point
    assert.ok(lon <= 13.4);
    assert.ok(lat <= 52.5);
    // and reasonably close (within one tile width)
    assert.ok(lon > 12.0);
    assert.ok(lat > 51.0);
  });

  test('throws on non-GraphId argument', () => {
    assert.throws(() => getTileBaseLonLat('not a graphid'), TypeError);
  });
});

describe('getTileIdFromLonLat', () => {
  test('should export getTileIdFromLonLat function', () => {
    assert.strictEqual(typeof getTileIdFromLonLat, 'function');
  });

  test('returns a valid GraphId for a coordinate', () => {
    const gid = getTileIdFromLonLat(2, [13.4, 52.5]);
    assert.strictEqual(gid.is_valid(), true);
    assert.strictEqual(gid.level(), 2);
    assert.strictEqual(gid.id(), 0);
  });

  test('returns different tiles for different levels', () => {
    const gid0 = getTileIdFromLonLat(0, [13.4, 52.5]);
    const gid2 = getTileIdFromLonLat(2, [13.4, 52.5]);
    assert.strictEqual(gid0.level(), 0);
    assert.strictEqual(gid2.level(), 2);
  });

  test('throws on invalid level', () => {
    assert.throws(() => getTileIdFromLonLat(99, [13.4, 52.5]), Error);
  });

  test('throws on invalid coordinates', () => {
    assert.throws(() => getTileIdFromLonLat(2, [200, 0]), Error);
  });

  test('throws on wrong argument types', () => {
    assert.throws(() => getTileIdFromLonLat(2, 'not an array'), TypeError);
  });
});

describe('getTileIdsFromBbox', () => {
  test('should export getTileIdsFromBbox function', () => {
    assert.strictEqual(typeof getTileIdsFromBbox, 'function');
  });

  test('returns GraphId array for a bbox', () => {
    const result = getTileIdsFromBbox(13.3, 52.4, 13.5, 52.6);
    assert.ok(Array.isArray(result));
    assert.ok(result.length > 0);
    for (const gid of result) {
      assert.strictEqual(gid.is_valid(), true);
      assert.strictEqual(gid.id(), 0);
    }
  });

  test('respects the levels parameter', () => {
    const result = getTileIdsFromBbox(13.3, 52.4, 13.5, 52.6, [2]);
    assert.ok(result.length > 0);
    for (const gid of result) {
      assert.strictEqual(gid.level(), 2);
    }
  });

  test('throws on invalid coordinates', () => {
    assert.throws(() => getTileIdsFromBbox(200, 0, 201, 1), Error);
  });

  test('throws on missing arguments', () => {
    assert.throws(() => getTileIdsFromBbox(1, 2), TypeError);
  });
});

describe('getTileIdsFromRing', () => {
  test('should export getTileIdsFromRing function', () => {
    assert.strictEqual(typeof getTileIdsFromRing, 'function');
  });

  test('returns GraphId array for a simple polygon', () => {
    // small polygon around Berlin
    const ring = [
      [13.3, 52.4],
      [13.5, 52.4],
      [13.5, 52.6],
      [13.3, 52.6],
    ];
    const result = getTileIdsFromRing(ring);
    assert.ok(Array.isArray(result));
    assert.ok(result.length > 0);
    // all results should be valid GraphId instances
    for (const gid of result) {
      assert.strictEqual(gid.is_valid(), true);
      assert.strictEqual(gid.id(), 0); // tile-base ids have id=0
    }
  });

  test('respects the levels parameter', () => {
    const ring = [
      [13.3, 52.4],
      [13.5, 52.4],
      [13.5, 52.6],
      [13.3, 52.6],
    ];
    const result = getTileIdsFromRing(ring, [2]);
    assert.ok(result.length > 0);
    for (const gid of result) {
      assert.strictEqual(gid.level(), 2);
    }
  });

  test('throws on too few coordinates', () => {
    assert.throws(() => getTileIdsFromRing([[0, 0], [1, 1]]), Error);
  });

  test('throws on invalid coordinates', () => {
    assert.throws(() => getTileIdsFromRing([[200, 0], [0, 0], [0, 1]]), Error);
  });

  test('throws on non-array input', () => {
    assert.throws(() => getTileIdsFromRing('not an array'), TypeError);
  });
});
