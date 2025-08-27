// DCT-II algorithm is used to encode speed buckets for Valhalla
// https://en.wikipedia.org/wiki/Discrete_cosine_transform
//
// see: https://github.com/valhalla/valhalla/blob/master/src/baldr/predictedspeeds.cc
// (this file is converted from the valhalla version)

// These constants come directly from valhalla/baldr/predictedspeeds.h and predictedspeeds.cc
const kCoefficientCount = 200;
const kBucketsPerWeek = 2016; // 7 days * 24 hours * 12 5-minute buckets per hour
const k1OverSqrt2 = 0.707106781; // 1 / sqrt(2)
const kPiBucketConstant = 3.14159265 / 2016.0; // PI / kBucketsPerWeek
const kSpeedNormalization = 0.031497039; // sqrt(2.0 / 2016.0)
const kDecodedSpeedSize = kCoefficientCount * 2; // 2 bytes per coefficient

// BucketCosTable singleton implementation
class BucketCosTable {
  static instance = null;

  constructor() {
    // allocate table
    this.table = new Float32Array(kCoefficientCount * kBucketsPerWeek);

    // fill out the table in bucket order
    let index = 0;
    for (let bucket = 0; bucket < kBucketsPerWeek; bucket++) {
      for (let c = 0; c < kCoefficientCount; c++) {
        this.table[index++] = Math.cos(kPiBucketConstant * (bucket + 0.5) * c);
      }
    }
  }

  static getInstance() {
    if (!this.instance) {
      this.instance = new BucketCosTable(); // lazy init
    }
    return this.instance;
  }

  get(bucket) {
    // Return a view starting at the specified bucket
    const startIndex = bucket * kCoefficientCount;
    return this.table.subarray(startIndex, startIndex + kCoefficientCount);
  }
}


// compress speed buckets using DCT-II
// this is an exact implementation of the C++ version in predictedspeeds.cc
export const compressSpeedBuckets = (speeds) => {
  const coefficients = new Float32Array(kCoefficientCount);

  // DCT-II with speed normalization
  for (let bucket = 0; bucket < kBucketsPerWeek; bucket++) {
      // get the precomputed cos values for this bucket
    const cosValues = BucketCosTable.getInstance().get(bucket);
    const speed = speeds[bucket];

    for (let c = 0; c < kCoefficientCount; c++) {
      coefficients[c] += cosValues[c] * speed;
    }
  }

  // apply normalization factor to the first coefficient
  coefficients[0] *= k1OverSqrt2;

  // convert to int16 array with rounding
  const result = new Int16Array(kCoefficientCount);
  for (let i = 0; i < kCoefficientCount; i++) {
    result[i] = Math.round(kSpeedNormalization * coefficients[i]);
  }

  return result;
};

// encode compressed speeds to base64
// Implements Valhalla's encode64 function behavior (from util.cc)
export const encodeCompressedSpeeds = (coefficients) => {
  // Create a buffer to hold the big-endian int16 values
  const buffer = new ArrayBuffer(kCoefficientCount * 2);
  const view = new DataView(buffer);

  // Convert each coefficient to big-endian int16 (matching C++ to_big_endian)
  for (let i = 0; i < kCoefficientCount; i++) {
    view.setInt16(i * 2, coefficients[i], false); // false = big-endian
  }

  // Convert binary data to base64
  const bytes = new Uint8Array(buffer);
  const base64String = Buffer.from(bytes).toString('base64');

  // In Valhalla, padding is added in encode64 but might be removed later.
  // We'll do precise verification to determine if Valhalla expects padding or not.
  return base64String.replace(/=+$/, '');
};

/**
 * Verify that an encoded speed string meets Valhalla's expectations
 * @param {string} encoded - The base64 encoded speed string
 * @param {Int16Array} originalCoefficients - Original coefficients for comparing round-trip
 * @returns {object} Diagnostic information about the encoding
 */
export const verifyEncodedSpeeds = (encoded, originalCoefficients = null) => {
  try {
    // Log detailed information about the encoded string
    const diagnostics = {
      length: encoded.length,
      paddingCount: (encoded.match(/=/g) || []).length,
      expectedDecodedSize: kCoefficientCount * 2,
      decodedSize: 0,
      valhallaPaddingStrategy: "Adds padding in encode64() but may be removed elsewhere",
      roundTripAccuracy: null,
      hasPaddingChars: encoded.includes('='),
      modulo4Length: encoded.length % 4,
      isValidLength: encoded.length % 4 === 0 || encoded.length % 4 === 2 || encoded.length % 4 === 3,
    };

    // Try to decode with and without padding
    try {
      // Test padding as in Valhalla
      const paddingNeeded = (4 - encoded.length % 4) % 4;
      const paddedEncoded = encoded + '='.repeat(paddingNeeded);
      const buffer = Buffer.from(paddedEncoded, 'base64');
      diagnostics.decodedSize = buffer.length;
      diagnostics.withPaddingDecodedSize = buffer.length;

      // If this is the expected size, things are likely correct
      diagnostics.sizeMatchesExpected = buffer.length === kCoefficientCount * 2;

      // Decode coefficients to check fidelity
      if (originalCoefficients) {
        const decoded = decodeCompressedSpeeds(encoded);
        let maxDiff = 0;
        let diffCount = 0;

        for (let i = 0; i < kCoefficientCount; i++) {
          const diff = Math.abs(decoded[i] - originalCoefficients[i]);
          if (diff > 0) {
            diffCount++;
            maxDiff = Math.max(maxDiff, diff);
          }
        }

        diagnostics.roundTripAccuracy = {
          diffCount,
          maxDiff,
          isLossless: diffCount === 0,
          isAcceptable: maxDiff <= 1
        };
      }
    } catch (e) {
      diagnostics.decodingWithPaddingError = e.message;
    }

    // Try without padding to compare
    try {
      const noPaddingEncoded = encoded.replace(/=+$/, '');
      const buffer = Buffer.from(noPaddingEncoded, 'base64');
      diagnostics.withoutPaddingDecodedSize = buffer.length;
    } catch (e) {
      diagnostics.decodingWithoutPaddingError = e.message;
    }

    // Output diagnostics to console
    console.log("=== Speed Encoding Diagnostics ===");
    console.log(JSON.stringify(diagnostics, null, 2));

    // Based on diagnostics, provide recommendations
    if (!diagnostics.sizeMatchesExpected) {
      console.error(`ERROR: Decoded size ${diagnostics.decodedSize} doesn't match expected ${diagnostics.expectedDecodedSize}`);

      // Suggest fixes
      if (diagnostics.decodedSize > diagnostics.expectedDecodedSize) {
        console.error("  Possible fix: Check coefficient count and buffer creation");
      } else {
        console.error("  Possible fix: Ensure all coefficients are properly encoded");
      }
    }

    // Check if we're matching Valhalla's expectations for the specific error in the logs
    if (diagnostics.decodedSize === 408 || diagnostics.decodedSize === 409 || diagnostics.decodedSize === 410) {
      console.error(`WARNING: Decoded size of ${diagnostics.decodedSize} matches the error seen in Valhalla logs.`);
      console.error("  This suggests a size mismatch between coefficient count settings.");
      console.error(`  Valhalla expects exactly 400 bytes (${kCoefficientCount} coefficients * 2 bytes each).`);
    }

    return diagnostics;
  } catch (error) {
    console.error('ERROR validating encoded speeds:', error.message);
    return { error: error.message };
  }
};

// decode compressed speeds from base64
// Implements Valhalla's decode64 function behavior (from util.cc)
export const decodeCompressedSpeeds = (encoded) => {
  // Valhalla's decode64 calculates padding and adds it if needed
  let paddingNeeded = (4 - (encoded.length % 4)) % 4;
  let paddedEncoded = encoded;
  if (paddingNeeded > 0) {
    paddedEncoded = encoded + '='.repeat(paddingNeeded);
  }

  // decode from base64
  const buffer = Buffer.from(paddedEncoded, 'base64');

  if (buffer.length !== kDecodedSpeedSize) {
    throw new Error(`Decoded speed string size expected=${kDecodedSpeedSize} actual=${buffer.length}`);
  }

  // create the coefficients array
  const coefficients = new Int16Array(kCoefficientCount);
  const view = new DataView(buffer.buffer, buffer.byteOffset, buffer.byteLength);

  // convert from big-endian to little-endian
  for (let i = 0; i < kCoefficientCount; i++) {
    coefficients[i] = view.getInt16(i * 2, false); // false = big-endian
  }
  return coefficients;
};

// decompress a single speed bucket using DCT-III
export const decompressSpeedBucket = (coefficients, bucket_idx) => {
  // get a pointer to the precomputed cos values for this bucket
  const b = BucketCosTable.getInstance().get(bucket_idx);

  // DCT-III with speed normalization
  // In C++ this is: float speed = *coefficients * k1OverSqrt2;
  let speed = coefficients[0] * k1OverSqrt2;

  // this starts with both pointers at position 1 and increments both together
  for (let i = 1; i < kCoefficientCount; i++) {
    speed += coefficients[i] * b[i];
  }

  return speed * kSpeedNormalization;
};
