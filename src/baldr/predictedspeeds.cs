using System;
using System.Collections.Generic;
using System.Linq;

namespace valhalla.baldr
{
    public static class PredictedSpeeds
    {
        private const float k1OverSqrt2 = 0.707106781f; // 1 / sqrt(2)
        private const float kPiBucketConstant = 3.14159265f / 2016.0f;
        private const float kSpeedNormalization = 0.031497039f; // sqrt(2.0f / 2016.0f)
        private const uint kCoefficientCount = 64;
        private const uint kBucketsPerWeek = 2016;
        private const uint kDecodedSpeedSize = kCoefficientCount * sizeof(short);

        private static readonly float[] BucketCosTable = CreateBucketCosTable();

        private static float[] CreateBucketCosTable()
        {
            var table = new float[kCoefficientCount * kBucketsPerWeek];
            for (uint bucket = 0; bucket < kBucketsPerWeek; ++bucket)
            {
                for (uint c = 0; c < kCoefficientCount; ++c)
                {
                    table[bucket * kCoefficientCount + c] = (float)Math.Cos(kPiBucketConstant * (bucket + 0.5f) * c);
                }
            }
            return table;
        }

        public static short[] CompressSpeedBuckets(float[] speeds)
        {
            var coefficients = new float[kCoefficientCount];
            for (uint bucket = 0; bucket < kBucketsPerWeek; ++bucket)
            {
                var cosValues = GetCosValues(bucket);
                for (uint c = 0; c < kCoefficientCount; ++c)
                {
                    coefficients[c] += cosValues[c] * speeds[bucket];
                }
            }
            coefficients[0] *= k1OverSqrt2;

            var result = new short[kCoefficientCount];
            for (int i = 0; i < coefficients.Length; ++i)
            {
                result[i] = (short)Math.Round(kSpeedNormalization * coefficients[i]);
            }
            return result;
        }

        public static float DecompressSpeedBucket(short[] coefficients, uint bucketIdx)
        {
            var cosValues = GetCosValues(bucketIdx);
            float speed = coefficients[0] * k1OverSqrt2;
            for (int i = 1; i < kCoefficientCount; ++i)
            {
                speed += coefficients[i] * cosValues[i];
            }
            return speed * kSpeedNormalization;
        }

        public static string EncodeCompressedSpeeds(short[] coefficients)
        {
            var bytes = new byte[kCoefficientCount * sizeof(short)];
            for (int i = 0; i < kCoefficientCount; ++i)
            {
                var value = BitConverter.GetBytes((ushort)coefficients[i]);
                Array.Reverse(value); // Convert to big endian
                Array.Copy(value, 0, bytes, i * sizeof(short), sizeof(short));
            }
            return Convert.ToBase64String(bytes);
        }

        public static short[] DecodeCompressedSpeeds(string encoded)
        {
            var decodedBytes = Convert.FromBase64String(encoded);
            if (decodedBytes.Length != kDecodedSpeedSize)
            {
                throw new ArgumentException($"Decoded speed string size expected= {kDecodedSpeedSize} actual={decodedBytes.Length}");
            }

            var coefficients = new short[kCoefficientCount];
            for (int i = 0; i < kCoefficientCount; ++i)
            {
                var value = BitConverter.ToUInt16(decodedBytes, i * sizeof(short));
                coefficients[i] = (short)IPAddress.NetworkToHostOrder((short)value); // Convert to little endian
            }
            return coefficients;
        }

        private static float[] GetCosValues(uint bucket)
        {
            var cosValues = new float[kCoefficientCount];
            Array.Copy(BucketCosTable, bucket * kCoefficientCount, cosValues, 0, kCoefficientCount);
            return cosValues;
        }
    }
}
