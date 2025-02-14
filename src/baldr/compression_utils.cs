using System;
using System.IO;
using System.IO.Compression;
using System.Threading.Tasks;

namespace valhalla.baldr
{
    public static class CompressionUtils
    {
        public static async Task<bool> DeflateAsync(Func<Stream, Task<int>> srcFunc, Func<Stream, Task> dstFunc, int level, bool gzip)
        {
            try
            {
                using (var memoryStream = new MemoryStream())
                {
                    using (var compressionStream = gzip ? new GZipStream(memoryStream, (CompressionLevel)level, true) : new DeflateStream(memoryStream, (CompressionLevel)level, true))
                    {
                        int bytesRead;
                        while ((bytesRead = await srcFunc(compressionStream)) > 0)
                        {
                            await dstFunc(compressionStream);
                        }
                    }

                    memoryStream.Seek(0, SeekOrigin.Begin);
                    await dstFunc(memoryStream);
                }

                return true;
            }
            catch
            {
                return false;
            }
        }

        public static async Task<bool> InflateAsync(Func<Stream, Task> srcFunc, Func<Stream, Task<int>> dstFunc)
        {
            try
            {
                using (var memoryStream = new MemoryStream())
                {
                    await srcFunc(memoryStream);
                    memoryStream.Seek(0, SeekOrigin.Begin);

                    using (var decompressionStream = new GZipStream(memoryStream, CompressionMode.Decompress))
                    {
                        int bytesRead;
                        while ((bytesRead = await dstFunc(decompressionStream)) > 0)
                        {
                            await dstFunc(decompressionStream);
                        }
                    }
                }

                return true;
            }
            catch
            {
                return false;
            }
        }
    }
}
