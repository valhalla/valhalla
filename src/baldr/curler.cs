using System;
using System.Collections.Generic;
using System.Net.Http;
using System.Threading.Tasks;

namespace valhalla.baldr
{
    public class curler_t
    {
        private readonly HttpClient httpClient;
        private readonly string userAgent;

        public curler_t(string userAgent)
        {
            this.userAgent = userAgent;
            httpClient = new HttpClient();
            httpClient.DefaultRequestHeaders.Add("User-Agent", userAgent);
        }

        public async Task<List<byte>> fetch(string url, bool gzipped, Func<Task> interrupt = null)
        {
            if (interrupt != null)
            {
                await interrupt();
            }

            if (gzipped)
            {
                httpClient.DefaultRequestHeaders.Add("Accept-Encoding", "gzip");
            }

            var response = await httpClient.GetAsync(url);
            response.EnsureSuccessStatusCode();

            var result = await response.Content.ReadAsByteArrayAsync();
            return new List<byte>(result);
        }
    }

    public class curler_pool_t
    {
        private readonly List<curler_t> curlers;
        private readonly object lockObject = new object();

        public curler_pool_t(int poolSize, string userAgent)
        {
            curlers = new List<curler_t>(poolSize);
            for (int i = 0; i < poolSize; i++)
            {
                curlers.Add(new curler_t(userAgent));
            }
        }

        public curler_t acquire()
        {
            lock (lockObject)
            {
                if (curlers.Count == 0)
                {
                    throw new InvalidOperationException("No available curlers in the pool.");
                }

                var curler = curlers[curlers.Count - 1];
                curlers.RemoveAt(curlers.Count - 1);
                return curler;
            }
        }

        public void release(curler_t curler)
        {
            lock (lockObject)
            {
                curlers.Add(curler);
            }
        }
    }
}
