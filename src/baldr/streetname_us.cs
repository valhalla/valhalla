using System;
using System.Collections.Generic;

namespace valhalla.baldr
{
    public class StreetNameUs : StreetName
    {
        private static readonly List<string> pre_dirs_ = new List<string>
        {
            "North ", "East ", "South ", "West ", "Northeast ", "Southeast ", "Southwest ", "Northwest "
        };

        private static readonly List<string> post_dirs_ = new List<string>
        {
            " North", " East", " South", " West", " Northeast", " Southeast", " Southwest", " Northwest"
        };

        private static readonly List<string> post_cardinal_dirs_ = new List<string>
        {
            " North", " East", " South", " West"
        };

        public StreetNameUs(string value, bool is_route_number, Pronunciation? pronunciation = null)
            : base(value, is_route_number, pronunciation)
        {
        }

        public override string GetPreDir()
        {
            foreach (var pre_dir in pre_dirs_)
            {
                if (StartsWith(pre_dir))
                {
                    return pre_dir;
                }
            }
            return "";
        }

        public override string GetPostDir()
        {
            foreach (var post_dir in post_dirs_)
            {
                if (EndsWith(post_dir))
                {
                    return post_dir;
                }
            }
            return "";
        }

        public override string GetPostCardinalDir()
        {
            foreach (var post_cardinal_dir in post_cardinal_dirs_)
            {
                if (EndsWith(post_cardinal_dir))
                {
                    return post_cardinal_dir;
                }
            }
            return "";
        }

        public override string GetBaseName()
        {
            var pre_dir = GetPreDir();
            var post_dir = GetPostDir();

            return value_.Substring(pre_dir.Length, value_.Length - pre_dir.Length - post_dir.Length);
        }

        public override bool HasSameBaseName(StreetName rhs)
        {
            return GetBaseName() == rhs.GetBaseName();
        }
    }
}
