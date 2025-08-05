## Updating the timezone database

The timezone information is coming from 2 repositories:
- [eggert/tz](https://github.com/eggert/tz): Contains the IANA rules and prepares releases for each IANA release. We have this repo submodule'd and configure the files via CMake.
- [evansiroky/timezone-boundary-builder](https://github.com/evansiroky/timezone-boundary-builder): Contains the geometries of IANA timezones, mostly sourced from OSM. Releases comprise the `timezones-with-oceans.shapefile.zip` shapefile which we use to build our timezone SQLite database.

Updating the timezone information regularly is paramount to keep up with:
- renamed/merged timezones, which implicitly deprecates (but still preserves) old timezones
- geometry changes of timezones
- entirely new timezones which are carved out due to local/regional DST changes
- DST changes of existing timezones

DST changes are the most important reason to update regularly.

### Update process

1. Update the `tz` submodule
    ```
    git -C third_party/tz checkout <release_tag>
    ```
2. Update `scripts/valhalla_build_timezones` to download the latest release
3. Run `datetime` test. If any timezones were merged/renamed, it'll fail with a pretty-print of new/old elements for copy/pasting convenience. However, if entirely new timezones were added, there's more manual work:
- identify which timezone the new one is carved out of (look into the submoduled `tz` repo's NEWS)
- if the parent timezone has an ID < 387, it just needs a bit shift of the parent timezone
- if the parent timezone is itself a previously added new timezone, one might add another field to NodeInfo; follow the instructions in baldr/nodeinfo.h/cc
4. Sanity check step 3! Both looking at the release notes of the IANA data and adding test cases.
### Important notes
- The 2 repos we source our information from are not always in-sync. If there's a IANA release being skipped in the boundary builder project, it usually means that nothing changed wrt geometries. However, you'll need to verify that.
- Regarding data compatibility:
    - **new code/old data**: the algorithms can ask for nodes' timezone indices of potentially deprecated timezones (old tile data), and it'll receive the new timezone information, which is not a problem
    - **old code/new data**: for entirely new timezones, the new data has an additional field in NodeInfo which tells new code that it's a new timezone, while old code continues to see the new timezone's parent.