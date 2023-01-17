#Automatic Map Roulette Task Updates

## Overview
The tool is located under `mjonir/py/challenge_admin/` as  `admin_tool.py` and is run from the command line. This tool works in conjunction with `valhalla_build_statistics` which detects errors in the OpenStreetMap data set and outputs those errors as a geojson file. This tool then reads the geojson file and compares it to the existing tasks already uploaded to Map Roulette and uploads the new tasks. Optionally, the tool can also detect when tasks have been marked as fixed, but have shown up again during the detection process.

## Configuring
A sample configuration file can be found at [valhalla/conf/maproulette.json](https://github.com/valhalla/conf/blob/master/maproulette.json) and must be configured before this tool will work properly. Additionally, you will have to do some setup on the [maproulette website](http://maproulette.org).

### Perform Map Roulette Setup
#### Create a challenge
To get started you'll need to create a challenge using the interface on the website. Once you have created a challenge, you will just need the challenge ID. You will put the challenge ID into your configuration file later.

#### Get an API key
On the Map Roulette website click on your username at the top right, then select "Profile". Your API key can be found in the side panel on the left.

### Modify the Config File
Now that you have the information you need, substitute your api key into the config file. Then, you have to set up your challenges entry. A simple entry will look something like:

    "challenges":
        {
            "18": ["Loop", "Node"]
        }

This simply specifies that challenge number 18 will contain tasks of type Loop and Node. The challenge number is the one that you found after creating the challenge. The task types are the types of tasks you specified in the geojson.
If you have gotten here and don't know what geojson you should have, [click here](https://github.com/valhalla/mjolnir/blob/master/docs/geojson.md) to learn more.

Finally, make sure that you change the `server_url` field to reflect the actual server (http://maproulette.org) if not running the server locally.

## Using this tool
Usage of the tool is fairly simple as it is designed to be largely automatic. All you need to do is provide some command line arguments.

### Required Arguments
There are two required Arguments:

    -c, --config    json config file to use
    -i, --geojson   geojson to build new tasks from

There are also others available:

    -h, --help      how to call the program
    -r, --resubmit  try to detect tasks that have not been fixed,
                    if there are unfixed tasks marked as fixed,
                    resubmit them as new tasks