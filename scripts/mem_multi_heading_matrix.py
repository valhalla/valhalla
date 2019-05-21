# git pull https://github.com/valhalla/valhalla.git master


import sys
import os
import requests
from datetime import datetime
import json

outputMatrix = sys.argv[1]


def requestLocations(stops):
    resultsStops = []
    locations = []

    for stop in stops:
        contents1 = stop.split(",")
        latFrom = contents1[0]
        lngFrom = contents1[1]
        combinedCoodrs = "{" + "\"lon\":" + lngFrom + "," + "\"lat\":" + latFrom + "," + "\"name\":\"from\"" + "}"
        locations.append(combinedCoodrs)
    locations = ', '.join(locations)
    data = '{"locations":[' + locations + '],"costing":"auto","verbose":false}'
    headers = {
        'Content-type': 'application/json',
    }
    print("sending request with: ","http://localhost:8002/locate --data ","'"+data+"'")

    start_time = datetime.now()

    # {"locations":[{"lon":35.169715,"lat":136.931980}],"costing":"auto", "verbose":true"}
    # curl http://localhost:8002/locate --data '{"locations":[{"lat":35.169715,"lon":136.931980},{"lat":35.169419,"lon":136.931899}],"costing":"auto","verbose":true}'
    raw_response = requests.post('http://localhost:8002/locate', headers=headers, data=data)
    json_response = raw_response.json()
    for edgeObjs in json_response:

        # print("input:", edgeObjs)
        edge_headings = []
        for edge in edgeObjs["edges"]:
            input_lon = edgeObjs['input_lon']
            input_lat = edgeObjs['input_lat']
            correlated_lon = edge["correlated_lon"]
            correlated_lat = edge["correlated_lat"]
            edge_heading = edge["edge_heading"]
            edge_headings.append(edge_heading)
            resultsStops.append(str(input_lat) + "," + str(input_lon) + "," + str(edge_heading))
    return resultsStops


stopsFrom = ["35.202929,136.891067"]
stopsTo = ["35.172888,136.890023"]
stopsFromWithHeadings=requestLocations(stopsFrom)
stopsToWithHeadings=requestLocations(stopsTo)

print(stopsFromWithHeadings)
print(stopsToWithHeadings)

if outputMatrix:

    fromlocations = []
    tolocations = []

    for stopFrom in stopsFromWithHeadings:
        contents1 = stopFrom.split(",")
        latFrom = contents1[0]
        lngFrom = contents1[1]
        headingFrom = contents1[2]

        for stopTo in stopsToWithHeadings:
            contents2 = stopTo.split(",")
            latTo = contents2[0]
            lngTo = contents2[1]
            headingTo = contents2[2]
            combinedCoodrsFrom = "{" + "\"lon\":" + lngFrom + "," + "\"lat\":" + latFrom + "," + "\"heading\":" + headingFrom + "}"
            combinedCoodrsTo = "{" + "\"lon\":" + lngTo + "," + "\"lat\":" + latTo + "," + "\"heading\":" + headingTo + "}"

            fromlocations.append(combinedCoodrsFrom)
            tolocations.append(combinedCoodrsTo)

    fromlocations = ', '.join(fromlocations)
    tolocations = ', '.join(tolocations)
    # print(fromlocations)

    data = '{"sources":[' + fromlocations + '],"targets":[' + tolocations + '],"costing":"auto"}'

    headers = {
        'Content-type': 'application/json',
    }

    # curl http://localhost:8002/sources_to_targets --data '{"sources":[{"lat":35.186695,"lon":136.924255,"heading":100},{"lat":35.186695,"lon":136.924255,"heading":76}],"targets":[{"lat":35.186218,"lon":136.923218,"heading":88},{"lat":35.186218,"lon":136.923218,"heading":320}],"costing":"auto","directions_options":{"units":"km"}}'
    print("sending request with: ","http://localhost:8002/lsources_to_targets --data ","'"+data+"'")
    start_time = datetime.now()

    raw_response = requests.post('http://localhost:8002/sources_to_targets', headers=headers, data=data)
    time_elapsed = datetime.now() - start_time
    run_secs = time_elapsed.total_seconds()
    json_response = raw_response.json()

    pretty_reponse = json.dumps(json_response, indent=4, sort_keys=True)
    print(pretty_reponse)
    print('Time elapsed: ' +str(run_secs))

