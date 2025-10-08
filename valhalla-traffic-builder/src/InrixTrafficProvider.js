import fs from 'fs';
import axios from 'axios';
import sqlite3 from 'sqlite3';
import { open } from 'sqlite';
import dotenv from 'dotenv'
import { toGraphIdString } from './GraphUtil.js';
import { createRequire } from 'module';
import path from 'path'
const require = createRequire(import.meta.url);
import * as Sentry from '@sentry/node';
import { fileURLToPath } from "url";

// ESM-safe __filename and __dirname
const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
Sentry.init({
  dsn: process.env.SENTRY_DSN,
});

dotenv.config();
const INRIX_APP_ID = process.env.INRIX_APP_ID;
const INRIX_HASH_TOKEN = process.env.INRIX_HASH_TOKEN;
if (!INRIX_APP_ID || !INRIX_HASH_TOKEN) {
  throw new Error('Inrix App Id or Inrix Hash Token not set! Make sure to add it to .env file');
}


// in-memory lookups
const TILE_OFFSET = new Map();

// tileOffset -> { edgeIndex: trafficData }
const OUTPUT = new Map();

class InrixTrafficProvider {

  static writeToValhalla(binding, trafficPath) {
    OUTPUT.forEach((edgesMap, offset) => {

      const edgeValues = [...edgesMap.entries()].flatMap(
        ([k, v]) => [parseInt(k, 10), ...v.split(',').map(Number)]
      )
      try {
        return binding.handleTileTraffic(parseInt(offset), edgeValues, Date.now(), trafficPath);
      } catch (error) {
        Sentry.captureMessage(`Failed to update tile at offset ${offset}: ${error.message}`);
        console.error(`Failed to update tile ${offset}:`, error.message);
        throw error;
      }
    });

  }

  static async makeRequests(batch, token) {
    const baseUrl = 'https://segment-api.inrix.com/v1/segments/speed';

    const requests = batch.map(quadkey => {
      const url = `${baseUrl}?quadkey=${quadkey}&Coverage=8&SpeedOutputFields=all&units=1&FRCLevel=1,2,3,4,5&accesstoken=${token}`;
      return axios.get(url)
        .then(response => ({ quadkey, data: response.data.result }))
        .catch(error => ({ quadkey, error }));
    });


    const results = await Promise.all(requests);
    return results;
  }

  static async downloadBatch(batch, bindingPath, trafficPath, inputPath) {

    if (!batch || (batch.filter(quadkey => quadkey?.length > 0)).length === 0) {
      return;
    }


    const resolvedPath = path.resolve(__dirname, "..", bindingPath);
    const bindings = require(resolvedPath);

    // load tile offset data
    fs.readFileSync(`${inputPath}/traffic_tile_offset.csv`).toString().split('\n').forEach((row) => {
      const [tileId, offset] = row.split(',');
      TILE_OFFSET.set(tileId, offset);
    });

    // load the ways lookup db
    const db = await open({
      filename: `${inputPath}/ways_to_edges.db`,
      driver: sqlite3.Database,
      mode: sqlite3.OPEN_READONLY
    });

    // Set read-optimized pragmas
    await db.exec('PRAGMA journal_mode = WAL');
    await db.exec('PRAGMA cache_size = 5000');
    await db.exec('PRAGMA temp_store = MEMORY');


    // load the ways lookup db
    const segmentsToWaysDb = await open({
      filename: `${inputPath}/xds_segments.db`,
      driver: sqlite3.Database,
      mode: sqlite3.OPEN_READONLY
    });

    const segmentsStatement = await segmentsToWaysDb.prepare('SELECT * FROM segments WHERE seg_id = ?');


    const res = await axios.get(`https://uas-api.inrix.com/v1/appToken?appId=${INRIX_APP_ID}&hashToken=${INRIX_HASH_TOKEN}`);
    const inrixAppToken = res?.data?.result?.token

    const data = await this.makeRequests(batch, inrixAppToken);

    const promises = data.map(async batch => {
      await this.processLiveData(batch, db, segmentsStatement);
    });
    await Promise.all(promises);

    this.writeToValhalla(bindings, trafficPath);

  }

  static async processLiveData(batch, db, segmentsStatement) {
    if (batch.error) {
      console.log(`Error fetching data for quadkey ${batch.quadkey}:`, batch.error.message);
      Sentry.captureMessage(`Error fetching data for quadkey ${batch.quadkey}: ${batch.error.message}`);
      return;
    }

    console.log("Processing batch:", batch.quadkey);
    const segments = batch.data.segmentspeeds[0].segments; // TODO: check why is there so much indexing, do those list ever have more than one element? Maybe around the border

    const unit = batch.data.unit;
    const promises = segments.map(async segment => {
      let { code: segmentId, speed } = segment;


      // Inrix Speed Adjustments
      // Reference speed is freeflow speed at that datetime
      if (speed / segment.reference <= 0.25) { // Inrix really tends to underestimate heavy heavy traffic. It knows where the traffic jams are, but doesn't make them cost enough 10km/h -> 6km/h
        speed = speed * 0.6;
      } else if (speed / segment.reference <= 0.4) { // Lighter version of above rule
        speed = speed * 0.75;
      } else if (segment.reference < 30 && speed / segment.reference < 0.75) { // Apply higher cost to traffic set in downtowns (where segment.reference is very slow)
        speed = speed * 0.65;
      } else if (segment.reference < 30) { // Inrix tends to overestimate speed in downtowns
        speed = speed * 0.85;
      }

      if (segment.segmentClosed) {
        speed = 0;
      }

      try {
        const row = await segmentsStatement.get(segmentId);

        if (!row) {
          return;
        }

        const osmIds = row.OSMWayIDs.split(';');
        const directions = row.OSMWayDirections.split(';');

        const placeholders = osmIds.map(() => '?').join(', ');
        const sql = `SELECT * FROM edges WHERE way_id IN (${placeholders})`;
        const rows = await db.all(sql, osmIds);


        for (let i = 0; i < osmIds.length; i++) {
          const direction = directions[i] === 'P' ? 1 : 0;
          const osmId = osmIds[i];

          const data = rows.find(r => osmId === r.way_id)?.data;
          if (!data) {
            continue;
          }
          const edges = data.split('|');

          for (let j = 0; j < edges.length;) {
            const osmDirection = parseInt(edges[j++]);
            const edgeId = edges[j++];
            const length = parseInt(edges[j++]);
            const shortcut = parseInt(edges[j++]);

            if (osmDirection === direction || shortcut) {
              const tileId = toGraphIdString(edgeId);
              const edge = { forward: osmDirection, length, tileId, edgeId, osmId, shortcut }
              const multiplier = unit === 'MPH' ? 1.60934 : 1; // convert to km/h if needed

              this.addEdgeToOutput(edge, Math.floor(speed * multiplier));
            }
          }
        }
      } catch (error) {
        console.error(`Error processing record ${segmentId}:`, error);
      }

    });

    await Promise.all(promises);

  }

  static addEdgeToOutput(edge, speed) {
    const { tileId } = edge;
    const tileGraphId = tileId.split('/');
    const tileOffset = TILE_OFFSET.get(`${tileGraphId[0]}/${tileGraphId[1]}/0`);
    if (tileOffset) {
      let edgeIndex = tileGraphId[2];
      let overallSpeed = Math.floor(speed / 2); // Valhalla multiplies by 2 for some reason
      let speed1 = Math.floor(speed / 2);
      let speed2 = Math.floor((speed / 2));  // Set this as a flag var
      const speed3 = Math.floor(new Date().getMinutes() / 2)
      let breakpoint1 = 255; // don't use breakpoints for now.
      let breakpoint2 = 0;
      // let congestion1 = 0;
      // let congestion2 = 0;
      // let congestion3 = 0;
      // let hasIncidents = 0;

      // add to output
      // use a Map to avoid duplicates - always take the latest edge update
      // inrix overestimates take the smaller speed
      // TODO: is it OK to dedupe on the edgeIndex?
      const rows = OUTPUT.get(tileOffset) || new Map();

      const prevTrafficData = rows.get(edgeIndex);
      if (prevTrafficData) {
        const prevValues = prevTrafficData.split(',').map(Number);
        overallSpeed = Math.min(overallSpeed, prevValues[0]);
        speed1 = Math.min(speed1, prevValues[1]);
        speed2 = Math.min(speed2, prevValues[2]);
      }

      const trafficData = `${overallSpeed},${speed1},${speed2},${speed3},${breakpoint1},${breakpoint2}`;
      rows.set(edgeIndex, trafficData);
      OUTPUT.set(tileOffset, rows);
      return true;
    }

    return false;
  }


  static async makeRequestsIncidents(batch, token) {
    const baseUrl = 'https://incident-api.inrix.com/v1/incidents';

    const requests = batch.map(incident => {
      const bbox = incident.split(',');
      const url = `${baseUrl}?box=${bbox[0]}|${bbox[1]},${bbox[2]}|${bbox[3]}&roadSegmentType=OSM&incidentType=Incidents,Flow,Construction&status=active&incidentoutputfields=InrixCode,Location&accesstoken=${token}`;

      return axios.get(url)
        .then(response => ({ incident, data: response.data.result }))
        .catch(error => ({ incident, error }));
    });


    const results = await Promise.all(requests);
    return results;
  }


  static async processIncidents(batch, db) {
    if (batch.error) {
      console.log(`Error fetching data for incident ${batch.incident}:`, batch.error.message);
      Sentry.captureMessage(`Error fetching data for incident ${batch.incident}: ${batch.error.message}`);
      return;
    }

    console.log("Processing incident:", batch.incident);

    const closureCodes = new Set(["8", "220", "223", "224", "262", "263", "592", "626", "628", "632"]); // inrix codes we care about using: https://docs.inrix.com/reference/inrixcode/
    const rampClosures = new Set(["262", "263", "592", "626", "628", "632"])
    const incidents = (batch.data.incidents || []).filter(incident =>
      incident.messages?.inrixMessage?.some(msg => closureCodes.has(msg.inrixCode))
    );

    const promises = incidents.map(async (incident, i) => {
      const isRampEdge = incident.messages?.inrixMessage?.some(msg => rampClosures.has(msg.inrixCode));


      const osmIds = [];

      if (isRampEdge) {
        return; // for now don't use
        osmIds.push(
          ...(incident.locationRamp || []).map(segment => ({
            code: segment.code,
            direction: 1
          }))
        );
      } else {
        osmIds.push(
          ...(incident.location?.segments || []).map(segment => ({
            code: segment.code,
            direction: 1
          }))
        );


        osmIds.push(
          ...(incident.location?.oppositeSegments || []).map(segment => ({
            code: segment.code,
            direction: 0
          }))
        );
      }

      try {

        if (osmIds.length === 0) {
          return;
        }

        const placeholders = osmIds.map(() => '?').join(', ');
        const sql = `SELECT * FROM edges WHERE way_id IN (${placeholders})`;
        const rows = await db.all(sql, osmIds.map(o => o.code));

        osmIds.forEach(osmIdObj => {
          const data = rows.find(r => osmIdObj.code === r.way_id)?.data;
          if (!data) {
            return;
          }
          const edges = data.split('|');

          for (let j = 0; j < edges.length;) {
            const osmDirection = parseInt(edges[j++]);
            const edgeId = edges[j++];
            const length = parseInt(edges[j++]);
            const shortcut = parseInt(edges[j++]);

            if (osmDirection === osmIdObj.direction || shortcut) {
              const tileId = toGraphIdString(edgeId);
              const edge = { forward: osmDirection, length, tileId, edgeId, osmId: osmIdObj.code, shortcut }

              this.addEdgeToOutput(edge, 0);
            }
          }

        });
      } catch (error) {
        console.error(`Error processing record ${incident}:`, error);
      }

    });

    await Promise.all(promises);

  }
  static async downloadBatchIncidents(incidents, bindingPath, trafficPath, inputPath) {
    if (!incidents || (incidents.filter(incident => incident?.length > 0)).length === 0) {
      return;
    }

    const resolvedPath = path.resolve(__dirname, "..", bindingPath);
    const bindings = require(resolvedPath);

    // load tile offset data
    fs.readFileSync(`${inputPath}/traffic_tile_offset.csv`).toString().split('\n').forEach((row) => {
      const [tileId, offset] = row.split(',');
      TILE_OFFSET.set(tileId, offset);
    });

    // load the ways lookup db
    const db = await open({
      filename: `${inputPath}/ways_to_edges.db`,
      driver: sqlite3.Database,
      mode: sqlite3.OPEN_READONLY
    });

    // Set read-optimized pragmas
    await db.exec('PRAGMA journal_mode = WAL');
    await db.exec('PRAGMA cache_size = 5000');
    await db.exec('PRAGMA temp_store = MEMORY');


    const res = await axios.get(`https://uas-api.inrix.com/v1/appToken?&appId=${INRIX_APP_ID}&hashToken=${INRIX_HASH_TOKEN}`);
    const inrixAppToken = res?.data?.result?.token

    const data = await this.makeRequestsIncidents(incidents, inrixAppToken);

    const promises = data.map(async incidents => {
      await this.processIncidents(incidents, db);
    });
    await Promise.all(promises);

    this.writeToValhalla(bindings, trafficPath);
  }
}

export default InrixTrafficProvider;
