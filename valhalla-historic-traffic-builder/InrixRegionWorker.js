import fs from 'fs';
import { existsSync, mkdirSync, appendFile } from 'node:fs';
import csvParser from 'csv-parser';
import { createRequire } from 'module';
import sqlite3 from 'sqlite3';
import { open } from 'sqlite';
import { toGraphIdString, toTileIdString } from './GraphUtils.js';
import { dirname } from 'node:path';
import path from 'path';

const require = createRequire(import.meta.url);
const { compressSpeedBuckets } = require('./build/Release/compression.node');
import { encodeCompressedSpeeds } from './DCT.js';

// Get process arguments
const [processIndex, ...files] = process.argv.slice(2);

console.log(`Worker ${processIndex} starting with ${files.length} files`);

// Each worker has its own isolated resources
const speedBuckets = new Uint8Array(2016);
let records = 0;
let waysToEdgesDb, segmentsToWaysDb, segmentsStatement;

async function initializeWorker() {
  // Each worker gets its own database connections
  waysToEdgesDb = await open({
    filename: './ways_to_edges.db',
    driver: sqlite3.Database,
    mode: sqlite3.OPEN_READONLY
  });

  segmentsToWaysDb = await open({
    filename: './xds_segments.db',
    driver: sqlite3.Database,
    mode: sqlite3.OPEN_READONLY
  });

  segmentsStatement = await segmentsToWaysDb.prepare('SELECT * FROM segments WHERE seg_id = ?');
}

function encodeTrafficData(record) {
  if (record.length === 0) return;

  speedBuckets.fill(Math.floor(parseInt(record[0][3]) * 1.6));
  let prevDay = 0;

  for (let i = 1; i < record.length; i++) {
    const speedRecord = record[i];
    const day = parseInt(speedRecord[1]) - 1;
    const minutesSinceMidnight = parseInt(speedRecord[2]);
    const speed = Math.floor(parseInt(speedRecord[3]) * 1.6);

    const startIndex = day * 288 + (minutesSinceMidnight / 5);
    const endIndex = i === record.length - 1
      ? 2016
      : (record[i + 1][1] * 288 + (record[i + 1][2] / 5));

    speedBuckets.fill(speed, startIndex, endIndex);

    if (day !== prevDay) {
      speedBuckets.fill(speed, day * 288, startIndex);
    }
    prevDay = day;
  }

  const compressedSpeedValues = compressSpeedBuckets(speedBuckets);
  const encodedSpeedValues = encodeCompressedSpeeds(compressedSpeedValues);

  return {
    encodedSpeedValues,
    freeflow: speedBuckets[1116],
    constrained: speedBuckets[1068]
  };
}

async function processTrafficRecord(record) {
  const trafficRows = [];
  if (!record || record.length === 0) return trafficRows;

  const segmentId = record[0][0];

  try {
    const row = await segmentsStatement.get(segmentId);
    if (!row) return trafficRows;

    const osmIds = row.OSMWayIDs.split(';');
    const directions = row.OSMWayDirections.split(';');

    const placeholders = osmIds.map(() => '?').join(', ');
    const sql = `SELECT * FROM edges WHERE way_id IN (${placeholders})`;
    const rows = await waysToEdgesDb.all(sql, osmIds);

    const { encodedSpeedValues, freeflow, constrained } = encodeTrafficData(record);

    for (let i = 0; i < osmIds.length; i++) {
      const direction = directions[i] === 'P' ? 1 : 0;
      const osmId = osmIds[i];
      const data = rows.find(r => osmId === r.way_id)?.data;
      if (!data) continue;

      const edges = data.split('|');
      for (let j = 0; j < edges.length;) {
        const osmDirection = edges[j++];
        const edge = edges[j++];
        const length = edges[j++];
        const shortcut = edges[j++];

        if (osmDirection === direction || shortcut) {
          const edgeId = toGraphIdString(edge);
          const tileId = toTileIdString(edge);
          const csvRow = [edgeId, freeflow, constrained, encodedSpeedValues].join(',');
          trafficRows.push([tileId, csvRow]);
        }
      }
    }

    records += trafficRows.length;
    return trafficRows;

  } catch (err) {
    console.error(`Error processing record ${segmentId}:`, err);
  }
}

const writeTiles = async (tileBatches) => {
  const tiles = new Map();

  for (const pbfBatch of (tileBatches || [])) {
    for (const trafficBatch of (pbfBatch || [])) {
      const [tileId, csvRow] = trafficBatch;
      const rows = tiles.get(tileId) || [];
      rows.push(csvRow);
      tiles.set(tileId, rows);
    }
  }

  const writeFilePromises = [];
  for (const [tileId, rows] of tiles.entries()) {
    const file = `./traffic/${tileId}.csv`;
    const dir = dirname(file);
    if (!existsSync(dir)) mkdirSync(dir, { recursive: true });

    const content = rows.join('\n') + '\n';

    // Atomic appendFile prevents race conditions
    writeFilePromises.push(new Promise((resolve, reject) => {
      appendFile(file, content, { flag: 'a+' }, (err) => {
        if (err) reject(err);
        else resolve();
      });
    }));
  }

  await Promise.all(writeFilePromises);
};

const MAX_CONCURRENT = 100;
let activeCount = 0;
const queue = [];

function delay(ms) {
  return new Promise(resolve => setTimeout(resolve, ms));
}

function tryProcessNext(stream) {
  if (queue.length === 0 || activeCount >= MAX_CONCURRENT) return;

  const { id, group, resolve } = queue.shift();
  activeCount++;

  processGroup(id, group)
    .then(() => {
      activeCount--;
      resolve();
      tryProcessNext(stream);
      if (activeCount < MAX_CONCURRENT && stream.isPaused()) {
        stream.resume();
      }
    })
    .catch((err) => {
      activeCount--;
      console.error('Error processing group:', err);
      resolve();
      tryProcessNext(stream);
      if (activeCount < MAX_CONCURRENT && stream.isPaused()) {
        stream.resume();
      }
    });
}

async function enqueueGroup(id, group, stream) {
  if (activeCount >= MAX_CONCURRENT && !stream.isPaused()) {
    stream.pause();
  }

  return new Promise(resolve => {
    queue.push({ id, group, resolve });
    tryProcessNext(stream);
  });
}

async function processGroup(id, group) {
  const tileBatch = await processTrafficRecord(group);
  if (tileBatch?.length > 0) {
    await writeTiles([tileBatch]);
  }
}

async function processCsvStream(stream) {
  return new Promise((resolve, reject) => {
    let currentGroupId = null;
    let currentGroup = [];

    stream.on('data', async (row) => {
      const id = row[0];

      if (currentGroupId === null) {
        currentGroupId = id;
      }

      if (id !== currentGroupId) {
        stream.pause();
        await enqueueGroup(currentGroupId, currentGroup, stream);
        currentGroupId = id;
        currentGroup = [row];
        stream.resume();
      } else {
        currentGroup.push(row);
      }
    });

    stream.on('end', async () => {
      if (currentGroup.length > 0) {
        await enqueueGroup(currentGroupId, currentGroup, stream);
      }

      while (activeCount > 0) {
        await delay(100);
      }
      resolve();
    });

    stream.on('error', reject);
  });
}

// Progress reporting
const progressInterval = setInterval(() => {
  console.log(`Worker ${processIndex}: ${records} records, ${Math.round(process.memoryUsage().heapUsed / 1024 / 1024)}MB`);
}, 5000);

// Main processing loop
async function processFiles() {
  await initializeWorker();

  for (const file of files) {
    try {
      console.log(`Worker ${processIndex} processing: ${file}`);
      const filePath = path.join('./data/inrix', file);
      const stream = fs.createReadStream(filePath).pipe(csvParser({ headers: false }));
      await processCsvStream(stream);

      console.log(`Worker ${processIndex} completed: ${file} (${records} records)`);

      // Reset records for next file
      records = 0;
    } catch (error) {
      console.error(`Worker ${processIndex} error processing ${file}:`, error.message);
    }
  }

  clearInterval(progressInterval);
  console.log(`Worker ${processIndex} completed all files`);
  process.exit(0);
}

// Start processing
processFiles();
