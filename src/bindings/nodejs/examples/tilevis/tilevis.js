#!/usr/bin/env node

/**
 * Valhalla Vector Tile Visualization Server
 * 
 * A simple HTTP server that serves vector tiles using Valhalla's tile method.
 * Compatible with MapLibre GL JS and Mapbox GL JS.
 */

const http = require('http');
const path = require('path');
const fs = require('fs');

function showHelp() {
  console.log(`
Valhalla Vector Tile Visualization Server

Usage:
  node tilevis.js [options]

Options:
  --config <path>   Path to Valhalla config file (default: valhalla.json)
  --port <number>   Port to listen on (default: 8080)
  --host <host>     Host to bind to (default: localhost)
  --help, -h        Show this help message

Environment Variables:
  VALHALLA_NODE          Path to custom valhalla_node.node binary

Examples:
  # Use local build
  VALHALLA_NODE=../../build/lib/valhalla_node.node node tilevis.js
  
  # Use npm package
  npm install @valhallajs/valhallajs
  node tilevis.js --config ./valhalla.json
`);
}

function parseArgs() {
  const args = process.argv.slice(2);
  const options = {
    configPath: 'valhalla.json',
    port: 8080,
    host: 'localhost'
  };

  for (let i = 0; i < args.length; i++) {
    if (args[i] === '--config' && i + 1 < args.length) {
      options.configPath = args[++i];
    } else if (args[i] === '--port' && i + 1 < args.length) {
      options.port = parseInt(args[++i], 10);
    } else if (args[i] === '--host' && i + 1 < args.length) {
      options.host = args[++i];
    } else if (args[i] === '--help' || args[i] === '-h') {
      showHelp();
      process.exit(0);
    }
  }

  return options;
}

function loadConfig(configPath) {
  try {
    const configData = fs.readFileSync(configPath, 'utf8');
    const config = JSON.parse(configData);
    console.log(`Loaded config from: ${configPath}`);
    return config;
  } catch (error) {
    console.error(`Failed to load config from ${configPath}:`, error.message);
    console.error('\nPlease provide a valid Valhalla configuration file.');
    process.exit(1);
  }
}

function initializeActor(valhalla, config) {
  try {
    const actor = new valhalla.Actor(JSON.stringify(config));
    console.log('Valhalla actor initialized successfully');
    if (valhalla.VALHALLA_VERSION) {
      console.log(`Valhalla version: ${valhalla.VALHALLA_VERSION}`);
    }
    return actor;
  } catch (error) {
    console.error('Failed to initialize Valhalla actor:', error.message);
    process.exit(1);
  }
}

function createRequestHandler(actor) {
  return async (req, res) => {
    const url = new URL(req.url, `http://${req.headers.host}`);
    
    // Serve index.html
    if (url.pathname === '/' || url.pathname === '') {
      const indexPath = path.join(__dirname, 'index.html');
      try {
        const indexHtml = fs.readFileSync(indexPath, 'utf8');
        
        res.writeHead(200, { 
          'Content-Type': 'text/html',
          'Cache-Control': 'no-cache'
        });
        res.end(indexHtml);
      } catch (error) {
        res.writeHead(500, { 'Content-Type': 'text/plain' });
        res.end('index.html not found');
      }
      return;
    }
    
    // Validate tile coordinates from URL: /?json={"tile": {"z": 10, "x": 10, "y": 10}}
    let error = false;
    const jsonString = url.searchParams.get('json');
    if (!jsonString) {
      error = true;
    }
    const data = JSON.parse(decodeURIComponent(jsonString));
    const { x = null, y = null, z = null } = data?.tile ?? {};
    if ([x, y, z].some(v => typeof v !== 'number')) {
      error = true;
    }
    
    if (error) {
      res.writeHead(404, { 'Content-Type': 'text/plain' });
      res.end('Not found. Expected format: /?json={"tile": {"z": 10, "x": 10, "y": 10}}');
      return;
    }
    
    // Validate tile coordinates
    const maxCoord = Math.pow(2, z);
    if (z < 0 || z > 30 || x < 0 || x >= maxCoord || y < 0 || y >= maxCoord) {
      res.writeHead(400, { 'Content-Type': 'text/plain' });
      res.end(`Invalid tile coordinates: z=${z}, x=${x}, y=${y}`);
      return;
    }

    try {
      const tileData = await actor.tile(JSON.stringify(data));
      const buf = Buffer.isBuffer(tileData) ? tileData : Buffer.from(tileData, 'binary');
      //console.log(buf.length)
      
      res.writeHead(200, {
        'Content-Type': 'application/vnd.mapbox-vector-tile',
        'Content-Length': buf.length,
        'Access-Control-Allow-Origin': '*',
        'Access-Control-Allow-Methods': 'GET',
        'Cache-Control': 'no-store'
      });
      
      res.end(buf);
    } catch (error) {
      console.error(`Error generating tile z=${z}, x=${x}, y=${y}:`, error.message);
      res.writeHead(500, { 'Content-Type': 'text/plain' });
      res.end(`Error generating tile: ${error.message}`);
    }
  };
}

function startServer(server, host, port) {
  server.listen(port, host, () => {
    console.log(`
Server listening at: http://${host}:${port}
Tile URL format:     http://${host}:${port}/?json={"tile": {"z": 10, "x": 10, "y": 10}}

Open http://${host}:${port} in your browser to view the map
Press Ctrl+C to stop
`);
  });

  // Graceful shutdown
  const shutdown = () => {
    console.log('\n\nShutting down...');
    server.close(() => {
      console.log('Server stopped');
      process.exit(0);
    });
  };

  process.on('SIGINT', shutdown);
  process.on('SIGTERM', shutdown);
}

function main() {
  const options = parseArgs();
  const valhalla = require(process.env.VALHALLA_NODE ?? '@valhallajs/valhallajs');

  const config = loadConfig(options.configPath);

  const actor = initializeActor(valhalla, config);

  const server = http.createServer(createRequestHandler(actor));
  startServer(server, options.host, options.port);
}

main();
