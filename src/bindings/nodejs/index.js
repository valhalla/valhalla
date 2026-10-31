const { getBinaryDir } = require('./lib/binary-path');
const path = require('path');
const fs = require('fs');
const { spawn } = require('child_process');

function getBinaryPath(baseDir) {
    return path.join(getBinaryDir(baseDir), 'valhalla_node.node');
}

const valhalla = require(getBinaryPath(__dirname));

class Actor {
    constructor(config) {
        if (typeof config !== 'string') {
            config = JSON.stringify(config);
        }
        this.actor = new valhalla.Actor(config);
    }

    static async fromConfigFile(configFile) {
        const config = await fs.promises.readFile(configFile, 'utf8');
        return new Actor(config);
    }

    async _callActor(method, query) {
        let request = query;
        let returnBinary = false;

        if (typeof query !== 'string') {
            request = JSON.stringify(query);
            if (query.format === 'pbf') {
                returnBinary = true;
            }
        } else {
            try {
                const parsed = JSON.parse(query);
                if (parsed.format === 'pbf') {
                    returnBinary = true;
                }
            } catch (e) {
                // If it's not valid JSON, let the C++ actor handle the error
            }
        }

        const result = await this.actor[method](request, returnBinary);

        if (returnBinary) {
            return result; // It's a Buffer
        }

        // It's a JSON string, parse it to Object
        return JSON.parse(result);
    }

    async route(query) {
        return this._callActor('route', query);
    }

    async locate(query) {
        return this._callActor('locate', query);
    }

    async matrix(query) {
        return this._callActor('matrix', query);
    }

    async optimizedRoute(query) {
        return this._callActor('optimizedRoute', query);
    }

    async isochrone(query) {
        return this._callActor('isochrone', query);
    }

    async traceRoute(query) {
        return this._callActor('traceRoute', query);
    }

    async traceAttributes(query) {
        return this._callActor('traceAttributes', query);
    }

    async height(query) {
        return this._callActor('height', query);
    }

    async transitAvailable(query) {
        return this._callActor('transitAvailable', query);
    }

    async expansion(query) {
        return this._callActor('expansion', query);
    }

    async centroid(query) {
        return this._callActor('centroid', query);
    }

    async status(query) {
        return this._callActor('status', query);
    }

    async tile(query) {
        // Tile always returns binary (MVT)
        if (typeof query === 'string') {
            return this.actor.tile(query);
        }
        return this.actor.tile(JSON.stringify(query));
    }
}

async function getConfig(options = {}) {
    const {
        tileDir = 'valhalla_tiles',
        tileExtract = 'valhalla_tiles.tar',
        verbose = false,
        additionalArgs = []
    } = options;

    const scriptPath = path.join(__dirname, 'valhalla_build_config');
    const args = [];

    if (tileDir) {
        args.push('--mjolnir-tile-dir', path.resolve(tileDir));
    }
    if (tileExtract) {
        args.push('--mjolnir-tile-extract', path.resolve(tileExtract));
    }
    args.push('--mjolnir-logging-type', verbose ? 'std_out' : '');
    args.push(...additionalArgs);

    return new Promise((resolve, reject) => {
        const proc = spawn(scriptPath, args);

        let stdout = '';
        let stderr = '';

        proc.stdout.on('data', (data) => {
            stdout += data.toString();
        });

        proc.stderr.on('data', (data) => {
            stderr += data.toString();
        });

        proc.on('error', (error) => {
            reject(new Error(`Failed to execute valhalla_build_config: ${error.message}`));
        });

        proc.on('close', (code) => {
            if (code !== 0) {
                const errorMsg = stderr || stdout || 'Unknown error';
                reject(new Error(`valhalla_build_config failed with exit code ${code}: ${errorMsg}`));
                return;
            }

            try {
                const config = JSON.parse(stdout);
                resolve(config);
            } catch (error) {
                reject(new Error(`Failed to parse Valhalla config JSON: ${error.message}`));
            }
        });
    });
}

const GraphId = valhalla.GraphId;
const getTileBaseLonLat = valhalla.getTileBaseLonLat;
const getTileIdFromLonLat = valhalla.getTileIdFromLonLat;
const getTileIdsFromBbox = valhalla.getTileIdsFromBbox;
const getTileIdsFromRing = valhalla.getTileIdsFromRing;

module.exports = { Actor, GraphId, getConfig, getTileBaseLonLat, getTileIdFromLonLat, getTileIdsFromBbox, getTileIdsFromRing, VALHALLA_VERSION: valhalla.VALHALLA_VERSION };