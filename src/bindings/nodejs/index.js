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

    async route(query) {
        if (typeof query === 'string') {
            return this.actor.route(query);
        }
        const result = await this.actor.route(JSON.stringify(query));
        if (query.format === 'pbf' || Buffer.isBuffer(result)) {
            return result;
        }
        return JSON.parse(result);
    }

    async locate(query) {
        if (typeof query === 'string') {
            return this.actor.locate(query);
        }
        return JSON.parse(await this.actor.locate(JSON.stringify(query)));
    }

    async matrix(query) {
        if (typeof query === 'string') {
            return this.actor.matrix(query);
        }
        const result = await this.actor.matrix(JSON.stringify(query));
        if (query.format === 'pbf' || Buffer.isBuffer(result)) {
            return result;
        }
        return JSON.parse(result);
    }

    async optimizedRoute(query) {
        if (typeof query === 'string') {
            return this.actor.optimizedRoute(query);
        }
        return JSON.parse(await this.actor.optimizedRoute(JSON.stringify(query)));
    }
    
    async isochrone(query) {
        if (typeof query === 'string') {
            return this.actor.isochrone(query);
        }
        const result = await this.actor.isochrone(JSON.stringify(query));
        if (query.format === 'pbf' || Buffer.isBuffer(result)) {
            return result;
        }
        return JSON.parse(result);
    }

    async traceRoute(query) {
        if (typeof query === 'string') {
            return this.actor.traceRoute(query);
        }
        const result = await this.actor.traceRoute(JSON.stringify(query));
        if (query.format === 'pbf' || Buffer.isBuffer(result)) {
            return result;
        }
        return JSON.parse(result);
    }
    
    async traceAttributes(query) {
        if (typeof query === 'string') {
            return this.actor.traceAttributes(query);
        }
        const result = await this.actor.traceAttributes(JSON.stringify(query));
        if (query.format === 'pbf' || Buffer.isBuffer(result)) {
            return result;
        }
        return JSON.parse(result);
    }

    async height(query) {
        if (typeof query === 'string') {
            return this.actor.height(query);
        }
        return JSON.parse(await this.actor.height(JSON.stringify(query)));
    }

    async transitAvailable(query) {
        if (typeof query === 'string') {
            return this.actor.transitAvailable(query);
        }
        return JSON.parse(await this.actor.transitAvailable(JSON.stringify(query)));
    }

    async expansion(query) {
        if (typeof query === 'string') {
            return this.actor.expansion(query);
        }
        const result = await this.actor.expansion(JSON.stringify(query));
        if (query.format === 'pbf' || Buffer.isBuffer(result)) {
            return result;
        }
        return JSON.parse(result);
    }

    async centroid(query) {
        if (typeof query === 'string') {
            return this.actor.centroid(query);
        }
        return JSON.parse(await this.actor.centroid(JSON.stringify(query)));
    }
    
    
    async status(query) {
        if (typeof query === 'string') {
            return this.actor.status(query);
        }
        return JSON.parse(await this.actor.status(JSON.stringify(query)));
    }
    
    async tile(query) {
        if (typeof query === 'string') {
            return this.actor.tile(query);
        }
        // tile returns raw binary data (MVT), not JSON
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

module.exports = { Actor, getConfig, VALHALLA_VERSION: valhalla.VALHALLA_VERSION };