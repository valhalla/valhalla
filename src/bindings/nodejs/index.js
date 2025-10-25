const { getBinaryDir } = require('./lib/binary-path');
const path = require('path');
const fs = require('fs');

function getBinaryPath(baseDir) {
    return path.join(getBinaryDir(baseDir), 'valhalla_node.node');
}

const valhalla = require(getBinaryPath(__dirname));
const defaultConfig = require('./config.json');

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

    route(query) {
        if (typeof query === 'string') {
            return this.actor.route(query);
        }
        return JSON.parse(this.actor.route(JSON.stringify(query)));
    }

    locate(query) {
        if (typeof query === 'string') {
            return this.actor.locate(query);
        }
        return JSON.parse(this.actor.locate(JSON.stringify(query)));
    }

    matrix(query) {
        if (typeof query === 'string') {
            return this.actor.matrix(query);
        }
        return JSON.parse(this.actor.matrix(JSON.stringify(query)));
    }

    optimizedRoute(query) {
        if (typeof query === 'string') {
            return this.actor.optimizedRoute(query);
        }
        return JSON.parse(this.actor.optimizedRoute(JSON.stringify(query)));
    }
    
    isochrone(query) {
        if (typeof query === 'string') {
            return this.actor.isochrone(query);
        }
        return JSON.parse(this.actor.isochrone(JSON.stringify(query)));
    }

    traceRoute(query) {
        if (typeof query === 'string') {
            return this.actor.traceRoute(query);
        }
        return JSON.parse(this.actor.traceRoute(JSON.stringify(query)));
    }
    
    traceAttributes(query) {
        if (typeof query === 'string') {
            return this.actor.traceAttributes(query);
        }
        return JSON.parse(this.actor.traceAttributes(JSON.stringify(query)));
    }

    height(query) {
        if (typeof query === 'string') {
            return this.actor.height(query);
        }
        return JSON.parse(this.actor.height(JSON.stringify(query)));
    }

    transitAvailable(query) {
        if (typeof query === 'string') {
            return this.actor.transitAvailable(query);
        }
        return JSON.parse(this.actor.transitAvailable(JSON.stringify(query)));
    }

    expansion(query) {
        if (typeof query === 'string') {
            return this.actor.expansion(query);
        }
        return JSON.parse(this.actor.expansion(JSON.stringify(query)));
    }

    centroid(query) {
        if (typeof query === 'string') {
            return this.actor.centroid(query);
        }
        return JSON.parse(this.actor.centroid(JSON.stringify(query)));
    }
    
    
    status(query) {
        if (typeof query === 'string') {
            return this.actor.status(query);
        }
        return JSON.parse(this.actor.status(JSON.stringify(query)));
    } 
}


function getConfig(options = {}) {
    const {
        tileExtract = 'valhalla_tiles.tar',
        tileDir = 'valhalla_tiles',
        verbose = false
    } = options;

    // Deep clone the config 
    const config = JSON.parse(JSON.stringify(defaultConfig));

    if (tileDir) {
        try {
            config.mjolnir.tile_dir = path.resolve(tileDir);
        } catch (error) {
            config.mjolnir.tile_dir = tileDir;
        }
    } else {
        config.mjolnir.tile_dir = '';
    }

    if (tileExtract) {
        try {
            config.mjolnir.tile_extract = path.resolve(tileExtract);
        } catch (error) {
            config.mjolnir.tile_extract = tileExtract;
        }
    } else {
        config.mjolnir.tile_extract = '';
    }

    config.mjolnir.logging.type = verbose ? 'std_out' : '';

    return config;
}

module.exports = { Actor, getConfig, VALHALLA_VERSION: valhalla.VALHALLA_VERSION };