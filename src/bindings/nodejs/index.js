const path = require('path');
const os = require('os');

// Detect platform and architecture
function getBinaryPath() {
    const platform = os.platform();
    const arch = os.arch();
    
    // Map Node.js platform names to our directory structure
    let platformDir;
    switch (platform) {
        case 'darwin':
            platformDir = 'darwin';
            break;
        case 'linux':
            platformDir = 'linux';
            break;
        case 'win32':
            platformDir = 'win32';
            break;
        default:
            throw new Error(`Unsupported platform: ${platform}`);
    }
    
    // Map Node.js arch names to our directory structure
    let archDir;
    switch (arch) {
        case 'x64':
            archDir = platformDir === 'linux' ? 'x86_64' : 'x64';
            break;
        case 'arm64':
            archDir = 'arm64';
            break;
        default:
            throw new Error(`Unsupported architecture: ${arch}`);
    }
    
    const binaryPath = path.join(__dirname, platformDir, archDir, 'valhalla_node.node');
    return binaryPath;
}

const valhalla = require(getBinaryPath());

class Actor {
    // TODO: handle both file path and string object config
    constructor(config) {
        this.actor = new valhalla.Actor(config);
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

module.exports = { Actor, VALHALLA_VERSION: valhalla.VALHALLA_VERSION };