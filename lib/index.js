var valhalla = require('./binding/node_valhalla.node');
const genericPool = require("generic-pool");

module.exports = function(configStr, minWorkers, maxWorkers) {
    var Valhalla = valhalla(configStr);

    function createWorker() {
        return new Promise(function(resolve, reject) {
            var valh;
            try {
                valh = new Valhalla(configStr);
                resolve(valh);
            } catch (e) {
                reject(e);
            }
        });
    }

    const actorFactory = {
        create: createWorker,
        destroy: function(actor) {
            return new Promise(function(resolve, reject) {
                try {
                    delete actor;
                    resolve();
                } catch (e) {
                    reject(e);
                }
            });
        }
    };

    const opts = {
        max: maxWorkers || 1,
        min: minWorkers || 1
    };
    
    return function Actor(configStr) {
        try {
            new Valhalla(configStr);
        } catch (e) {
            throw e;
        }

        const actorPool = genericPool.createPool(actorFactory, opts);
        function actorMethod(methodName, request, cb) {
            if (!request || !cb) throw new Error('method must be called with string and callback');

            actorPool.acquire().then(function(actor) {
                actorPool.on('factoryCreateError', function(err) {
                    return cb(err);
                });

                actor[methodName](request, function(err, result) {
                    actorPool.release(actor);
                    return cb(err, result);
                });
            });
        }

        this.route = function(request, cb) {
            return actorMethod('route', request, cb);
        }

        this.locate = function(request, cb) {
            return actorMethod('locate', request, cb);
        }

        this.height = function(request, cb) {
            return actorMethod('height', request, cb);
        }

        this.isochrone = function(request, cb) {
            return actorMethod('isochrone', request, cb);
        }

        this.matrix = function(request, cb) {
            return actorMethod('matrix', request, cb);
        }

        this.optimizedRoute = function(request, cb) {
            return actorMethod('optimizedRoute', request, cb);
        }

        this.traceAttributes = function(request, cb) {
            return actorMethod('traceAttributes', request, cb);
        }

        this.traceRoute = function(request, cb) {
            return actorMethod('traceRoute', request, cb);
        }

        this.transitAvailable = function(request, cb) {
            return actorMethod('transitAvailable', request, cb);
        }
    };
}

