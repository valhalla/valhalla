const valhalla = require('./binding/node_valhalla.node');
const genericPool = require('generic-pool');

module.exports = function(configStr, options = {}) {
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

    function deleteWorker(actor) {
        return new Promise(function(resolve, reject) {
            try {
                delete actor;
                resolve();
            } catch (e) {
                reject(e);
            }
        });
    }

    const actorFactory = {
        create: createWorker,
        destroy: deleteWorker 
    };

    const opts = {
        max: options.maxWorkers || 1,
        min: options.minWorkers || 1
    };
    
    return function Actor(configStr) {
        // if we can't create a new actor, throw early rather than in the actor pool
        new Valhalla(configStr);

        const actorPool = genericPool.createPool(actorFactory, opts);

        function actorMethodFactory(methodName) {
            return function(request, cb) {
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
        }

        this.route = actorMethodFactory('route'); 

        this.locate = actorMethodFactory('locate');

        this.height = actorMethodFactory('height');

        this.isochrone = actorMethodFactory('isochrone');

        this.matrix = actorMethodFactory('matrix');

        this.optimizedRoute = actorMethodFactory('optimizedRoute');

        this.traceAttributes = actorMethodFactory('traceAttributes');

        this.traceRoute = actorMethodFactory('traceRoute');

        this.transitAvailable = actorMethodFactory('transitAvailable');
    };
}

