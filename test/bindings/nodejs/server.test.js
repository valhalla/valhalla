const test = require('tape');
var config = require('./fixtures/valhalla');
var Valhalla = require('../../../')(JSON.stringify(config));
var path = require( "path" );
console.log( require.resolve('../../../') );

var valhalla = new Valhalla(JSON.stringify(config));

var http = require('http');
var currentPath = process.cwd();
console.log(currentPath)
//create a server object:
http.createServer(function (req, res) {

    res.write('Hello World!'); //write a response to the client
    var locateRequest = '{"verbose":true,"locations":[{"lat":35.546115,"lon":136.385076,"type":"break"}, {"lat":35.544232,"lon":136.385752,"type":"break"}],"costing":"bicycle","costing_options":{"bicycle":{"bicycle_type":"road"}},"directions_options":{"units":"miles"},"id":"12abc3afe23984fe"}';
    // var locateRequest = req.toString()
    valhalla.locate(locateRequest, (err, result) => {
        var locate = JSON.parse(result);
        console.log(result)

    });
    res.end(); //end the response
}).listen(8080); //the server object listens on port 8080
