# Support for Protocol Buffers as Request/Response Formats (BETA)

Valhalla allows users to interact with its service (and bindings/library) directly using pbf objects/bytes in addition to the conventional JSON request/response formats. This feature is currently in beta meaning that we are still actively developing it and there are likely still some bugs to work out before it's ready for widespread use. Most importantly, we reserve the right to make organizational changes to the protobuf schema while the label beta is still in effect. 

Valhalla keeps track of the state of a "request" as it is being processed through our various APIs using protobuf messages. In fact, when you call into our top level APIs like `route` `locate` `isochrones` etc, we parse your json directly into a protobuf object. We then pass that object around to different parts of the library adding to it as we fulfill the request. We extended this functionality such that, instead of passing json to the service/bindings/library you can pass protobuf bytes or objects and get back protobuf bytes or objects.

## Motivations

There are a couple good reasons for this. As formats go, protobuf has many benefits over json:

* speed of serialization and deserialization
* over the wire size
* backward/forward compatibility
* working with an object on both sides of the network which eventually leads to
* gRPC

The major drawback is that protobuf is a binary format so if you have just the bytes they aren't much good to you.

## Request

To use protobuf as a request/input to the HTTP API you need to do two things:

* Send the proper HTTP header to signal a protobuf payload. The header should be: `Content-Type: application/x-protobuf`
* Send protocol buffer's serialized bytes as the body of the HTTP request

The message we use for the entire transaction is the `Api` message, whose definition you can find [here](../../proto/api.proto). All of the request parameters should be filled out via the `Options` message attached to the `Api` message. Most importantly, you will want to set your `format` to `pbf` and your `action` to the relevant API you are calling (though the HTTP request path also provides the latter). The `options` object also contains a subobject named `pbf_field_selector` which can be used to turn on/off the top level fields in the response. For example, if you only want the `directions` part of the protobuf response to be present (much smaller payload) then turn on only that flag in the field selector. The rest of the request options depend on which API you are calling. For more information about what and which options to set for a given API please read that APIs specific docs regarding its request options.

## Response

As with the request/input, the response/output will again be the `Api` message but will have more parts of it filled out. Depending on which API you are calling different parts of the response object will be filled out. Route-like responses will have `Trip` and `Directions` objects filled out whereas non-route APIs will have different parts of the message filled out. Not all APIs support protobuf output. Those that don't, will return JSON as they do today. Currently, the following APIs support protobuf as output: `route, trace_route, optimized_route, centroid, trace_attributes, status` 

## Future Work

There are a few more things we should do before we can remove the beta label from this feature:

* **Add Native PBF Support to Python Bindings**: We can support, in addition to JSON strings, the ability for python to work directly with protobuf objects (those generated with protoc) across the python/c++ barrier. This would be a very natural way for python users to interact with Valhalla.
* **Support for All APIs**: As mentioned above we only support a certain subset Valhalla's APIs, over time we can add the rest of the APIs to the `Api` message.
