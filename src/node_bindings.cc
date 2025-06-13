#include "valhalla/midgard/point2.h"
#include "mjolnir/valhalla_traffic_utils.h"

#include <napi.h>
#include <vector>
#include <string>

#include <iostream>


Napi::Number GetLat(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();
    valhalla::midgard::Point2 point(40.0f, -75.0f); // x = 40.0, y = -75.0
    return Napi::Number::New(env, point.y()); // y() returns second (aka lat)
}

Napi::String Hello(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();
    return Napi::String::New(env, "Hello from N-API!");
}

Napi::Value HandleTileTraffic(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();


    // Check argument count
    if (info.Length() < 3) {
        Napi::TypeError::New(env, "Expected 3 arguments: tile_offset, traffic_params, last_updated")
            .ThrowAsJavaScriptException();
        return env.Null();
    }

    // Extract tile_offset (should be a number)
    if (!info[0].IsNumber()) {
        Napi::TypeError::New(env, "tile_offset must be a number")
            .ThrowAsJavaScriptException();
        return env.Null();
    }
    uint64_t tile_offset =  static_cast<uint64_t>(info[0].As<Napi::Number>().Int64Value());

    // Extract traffic_params (should be an array of numbers)
    if (!info[1].IsArray()) {
        Napi::TypeError::New(env, "traffic_params must be an array")
            .ThrowAsJavaScriptException();
        return env.Null();
    }

    Napi::Array js_params = info[1].As<Napi::Array>();
    std::vector<uint64_t> traffic_params;

    for (uint32_t i = 0; i < js_params.Length(); i++) {
        Napi::Value element = js_params[i];
        if (!element.IsNumber()) {
            Napi::TypeError::New(env, "All traffic_params elements must be numbers")
                .ThrowAsJavaScriptException();
            return env.Null();
        }
        traffic_params.push_back(element.As<Napi::Number>().Uint32Value());
    }

    // Extract last_updated (should be a number)
    if (!info[2].IsNumber()) {
        Napi::TypeError::New(env, "last_updated must be a number")
            .ThrowAsJavaScriptException();
        return env.Null();
    }
    uint64_t last_updated = info[2].As<Napi::Number>().Uint32Value();


    try {

        // Call the Valhalla traffic utility function
        update_traffic_tile(tile_offset, traffic_params, last_updated);

        // Return success indicator
        return Napi::String::New(env, "Traffic tile updated successfully");

    } catch (const std::exception& e) {
        Napi::Error::New(env, std::string("Error updating traffic tile: ") + e.what())
            .ThrowAsJavaScriptException();
        return env.Null();
    }
}

Napi::Object Init(Napi::Env env, Napi::Object exports) {
    exports.Set("hello", Napi::Function::New(env, Hello));
    exports.Set("getLat", Napi::Function::New(env, GetLat));
    exports.Set("handleTileTraffic", Napi::Function::New(env, HandleTileTraffic));
    return exports;
}

NODE_API_MODULE(bindings, Init)
