#include "mjolnir/valhalla_traffic_utils.h"
#include "valhalla/midgard/point2.h"

#include <napi.h>

#include <iostream>
#include <string>
#include <vector>

Napi::Value HandleTileTraffic(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  // Check argument count
  if (info.Length() < 4) {
    Napi::TypeError::New(env, "Expected 3 arguments: tile_offset, traffic_params, last_updated")
        .ThrowAsJavaScriptException();
    return env.Null();
  }

  // Extract tile_offset (should be a number)
  if (!info[0].IsNumber()) {
    Napi::TypeError::New(env, "tile_offset must be a number").ThrowAsJavaScriptException();
    return env.Null();
  }
  uint64_t tile_offset = static_cast<uint64_t>(info[0].As<Napi::Number>().Int64Value());

  // Extract traffic_params (should be an array of numbers)
  if (!info[1].IsArray()) {
    Napi::TypeError::New(env, "traffic_params must be an array").ThrowAsJavaScriptException();
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
    Napi::TypeError::New(env, "last_updated must be a number").ThrowAsJavaScriptException();
    return env.Null();
  }
  uint64_t last_updated = info[2].As<Napi::Number>().Uint32Value();

  if (!info[3].IsString()) {
    Napi::TypeError::New(env, "traffic_path must be a string").ThrowAsJavaScriptException();
    return env.Null();
  }
  std::string traffic_path = info[3].As<Napi::String>().Utf8Value();

  try {

    // Call the Valhalla traffic utility function
    update_traffic_tile(tile_offset, traffic_params, last_updated, traffic_path);
    // update_traffic_tile(tile_offset, traffic_params, last_updated, traffic_path);

    // Return success indicator
    return Napi::String::New(env, "Traffic tile updated successfully");

  } catch (const std::exception& e) {
    Napi::Error::New(env, std::string("Error updating traffic tile: ") + e.what())
        .ThrowAsJavaScriptException();
    return env.Null();
  }
}

Napi::Object Init(Napi::Env env, Napi::Object exports) {
  exports.Set("handleTileTraffic", Napi::Function::New(env, HandleTileTraffic));
  return exports;
}

NODE_API_MODULE(bindings, Init)
