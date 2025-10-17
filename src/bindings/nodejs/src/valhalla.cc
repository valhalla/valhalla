#include <napi.h>
#include <string>
#include <sstream>
#include <boost/property_tree/ptree.hpp>
#include "baldr/rapidjson_utils.h"
#include "config.h"
#include "midgard/logging.h"
#include "midgard/util.h"
#include "tyr/actor.h"

namespace vt = valhalla::tyr;

namespace {

// Configure Valhalla from a JSON config string or file path
const boost::property_tree::ptree configure(const std::string& config) {
  boost::property_tree::ptree pt;
  try {
    // Try to parse as a file first, then as a JSON string
    std::stringstream ss(config);
    rapidjson::read_json(ss, pt);

    auto logging_subtree = pt.get_child_optional("mjolnir.logging");
    if (logging_subtree) {
      auto logging_config = valhalla::midgard::ToMap<const boost::property_tree::ptree&,
                                                     std::unordered_map<std::string, std::string>>(
          logging_subtree.get());
      valhalla::midgard::logging::Configure(logging_config);
    }
  } catch (...) {
    throw std::runtime_error("Failed to load config from: " + config);
  }

  return pt;
}

} // namespace

// Actor class wrapper for Node.js
class Actor : public Napi::ObjectWrap<Actor> {
public:
  static Napi::Object Init(Napi::Env env, Napi::Object exports) {
    Napi::Function func = DefineClass(env, "Actor", {
      InstanceMethod("route", &Actor::Route),
      InstanceMethod("locate", &Actor::Locate),
      InstanceMethod("matrix", &Actor::Matrix),
      InstanceMethod("optimizedRoute", &Actor::OptimizedRoute),
      InstanceMethod("isochrone", &Actor::Isochrone),
      InstanceMethod("traceRoute", &Actor::TraceRoute),
      InstanceMethod("traceAttributes", &Actor::TraceAttributes),
      InstanceMethod("height", &Actor::Height),
      InstanceMethod("transitAvailable", &Actor::TransitAvailable),
      InstanceMethod("expansion", &Actor::Expansion),
      InstanceMethod("centroid", &Actor::Centroid),
      InstanceMethod("status", &Actor::Status)
    });

    Napi::FunctionReference* constructor = new Napi::FunctionReference();
    *constructor = Napi::Persistent(func);
    env.SetInstanceData(constructor);

    exports.Set("Actor", func);
    return exports;
  }

  Actor(const Napi::CallbackInfo& info) : Napi::ObjectWrap<Actor>(info) {
    Napi::Env env = info.Env();

    if (info.Length() < 1 || !info[0].IsString()) {
      Napi::TypeError::New(env, "String config expected").ThrowAsJavaScriptException();
      return;
    }

    std::string config = info[0].As<Napi::String>().Utf8Value();

    try {
      actor_ = std::make_unique<vt::actor_t>(configure(config), true);
    } catch (const std::exception& e) {
      Napi::Error::New(env, std::string("Failed to create actor: ") + e.what())
          .ThrowAsJavaScriptException();
    }
  }

private:
  std::unique_ptr<vt::actor_t> actor_;

  // Route method
  Napi::Value Route(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();

    if (info.Length() < 1 || !info[0].IsString()) {
      Napi::TypeError::New(env, "String request expected").ThrowAsJavaScriptException();
      return env.Null();
    }

    std::string request = info[0].As<Napi::String>().Utf8Value();

    try {
      std::string result = actor_->route(request);
      return Napi::String::New(env, result);
    } catch (const std::exception& e) {
      Napi::Error::New(env, std::string("Route error: ") + e.what())
          .ThrowAsJavaScriptException();
      return env.Null();
    }
  }

  // Locate method
  Napi::Value Locate(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();

    if (info.Length() < 1 || !info[0].IsString()) {
      Napi::TypeError::New(env, "String request expected").ThrowAsJavaScriptException();
      return env.Null();
    }

    std::string request = info[0].As<Napi::String>().Utf8Value();

    try {
      std::string result = actor_->locate(request);
      return Napi::String::New(env, result);
    } catch (const std::exception& e) {
      Napi::Error::New(env, std::string("Locate error: ") + e.what())
          .ThrowAsJavaScriptException();
      return env.Null();
    }
  }

  // Matrix method
  Napi::Value Matrix(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();

    if (info.Length() < 1 || !info[0].IsString()) {
      Napi::TypeError::New(env, "String request expected").ThrowAsJavaScriptException();
      return env.Null();
    }

    std::string request = info[0].As<Napi::String>().Utf8Value();

    try {
      std::string result = actor_->matrix(request);
      return Napi::String::New(env, result);
    } catch (const std::exception& e) {
      Napi::Error::New(env, std::string("Matrix error: ") + e.what())
          .ThrowAsJavaScriptException();
      return env.Null();
    }
  }

  // OptimizedRoute method
  Napi::Value OptimizedRoute(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();

    if (info.Length() < 1 || !info[0].IsString()) {
      Napi::TypeError::New(env, "String request expected").ThrowAsJavaScriptException();
      return env.Null();
    }

    std::string request = info[0].As<Napi::String>().Utf8Value();

    try {
      std::string result = actor_->optimized_route(request);
      return Napi::String::New(env, result);
    } catch (const std::exception& e) {
      Napi::Error::New(env, std::string("OptimizedRoute error: ") + e.what())
          .ThrowAsJavaScriptException();
      return env.Null();
    }
  }

  // Isochrone method
  Napi::Value Isochrone(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();

    if (info.Length() < 1 || !info[0].IsString()) {
      Napi::TypeError::New(env, "String request expected").ThrowAsJavaScriptException();
      return env.Null();
    }

    std::string request = info[0].As<Napi::String>().Utf8Value();

    try {
      std::string result = actor_->isochrone(request);
      return Napi::String::New(env, result);
    } catch (const std::exception& e) {
      Napi::Error::New(env, std::string("Isochrone error: ") + e.what())
          .ThrowAsJavaScriptException();
      return env.Null();
    }
  }

  // TraceRoute method
  Napi::Value TraceRoute(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();

    if (info.Length() < 1 || !info[0].IsString()) {
      Napi::TypeError::New(env, "String request expected").ThrowAsJavaScriptException();
      return env.Null();
    }

    std::string request = info[0].As<Napi::String>().Utf8Value();

    try {
      std::string result = actor_->trace_route(request);
      return Napi::String::New(env, result);
    } catch (const std::exception& e) {
      Napi::Error::New(env, std::string("TraceRoute error: ") + e.what())
          .ThrowAsJavaScriptException();
      return env.Null();
    }
  }

  // TraceAttributes method
  Napi::Value TraceAttributes(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();

    if (info.Length() < 1 || !info[0].IsString()) {
      Napi::TypeError::New(env, "String request expected").ThrowAsJavaScriptException();
      return env.Null();
    }

    std::string request = info[0].As<Napi::String>().Utf8Value();

    try {
      std::string result = actor_->trace_attributes(request);
      return Napi::String::New(env, result);
    } catch (const std::exception& e) {
      Napi::Error::New(env, std::string("TraceAttributes error: ") + e.what())
          .ThrowAsJavaScriptException();
      return env.Null();
    }
  }

  // Height method
  Napi::Value Height(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();

    if (info.Length() < 1 || !info[0].IsString()) {
      Napi::TypeError::New(env, "String request expected").ThrowAsJavaScriptException();
      return env.Null();
    }

    std::string request = info[0].As<Napi::String>().Utf8Value();

    try {
      std::string result = actor_->height(request);
      return Napi::String::New(env, result);
    } catch (const std::exception& e) {
      Napi::Error::New(env, std::string("Height error: ") + e.what())
          .ThrowAsJavaScriptException();
      return env.Null();
    }
  }

  // TransitAvailable method
  Napi::Value TransitAvailable(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();

    if (info.Length() < 1 || !info[0].IsString()) {
      Napi::TypeError::New(env, "String request expected").ThrowAsJavaScriptException();
      return env.Null();
    }

    std::string request = info[0].As<Napi::String>().Utf8Value();

    try {
      std::string result = actor_->transit_available(request);
      return Napi::String::New(env, result);
    } catch (const std::exception& e) {
      Napi::Error::New(env, std::string("TransitAvailable error: ") + e.what())
          .ThrowAsJavaScriptException();
      return env.Null();
    }
  }

  // Expansion method
  Napi::Value Expansion(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();

    if (info.Length() < 1 || !info[0].IsString()) {
      Napi::TypeError::New(env, "String request expected").ThrowAsJavaScriptException();
      return env.Null();
    }

    std::string request = info[0].As<Napi::String>().Utf8Value();

    try {
      std::string result = actor_->expansion(request);
      return Napi::String::New(env, result);
    } catch (const std::exception& e) {
      Napi::Error::New(env, std::string("Expansion error: ") + e.what())
          .ThrowAsJavaScriptException();
      return env.Null();
    }
  }

  // Centroid method
  Napi::Value Centroid(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();

    if (info.Length() < 1 || !info[0].IsString()) {
      Napi::TypeError::New(env, "String request expected").ThrowAsJavaScriptException();
      return env.Null();
    }

    std::string request = info[0].As<Napi::String>().Utf8Value();

    try {
      std::string result = actor_->centroid(request);
      return Napi::String::New(env, result);
    } catch (const std::exception& e) {
      Napi::Error::New(env, std::string("Centroid error: ") + e.what())
          .ThrowAsJavaScriptException();
      return env.Null();
    }
  }

  // Status method
  Napi::Value Status(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();

    if (info.Length() < 1 || !info[0].IsString()) {
      Napi::TypeError::New(env, "String request expected").ThrowAsJavaScriptException();
      return env.Null();
    }

    std::string request = info[0].As<Napi::String>().Utf8Value();

    try {
      std::string result = actor_->status(request);
      return Napi::String::New(env, result);
    } catch (const std::exception& e) {
      Napi::Error::New(env, std::string("Status error: ") + e.what())
          .ThrowAsJavaScriptException();
      return env.Null();
    }
  }
};

// Module initialization
Napi::Object Init(Napi::Env env, Napi::Object exports) {
  // Add version info
  exports.Set(Napi::String::New(env, "VALHALLA_VERSION"),
              Napi::String::New(env, VALHALLA_PRINT_VERSION));

  Actor::Init(env, exports);
  return exports;
}

NODE_API_MODULE(valhalla, Init)
