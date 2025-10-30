#include "baldr/rapidjson_utils.h"
#include "config.h"
#include "midgard/logging.h"
#include "midgard/util.h"
#include "tyr/actor.h"

#include <boost/property_tree/ptree.hpp>
#include <napi.h>

#include <functional>
#include <sstream>
#include <string>
#include <unordered_map>

namespace vt = valhalla::tyr;

namespace {

// These bindings may be used from NodeJS's thread pool, so we need to guarantee that each actor is
// used exclusively in its own thread. Since user is free to create as many actors as they want, we
// use pointer to config object to distinguish between concrete actors
vt::actor_t* GetThreadLocalActor(const boost::property_tree::ptree* config) {
  using ActorMap = std::unordered_map<const void*, std::shared_ptr<vt::actor_t>>;
  static thread_local ActorMap actors;

  auto it = actors.find(config);
  if (it == actors.end()) {
    auto actor = std::make_shared<vt::actor_t>(*config, true);
    actors[config] = actor;
    return actor.get();
  }
  return it->second.get();
}

const boost::property_tree::ptree configure(const std::string& config) {
  boost::property_tree::ptree pt;
  try {
    std::stringstream stream(config);
    rapidjson::read_json(stream, pt);

    auto logging_subtree = pt.get_child_optional("mjolnir.logging");
    if (logging_subtree) {
      auto logging_config = valhalla::midgard::ToMap<const boost::property_tree::ptree&,
                                                     std::unordered_map<std::string, std::string>>(
          logging_subtree.get());
      valhalla::midgard::logging::Configure(logging_config);
    }
  } catch (...) { throw std::runtime_error("Failed to load config"); }

  return pt;
}

} // namespace

class ValhallaAsyncWorker : public Napi::AsyncWorker {
public:
  using ActorMethodFunction = std::function<std::string(vt::actor_t*, const std::string&)>;

  ValhallaAsyncWorker(const Napi::Env& env,
                      const boost::property_tree::ptree* config,
                      ActorMethodFunction method,
                      const std::string& request,
                      const std::string& method_name)
      : Napi::AsyncWorker(env), deferred_(Napi::Promise::Deferred::New(env)), config_(config),
        method_(method), request_(request), method_name_(method_name) {
  }

  Napi::Promise GetPromise() {
    return deferred_.Promise();
  }

protected:
  void Execute() override {
    try {
      auto* actor = GetThreadLocalActor(config_);
      result_ = method_(actor, request_);
    } catch (const std::exception& e) {
      SetError(std::string(method_name_) + " error: " + e.what());
    } catch (...) { SetError(std::string(method_name_) + " error: unknown exception"); }
  }

  void OnOK() override {
    deferred_.Resolve(Napi::String::New(Env(), result_));
  }

  void OnError(const Napi::Error& e) override {
    deferred_.Reject(e.Value());
  }

private:
  Napi::Promise::Deferred deferred_;
  const boost::property_tree::ptree* config_;
  ActorMethodFunction method_;
  std::string request_;
  std::string method_name_;
  std::string result_;
};

class Actor : public Napi::ObjectWrap<Actor> {
public:
  static Napi::Object Init(Napi::Env env, Napi::Object exports) {
    Napi::Function func =
        DefineClass(env, "Actor",
                    {InstanceMethod("route", &Actor::Route), InstanceMethod("locate", &Actor::Locate),
                     InstanceMethod("matrix", &Actor::Matrix),
                     InstanceMethod("optimizedRoute", &Actor::OptimizedRoute),
                     InstanceMethod("isochrone", &Actor::Isochrone),
                     InstanceMethod("traceRoute", &Actor::TraceRoute),
                     InstanceMethod("traceAttributes", &Actor::TraceAttributes),
                     InstanceMethod("height", &Actor::Height),
                     InstanceMethod("transitAvailable", &Actor::TransitAvailable),
                     InstanceMethod("expansion", &Actor::Expansion),
                     InstanceMethod("centroid", &Actor::Centroid),
                     InstanceMethod("status", &Actor::Status)});

    // we don't need to delete it, it will be handled by Node
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
      config_ = configure(config);
    } catch (const std::exception& e) {
      Napi::Error::New(env, std::string("Failed to parse config: ") + e.what())
          .ThrowAsJavaScriptException();
    } catch (...) {
      Napi::Error::New(env, "Failed to parse config: unknown exception").ThrowAsJavaScriptException();
    }
  }

private:
  boost::property_tree::ptree config_;

  Napi::Value CreateAsyncRequest(const Napi::CallbackInfo& info,
                                 ValhallaAsyncWorker::ActorMethodFunction method,
                                 const std::string& method_name) {
    Napi::Env env = info.Env();

    if (info.Length() < 1 || !info[0].IsString()) {
      Napi::TypeError::New(env, "String request expected").ThrowAsJavaScriptException();
      return env.Null();
    }

    std::string request = info[0].As<Napi::String>().Utf8Value();

    // we don't need to delete it, it will be handled by Node
    auto* worker = new ValhallaAsyncWorker(env, &config_, method, request, method_name);
    worker->Queue();
    return worker->GetPromise();
  }

  Napi::Value Route(const Napi::CallbackInfo& info) {
    return CreateAsyncRequest(
        info, [](vt::actor_t* actor, const std::string& request) { return actor->route(request); },
        "Route");
  }

  Napi::Value Locate(const Napi::CallbackInfo& info) {
    return CreateAsyncRequest(
        info, [](vt::actor_t* actor, const std::string& request) { return actor->locate(request); },
        "Locate");
  }

  Napi::Value Matrix(const Napi::CallbackInfo& info) {
    return CreateAsyncRequest(
        info, [](vt::actor_t* actor, const std::string& request) { return actor->matrix(request); },
        "Matrix");
  }

  Napi::Value OptimizedRoute(const Napi::CallbackInfo& info) {
    return CreateAsyncRequest(
        info,
        [](vt::actor_t* actor, const std::string& request) {
          return actor->optimized_route(request);
        },
        "OptimizedRoute");
  }

  Napi::Value Isochrone(const Napi::CallbackInfo& info) {
    return CreateAsyncRequest(
        info,
        [](vt::actor_t* actor, const std::string& request) { return actor->isochrone(request); },
        "Isochrone");
  }

  Napi::Value TraceRoute(const Napi::CallbackInfo& info) {
    return CreateAsyncRequest(
        info,
        [](vt::actor_t* actor, const std::string& request) { return actor->trace_route(request); },
        "TraceRoute");
  }

  Napi::Value TraceAttributes(const Napi::CallbackInfo& info) {
    return CreateAsyncRequest(
        info,
        [](vt::actor_t* actor, const std::string& request) {
          return actor->trace_attributes(request);
        },
        "TraceAttributes");
  }

  Napi::Value Height(const Napi::CallbackInfo& info) {
    return CreateAsyncRequest(
        info, [](vt::actor_t* actor, const std::string& request) { return actor->height(request); },
        "Height");
  }

  Napi::Value TransitAvailable(const Napi::CallbackInfo& info) {
    return CreateAsyncRequest(
        info,
        [](vt::actor_t* actor, const std::string& request) {
          return actor->transit_available(request);
        },
        "TransitAvailable");
  }

  Napi::Value Expansion(const Napi::CallbackInfo& info) {
    return CreateAsyncRequest(
        info,
        [](vt::actor_t* actor, const std::string& request) { return actor->expansion(request); },
        "Expansion");
  }

  Napi::Value Centroid(const Napi::CallbackInfo& info) {
    return CreateAsyncRequest(
        info, [](vt::actor_t* actor, const std::string& request) { return actor->centroid(request); },
        "Centroid");
  }

  Napi::Value Status(const Napi::CallbackInfo& info) {
    return CreateAsyncRequest(
        info, [](vt::actor_t* actor, const std::string& request) { return actor->status(request); },
        "Status");
  }
};

Napi::Object Init(Napi::Env env, Napi::Object exports) {
  exports.Set(Napi::String::New(env, "VALHALLA_VERSION"),
              Napi::String::New(env, VALHALLA_PRINT_VERSION));

  Actor::Init(env, exports);
  return exports;
}

NODE_API_MODULE(valhalla, Init)
