#include <boost/make_shared.hpp>
#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/shared_ptr.hpp>
#include <functional>
#include <iostream>
#include <napi.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <sstream>
#include <string>

#include "baldr/rapidjson_utils.h"
#include "midgard/logging.h"
#include "midgard/util.h"
#include "tyr/actor.h"
#include "worker.h"

boost::property_tree::ptree json_to_pt(const char* json) {
  std::stringstream ss;
  ss << json;
  boost::property_tree::ptree pt;
  rapidjson::read_json(ss, pt);
  return pt;
}

boost::property_tree::ptree make_conf(const char* config) {
  return json_to_pt(config);
}

class ActorWorker : public Napi::AsyncWorker {
public:
  ActorWorker(Napi::Function& callback,
              const std::string request,
              valhalla::tyr::actor_t& actor,
              const std::function<std::string(valhalla::tyr::actor_t& actor,
                                              const std::string& request)>& func)
      : Napi::AsyncWorker(callback), request(request), actor(actor), func(func) {
  }
  ~ActorWorker() {
  }

  void Execute() {
    try {
      response = func(actor, request);
    } catch (const valhalla::valhalla_exception_t& e) {
      actor.cleanup();
      rapidjson::StringBuffer err_message;
      rapidjson::Writer<rapidjson::StringBuffer> writer(err_message);

      writer.StartObject();
      writer.Key("error_code");
      writer.Uint(e.code);
      writer.Key("http_code");
      writer.Uint(e.http_code);
      writer.Key("message");
      writer.String(e.message);
      writer.EndObject();
      throw std::runtime_error(err_message.GetString());
    } catch (const std::exception& e) {
      actor.cleanup();
      throw std::runtime_error(e.what());
    }
  }

  void OnOK() {
    Napi::HandleScope scope(Env());
    Callback().Call({Env().Undefined(), Napi::String::New(Env(), response)});
  }

  void OnError(const Napi::Error& e) {
    Napi::Env env = Env();
    Callback().MakeCallback(Receiver().Value(), {e.Value(), env.Undefined()});
  }

  valhalla::tyr::actor_t actor;
  std::function<std::string(valhalla::tyr::actor_t& actor, std::string& request)> func;

private:
  std::string request;
  std::string response;
};

class Actor : public Napi::ObjectWrap<Actor> {
public:
  static Napi::Function Init(const Napi::CallbackInfo& info) {
    Napi::Env my_env = info.Env();
    Napi::HandleScope scope(my_env);

    Napi::Function func =
        DefineClass(my_env, "Actor",
                    {InstanceMethod("route", &Actor::Route), InstanceMethod("locate", &Actor::Locate),
                     InstanceMethod("matrix", &Actor::Matrix),
                     InstanceMethod("optimizedRoute", &Actor::OptimizedRoute),
                     InstanceMethod("isochrone", &Actor::Isochrone),
                     InstanceMethod("traceRoute", &Actor::TraceRoute),
                     InstanceMethod("traceAttributes", &Actor::TraceAttributes),
                     InstanceMethod("height", &Actor::Height),
                     InstanceMethod("transitAvailable", &Actor::TransitAvailable)});

    int length = info.Length();

    if (length <= 0 || !info[0].IsString()) {
      Napi::TypeError::New(my_env, "config string expected");
    }

    static boost::optional<boost::property_tree::ptree> pt;

    try {
      // configure logging
      pt = make_conf(std::string(info[0].As<Napi::String>()).c_str());
      boost::optional<boost::property_tree::ptree&> logging_subtree =
          pt->get_child_optional("tyr.logging");
      if (logging_subtree) {
        auto logging_config = valhalla::midgard::ToMap<const boost::property_tree::ptree&,
                                                       std::unordered_map<std::string, std::string>>(
            logging_subtree.get());
        valhalla::midgard::logging::Configure(logging_config);
      }
    } catch (...) { throw Napi::Error::New(my_env, "Failed to load logging config"); }

    constructor = Napi::Persistent(func);
    constructor.SuppressDestruct();
    return func;
  };

  Actor(const Napi::CallbackInfo& info)
      : actor(get_conf_from_info(info), true), Napi::ObjectWrap<Actor>(info) {
    Napi::Env env = info.Env();
    Napi::HandleScope scope(env);

    int length = info.Length();

    if (length <= 0 || !info[0].IsString()) {
      Napi::TypeError::New(env, "config string expected").ThrowAsJavaScriptException();
    }
  }

private:
  static Napi::FunctionReference constructor;

  static boost::property_tree::ptree get_conf_from_info(const Napi::CallbackInfo& info) {
    static boost::property_tree::ptree pt;

    try {
      pt = make_conf(std::string(info[0].As<Napi::String>()).c_str());
    } catch (...) { throw Napi::Error::New(info.Env(), "Unable to parse config"); }

    return pt;
  }

  Napi::Value generic_action(const Napi::CallbackInfo& info,
                             const std::function<std::string(valhalla::tyr::actor_t& actor,
                                                             const std::string& req)>& actor_func) {
    if (info.Length() <= 0 || !info[0].IsString() || !info[1].IsFunction()) {
      throw Napi::Error::New(info.Env(), "method must be called with string and callback");
    }
    const std::string req = std::string(info[0].As<Napi::String>());
    Napi::Function callback = info[1].As<Napi::Function>();

    ActorWorker* actorWorker = new ActorWorker(callback, req, actor, actor_func);
    actorWorker->Queue();
    return info.Env().Undefined();
  }

  Napi::Value Route(const Napi::CallbackInfo& info) {
    return generic_action(info,
                          [](valhalla::tyr::actor_t& actor, const std::string& request)
                              -> std::string { return actor.route(request); });
  }

  Napi::Value Locate(const Napi::CallbackInfo& info) {
    return generic_action(info,
                          [](valhalla::tyr::actor_t& actor, const std::string& request)
                              -> std::string { return actor.locate(request); });
  }

  Napi::Value Matrix(const Napi::CallbackInfo& info) {
    return generic_action(info,
                          [](valhalla::tyr::actor_t& actor, const std::string& request)
                              -> std::string { return actor.matrix(request); });
  }

  Napi::Value OptimizedRoute(const Napi::CallbackInfo& info) {
    return generic_action(info,
                          [](valhalla::tyr::actor_t& actor, const std::string& request)
                              -> std::string { return actor.optimized_route(request); });
  }

  Napi::Value Isochrone(const Napi::CallbackInfo& info) {
    return generic_action(info,
                          [](valhalla::tyr::actor_t& actor, const std::string& request)
                              -> std::string { return actor.isochrone(request); });
  }

  Napi::Value TraceRoute(const Napi::CallbackInfo& info) {
    return generic_action(info,
                          [](valhalla::tyr::actor_t& actor, const std::string& request)
                              -> std::string { return actor.trace_route(request); });
  }

  Napi::Value TraceAttributes(const Napi::CallbackInfo& info) {
    return generic_action(info,
                          [](valhalla::tyr::actor_t& actor, const std::string& request)
                              -> std::string { return actor.trace_attributes(request); });
  }

  Napi::Value Height(const Napi::CallbackInfo& info) {
    return generic_action(info,
                          [](valhalla::tyr::actor_t& actor, const std::string& request)
                              -> std::string { return actor.height(request); });
  }

  Napi::Value TransitAvailable(const Napi::CallbackInfo& info) {
    return generic_action(info,
                          [](valhalla::tyr::actor_t& actor, const std::string& request)
                              -> std::string { return actor.transit_available(request); });
  }

  Napi::Value Expansion(const Napi::CallbackInfo& info) {
    return generic_action(info,
                          [](valhalla::tyr::actor_t& actor, const std::string& request)
                              -> std::string { return actor.expansion(request); });
  }

  valhalla::tyr::actor_t actor;
};

Napi::FunctionReference Actor::constructor;

Napi::Object InitAll(Napi::Env env, Napi::Object exports) {
  return Napi::Function::New(env, Actor::Init, "init");
};

NODE_API_MODULE(node, InitAll)
