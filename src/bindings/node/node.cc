#include <boost/make_shared.hpp>
#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/shared_ptr.hpp>
#include <functional>
#include <iostream>
#include <node_api.h>
#include <sstream>
#include <string>

#include "src/worker.cc"
#include "valhalla/exception.h"
#include "valhalla/tyr/actor.h"
#include "valhalla/midgard/logging.h"
#include "valhalla/midgard/util.h"

boost::property_tree::ptree json_to_pt(const char* json) {
  std::stringstream ss;
  ss << json;
  boost::property_tree::ptree pt;
  boost::property_tree::read_json(ss, pt);
  return pt;
}

boost::property_tree::ptree make_conf(const char* config) {
  return json_to_pt(config);
}

#define DECLARE_NAPI_METHOD(name, func)                                                              \
  { name, 0, func, 0, 0, 0, napi_default, 0 }

class Actor {
public:
  static napi_value Init(napi_env env, napi_callback_info info) {
    napi_status status;
    napi_property_descriptor properties[] = {DECLARE_NAPI_METHOD("route", Route),
                                             DECLARE_NAPI_METHOD("locate", Locate),
                                             DECLARE_NAPI_METHOD("matrix", Matrix),
                                             DECLARE_NAPI_METHOD("optimizedRoute", OptimizedRoute),
                                             DECLARE_NAPI_METHOD("isochrone", Isochrone),
                                             DECLARE_NAPI_METHOD("traceRoute", TraceRoute),
                                             DECLARE_NAPI_METHOD("traceAttributes", TraceAttributes),
                                             DECLARE_NAPI_METHOD("height", Height),
                                             DECLARE_NAPI_METHOD("transitAvailable",
                                                                 TransitAvailable)};
    // parse config file to get logging config
    size_t argc = 1;
    napi_value argv[1];

    status = napi_get_cb_info(env, info, &argc, argv, nullptr, nullptr);
    if (status != napi_ok) {
      napi_throw_error(env, NULL, "Failed to parse arguments");
      return NULL;
    }

    // parse config string passed in
    size_t config_string_size;
    status = napi_get_value_string_utf8(env, argv[0], nullptr, 0, &config_string_size);
    if (status != napi_ok) {
      napi_throw_error(env, NULL, "Failed to get config string length");
      return NULL;
    }

    if (config_string_size > 1024 * 1024) {
      napi_throw_error(env, NULL, "Too large JSON config");
      return NULL;
    }

    char config_string[++config_string_size];
    status = napi_get_value_string_utf8(env, argv[0], config_string, config_string_size,
                                        &config_string_size);
    if (status != napi_ok) {
      napi_throw_error(env, NULL, "Failed to get config string");
      return NULL;
    }

    static boost::optional<boost::property_tree::ptree> pt = make_conf(config_string);
    try {
      // configure logging
      boost::optional<boost::property_tree::ptree&> logging_subtree =
          pt->get_child_optional("tyr.logging");
      if (logging_subtree) {
        auto logging_config = valhalla::midgard::ToMap<const boost::property_tree::ptree&,
                                                       std::unordered_map<std::string, std::string>>(
            logging_subtree.get());
        valhalla::midgard::logging::Configure(logging_config);
      }
    } catch (...) {
      napi_throw_error(env, NULL, "Failed to load logging config");
      return NULL;
    }

    napi_value actor_constructor;
    status = napi_define_class(env, "Actor", NAPI_AUTO_LENGTH, New, nullptr, 9, properties,
                               &actor_constructor);
    if (status != napi_ok) {
      napi_throw_error(env, NULL, "Failed to define class");
      return NULL;
    }
    status = napi_create_reference(env, actor_constructor, 1, &constructor);
    if (status != napi_ok) {
      napi_throw_error(env, NULL, "Failed to create constructor reference");
      return NULL;
    }

    return actor_constructor;
  }
  static void Destructor(napi_env env, void* nativeObject, void* finalize_hint) {
    delete reinterpret_cast<Actor*>(nativeObject);
  }

private:
  explicit Actor(const char* config)
      : actor(make_conf(config), true), env_(nullptr), wrapper_(nullptr) {
  }
  ~Actor() {
    napi_delete_reference(env_, wrapper_);
  }

  static napi_value New(napi_env env, napi_callback_info info) {
    napi_status status;

    napi_value target;
    status = napi_get_new_target(env, info, &target);
    if (status != napi_ok) {
      napi_throw_error(env, NULL, "Failed to get 'new' target");
      return NULL;
    }
    bool is_constructor = target != nullptr;

    if (is_constructor) {
      // Invoked as constructor: `new Actor(...)`
      size_t argc = 1;
      napi_value jsthis;
      napi_value argv[1];

      status = napi_get_cb_info(env, info, &argc, argv, &jsthis, NULL);

      if (status != napi_ok) {
        napi_throw_error(env, NULL, "Failed to parse arguments");
        return NULL;
      }
      // parse config string passed in
      size_t config_string_size;
      status = napi_get_value_string_utf8(env, argv[0], nullptr, 0, &config_string_size);
      if (status != napi_ok) {
        napi_throw_error(env, NULL, "Failed to get config string length");
        return NULL;
      }

      if (config_string_size > 1024 * 1024) {
        napi_throw_error(env, NULL, "Too large JSON config");
        return NULL;
      }

      char config_string[++config_string_size];
      status = napi_get_value_string_utf8(env, argv[0], config_string, config_string_size,
                                          &config_string_size);
      if (status != napi_ok) {
        napi_throw_error(env, NULL, "Failed to get config string");
        return NULL;
      }

      Actor* obj = new Actor(config_string);

      obj->env_ = env;
      status = napi_wrap(env, jsthis, reinterpret_cast<void*>(obj), Actor::Destructor,
                         nullptr, // finalize_hint
                         &obj->wrapper_);
      if (status != napi_ok) {
        napi_throw_error(env, NULL, "Failed to wrap actor object");
        return NULL;
      }

      return jsthis;
    } else {
      // Invoked as plain function `Actor(...)`, turn into construct call.
      size_t argc_ = 1;
      napi_value args[1];
      status = napi_get_cb_info(env, info, &argc_, args, nullptr, nullptr);
      if (status != napi_ok) {
        napi_throw_error(env, NULL, "Failed to parse input args");
        return NULL;
      }

      const size_t argc = 1;
      napi_value argv[argc] = {args[0]};

      napi_value actor_constructor;
      status = napi_get_reference_value(env, constructor, &actor_constructor);
      if (status != napi_ok) {
        napi_throw_error(env, NULL, "Failed to reference the constructor");
        return NULL;
      }

      napi_value instance;
      status = napi_new_instance(env, actor_constructor, 0, NULL, &instance);
      if (status != napi_ok) {
        napi_throw_error(env, NULL, "Failed to create new instance");
        return NULL;
      }

      return instance;
    }
  }

  static bool
  ParseRequest(napi_env env, napi_callback_info info, napi_value* jsthis, std::string& req_string) {
    napi_status status;
    size_t argc = 1;
    napi_value argv[1];

    status = napi_get_cb_info(env, info, &argc, argv, jsthis, nullptr);
    if (status != napi_ok) {
      napi_throw_error(env, NULL, "Failed to parse argument(s)");
      return false;
    }

    size_t request_str_size;
    status = napi_get_value_string_utf8(env, argv[0], nullptr, 0, &request_str_size);
    if (status != napi_ok) {
      napi_throw_error(env, NULL, "Failed to get argument string length");
      return false;
    }

    if (request_str_size > 1024 * 1024) {
      status = napi_throw_error(env, NULL, "The request exceeds the maximum size");
      return false;
    }
    char request_string[++request_str_size];
    status =
        napi_get_value_string_utf8(env, argv[0], request_string, request_str_size, &request_str_size);
    if (status != napi_ok) {
      napi_throw_error(env, NULL, "Unable to parse argument string");
      return false;
    }
    req_string = std::string(request_string, request_str_size);

    return true;
  }

  static napi_value WrapString(napi_env env, std::string cppStr) {
    napi_value napiStr;
    napi_status status;
    const char* outBuff = cppStr.c_str();
    const auto nchars = cppStr.size();
    status = napi_create_string_utf8(env, outBuff, nchars, &napiStr);
    if (status != napi_ok) {
      napi_throw_error(env, NULL, "Failed to turn c++ string into napi_value");
      return NULL;
    }
    return napiStr;
  }

  static napi_value generic_action(
      napi_env env,
      napi_callback_info info,
      const std::function<std::string(valhalla::tyr::actor_t& actor, const std::string& request)>&
          func) {
    napi_value jsthis;
    napi_status status;
    std::string reqString;
    if (!ParseRequest(env, info, &jsthis, reqString)) {
      return NULL;
    }
    Actor* obj;
    status = napi_unwrap(env, jsthis, reinterpret_cast<void**>(&obj));
    if (status != napi_ok) {
      napi_throw_error(env, NULL, "Failed to unwrap js object");
      return NULL;
    }

    std::string resp_json;
    try {
      resp_json = func(obj->actor, reqString);
    } catch (const valhalla::valhalla_exception_t& e) {
      auto http_code = ERROR_TO_STATUS.find(e.code)->second;
      std::string err_message = "{ error_code: " + std::to_string(e.code) +
                                ", http_code: " + std::to_string(http_code) +
                                ", message: " + e.message + " }";
      napi_throw_error(env, NULL, err_message.c_str());
      return NULL;
    } catch (const std::exception& e) {
      napi_throw_error(env, NULL, e.what());
      return NULL;
    }

    auto outStr = WrapString(env, resp_json);
    return outStr;
  }

  static napi_value Route(napi_env env, napi_callback_info info) {
    return generic_action(env, info,
                          [](valhalla::tyr::actor_t& actor, const std::string& request)
                              -> std::string { return actor.route(request); });
  }

  static napi_value Locate(napi_env env, napi_callback_info info) {
    return generic_action(env, info,
                          [](valhalla::tyr::actor_t& actor, const std::string& request)
                              -> std::string { return actor.locate(request); });
  }

  static napi_value Matrix(napi_env env, napi_callback_info info) {
    return generic_action(env, info,
                          [](valhalla::tyr::actor_t& actor, const std::string& request)
                              -> std::string { return actor.matrix(request); });
  }

  static napi_value OptimizedRoute(napi_env env, napi_callback_info info) {
    return generic_action(env, info,
                          [](valhalla::tyr::actor_t& actor, const std::string& request)
                              -> std::string { return actor.optimized_route(request); });
  }

  static napi_value Isochrone(napi_env env, napi_callback_info info) {
    return generic_action(env, info,
                          [](valhalla::tyr::actor_t& actor, const std::string& request)
                              -> std::string { return actor.isochrone(request); });
  }

  static napi_value TraceRoute(napi_env env, napi_callback_info info) {
    return generic_action(env, info,
                          [](valhalla::tyr::actor_t& actor, const std::string& request)
                              -> std::string { return actor.trace_route(request); });
  }

  static napi_value TraceAttributes(napi_env env, napi_callback_info info) {
    return generic_action(env, info,
                          [](valhalla::tyr::actor_t& actor, const std::string& request)
                              -> std::string { return actor.trace_attributes(request); });
  }

  static napi_value Height(napi_env env, napi_callback_info info) {
    return generic_action(env, info,
                          [](valhalla::tyr::actor_t& actor, const std::string& request)
                              -> std::string { return actor.height(request); });
  }

  static napi_value TransitAvailable(napi_env env, napi_callback_info info) {
    return generic_action(env, info,
                          [](valhalla::tyr::actor_t& actor, const std::string& request)
                              -> std::string { return actor.transit_available(request); });
  }

  static napi_ref constructor;

  valhalla::tyr::actor_t actor;
  napi_env env_;
  napi_ref wrapper_;
};

napi_ref Actor::constructor;

napi_value Init(napi_env env, napi_value exports) {
  napi_value new_exports;
  napi_status status =
      napi_create_function(env, "", NAPI_AUTO_LENGTH, Actor::Init, nullptr, &new_exports);
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Failed to wrap init function");
    return NULL;
  }
  return new_exports;
}

NAPI_MODULE(NODE_GYP_MODULE_NAME, Init)
