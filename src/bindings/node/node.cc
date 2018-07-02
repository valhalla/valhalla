#include <boost/make_shared.hpp>
#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <node_api.h>
#include <sstream>
#include <string>

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

// check if napi_status is nap_ok, throw error if not
void checkNapiStatus(napi_status status, napi_env env, const char* error_message) {
  if (status != napi_ok) {
    napi_throw_error(env, NULL, error_message);
  }
}

#define DECLARE_NAPI_METHOD(name, func)                                                              \
  { name, 0, func, 0, 0, 0, napi_default, 0 }

class Actor {
public:
  static napi_value Init(napi_env env, napi_callback_info info) {
    napi_status status;
    napi_property_descriptor properties[] = {
        DECLARE_NAPI_METHOD("route", Route),
    };
    // parse config file to get logging config
    size_t argc = 1;
    napi_value argv[1];

    status = napi_get_cb_info(env, info, &argc, argv, nullptr, nullptr);
    checkNapiStatus(status, env, "Failed to parse arguments");

    // parse config string passed in
    size_t config_string_size;
    status = napi_get_value_string_utf8(env, argv[0], nullptr, 0, &config_string_size);
    checkNapiStatus(status, env, "Failed to get config string length");
    if (config_string_size > 1024 * 1024) {
      napi_throw_error(env, NULL, "Too large JSON config");
    }

    char config_string[++config_string_size];
    status = napi_get_value_string_utf8(env, argv[0], config_string, config_string_size,
                                        &config_string_size);
    checkNapiStatus(status, env, "Failed to get config string");
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
    } catch (...) { napi_throw_error(env, NULL, "Failed to load logging config"); }

    napi_value actor_constructor;
    status = napi_define_class(env, "Actor", NAPI_AUTO_LENGTH, New, nullptr, 1, properties,
                               &actor_constructor);
    checkNapiStatus(status, env, "Failed to define class");

    status = napi_create_reference(env, actor_constructor, 1, &constructor);
    checkNapiStatus(status, env, "Failed to create constructor reference");

    return actor_constructor;
  }
  static void Destructor(napi_env env, void* nativeObject, void* finalize_hint) {
    delete reinterpret_cast<Actor*>(nativeObject);
  }

private:
  explicit Actor(const char* config) : actor(make_conf(config)), env_(nullptr), wrapper_(nullptr) {
  }
  ~Actor() {
    napi_delete_reference(env_, wrapper_);
  }

  static napi_value New(napi_env env, napi_callback_info info) {
    napi_status status;

    napi_value target;
    status = napi_get_new_target(env, info, &target);
    checkNapiStatus(status, env, "Failed to get 'new' target");
    bool is_constructor = target != nullptr;

    if (is_constructor) {
      // Invoked as constructor: `new Actor(...)`
      size_t argc = 1;
      napi_value jsthis;
      napi_value argv[1];

      status = napi_get_cb_info(env, info, &argc, argv, &jsthis, NULL);
      checkNapiStatus(status, env, "Failed to parse arguments");

      // parse config string passed in
      size_t config_string_size;
      status = napi_get_value_string_utf8(env, argv[0], nullptr, 0, &config_string_size);
      checkNapiStatus(status, env, "Failed to get config string length");

      if (config_string_size > 1024 * 1024) {
        napi_throw_error(env, NULL, "Too large JSON config");
      }

      char config_string[++config_string_size];
      status = napi_get_value_string_utf8(env, argv[0], config_string, config_string_size,
                                          &config_string_size);
      checkNapiStatus(status, env, "Failed to get config string");

      Actor* obj = new Actor(config_string);

      obj->env_ = env;
      status = napi_wrap(env, jsthis, reinterpret_cast<void*>(obj), Actor::Destructor,
                         nullptr, // finalize_hint
                         &obj->wrapper_);
      checkNapiStatus(status, env, "Failed to wrap actor object");

      return jsthis;
    } else {
      // Invoked as plain function `Actor(...)`, turn into construct call.
      size_t argc_ = 1;
      napi_value args[1];
      status = napi_get_cb_info(env, info, &argc_, args, nullptr, nullptr);
      checkNapiStatus(status, env, "Failed to parse input args");

      const size_t argc = 1;
      napi_value argv[argc] = {args[0]};

      napi_value actor_constructor;
      status = napi_get_reference_value(env, constructor, &actor_constructor);
      checkNapiStatus(status, env, "Failed to reference the constructor");

      napi_value instance;
      status = napi_new_instance(env, actor_constructor, 0, NULL, &instance);
      checkNapiStatus(status, env, "Failed to create new instance");

      return instance;
    }
  }

  static std::string ParseRequest(napi_env env, napi_callback_info info, napi_value* jsthis) {
    napi_status status;
    size_t argc = 1;
    napi_value argv[1];

    status = napi_get_cb_info(env, info, &argc, argv, jsthis, nullptr);
    checkNapiStatus(status, env, "Failed to parse input args");

    size_t request_str_size;
    status = napi_get_value_string_utf8(env, argv[0], nullptr, 0, &request_str_size);
    checkNapiStatus(status, env, "Failed to get arg string length");

    if (request_str_size > 1024 * 1024) {
      napi_throw_error(env, NULL, "Too large JSON config");
    }

    char request_string[++request_str_size];
    status =
        napi_get_value_string_utf8(env, argv[0], request_string, request_str_size, &request_str_size);
    checkNapiStatus(status, env, "Unable to parse arg string");
    std::string reqString(request_string, request_str_size);

    return reqString;
  }

  static napi_value WrapString(napi_env env, std::string cppStr) {
    napi_value napiStr;
    napi_status status;
    const char* outBuff = cppStr.c_str();
    const auto nchars = cppStr.size();
    status = napi_create_string_utf8(env, outBuff, nchars, &napiStr);
    checkNapiStatus(status, env, "Failed to turn c++ string into napi_value");
    return napiStr;
  }

  static napi_value Route(napi_env env, napi_callback_info info) {
    napi_value jsthis;
    napi_status status;
    // parse input arg into string
    std::string reqString = ParseRequest(env, info, &jsthis);

    Actor* obj;
    status = napi_unwrap(env, jsthis, reinterpret_cast<void**>(&obj));
    checkNapiStatus(status, env, "Failed to unwrap js object");

    // get the actual route
    std::string route_json;
    try {
      route_json = obj->actor.route(reqString);
    } catch (const std::exception& e) { napi_throw_error(env, NULL, e.what()); }

    // wrap route_json in napi value for return
    auto outStr = WrapString(env, route_json);
    return outStr;
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
  checkNapiStatus(status, env, "Failed to wrap init function");
  return new_exports;
}

NAPI_MODULE(NODE_GYP_MODULE_NAME, Init)
