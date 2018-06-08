#include <assert.h>
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
  static napi_value Init(napi_env env, napi_value exports) {
    napi_status status;
    napi_property_descriptor properties[] = {
        DECLARE_NAPI_METHOD("route", Route),
    };

    napi_value cons;
    status = napi_define_class(env, "Actor", NAPI_AUTO_LENGTH, New, nullptr, 1, properties, &cons);
    assert(status == napi_ok);

    status = napi_create_reference(env, cons, 1, &constructor);
    assert(status == napi_ok);

    status = napi_set_named_property(env, exports, "Actor", cons);
    assert(status == napi_ok);
    return exports;
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
    assert(status == napi_ok);
    bool is_constructor = target != nullptr;

    if (is_constructor) {
      // Invoked as constructor: `new Actor(...)`
      size_t argc = 1;
      napi_value jsthis;
      napi_value argv[1];

      status = napi_get_cb_info(env, info, &argc, argv, &jsthis, NULL);
      checkNapiStatus(status, env, "Failed to parse arguments");

      // parse config string passed in
      size_t buffer_size;
      status = napi_get_value_string_utf8(env, argv[0], nullptr, 0, &buffer_size);
      checkNapiStatus(status, env, "Failed to get config string length");

      if (buffer_size > 1024 * 1024) {
        napi_throw_error(env, NULL, "Too large JSON config");
      }

      char buffer[++buffer_size];
      status = napi_get_value_string_utf8(env, argv[0], buffer, buffer_size, &buffer_size);
      checkNapiStatus(status, env, "Failed to get config string");

      Actor* obj = new Actor(buffer);

      obj->env_ = env;
      status = napi_wrap(env, jsthis, reinterpret_cast<void*>(obj), Actor::Destructor,
                         nullptr, // finalize_hint
                         &obj->wrapper_);
      assert(status == napi_ok);

      return jsthis;
    } else {
      // Invoked as plain function `Actor(...)`, turn into construct call.
      size_t argc_ = 1;
      napi_value args[1];
      status = napi_get_cb_info(env, info, &argc_, args, nullptr, nullptr);
      assert(status == napi_ok);

      const size_t argc = 1;
      napi_value argv[argc] = {args[0]};

      napi_value cons;
      status = napi_get_reference_value(env, constructor, &cons);
      assert(status == napi_ok);

      napi_value instance;
      status = napi_new_instance(env, cons, 0, NULL, &instance);
      assert(status == napi_ok);

      return instance;
    }
  }

  static napi_value Route(napi_env env, napi_callback_info info) {
    napi_status status;
    napi_value jsthis;
    size_t argc = 1;
    napi_value argv[1];

    status = napi_get_cb_info(env, info, &argc, argv, &jsthis, nullptr);
    assert(status == napi_ok);

    Actor* obj;
    status = napi_unwrap(env, jsthis, reinterpret_cast<void**>(&obj));
    assert(status == napi_ok);

    size_t buffer_size;
    status = napi_get_value_string_utf8(env, argv[0], nullptr, 0, &buffer_size);
    checkNapiStatus(status, env, "Failed to get config string length");

    if (buffer_size > 1024 * 1024) {
      napi_throw_error(env, NULL, "Too large JSON config");
    }

    char buffer[++buffer_size];
    status = napi_get_value_string_utf8(env, argv[0], buffer, buffer_size, &buffer_size);
    checkNapiStatus(status, env, "Unable to parse req string");
    std::string reqString(buffer, buffer_size);

    assert(status == napi_ok);
    std::string route_json;
    try {
      route_json = obj->actor.route(reqString);
    } catch (const std::exception& e) { napi_throw_error(env, NULL, e.what()); }

    napi_value myStr;
    const char* outBuff = route_json.c_str();
    const auto nchars = route_json.size();

    status = napi_create_string_utf8(env, outBuff, nchars, &myStr);
    return myStr;
  }

  static napi_ref constructor;

  valhalla::tyr::actor_t actor;
  napi_env env_;
  napi_ref wrapper_;
};

napi_ref Actor::constructor;

napi_value Init(napi_env env, napi_value exports) {
  return Actor::Init(env, exports);
}

NAPI_MODULE(NODE_GYP_MODULE_NAME, Init)
