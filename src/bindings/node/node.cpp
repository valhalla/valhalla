#include <node_api.h>

#include <boost/make_shared.hpp>
#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/shared_ptr.hpp>
#include <sstream>
#include <string>
#include <iostream>

#include "valhalla/tyr/actor.h"

//NOTE: the Route function has a lot of boiler plate in it so it would be good to break that out
//so that it can be re-used for all of the various functions that need to do input output marshalling

#define DECLARE_NAPI_METHOD(name, func) { name, 0, func, 0, 0, 0, napi_default, 0 }

class Actor {
 public:
  static napi_value Init(napi_env env, napi_value exports) {
    napi_property_descriptor properties[] = {
        DECLARE_NAPI_METHOD("route", Route)
        //many more...
    };

    napi_value cons;
    napi_status status =
        napi_define_class(env, "Actor", NAPI_AUTO_LENGTH, New, nullptr, 1, properties, &cons);
    if (status != napi_ok) {
      napi_throw_error(env, NULL, "Failed to define Actor class");
    }

    status = napi_create_reference(env, cons, 1, &constructor);
    if (status != napi_ok) {
      napi_throw_error(env, NULL, "Failed to create reference to constructor");
    }

    status = napi_set_named_property(env, exports, "Actor", cons);
    if (status != napi_ok) {
      napi_throw_error(env, NULL, "Failed to set Actor property on exports");
    }

    status = napi_set_named_property(env, exports, "configure", Configure);
    if (status != napi_ok) {
      napi_throw_error(env, NULL, "Failed to set configure property on exports");
    }
    return exports;
  }

  static void Destructor(napi_env env, void* nativeObject, void* finalize_hint) {
    // reinterpret_cast<valhalla::tyr::actor_t*>(nativeObject)->~MyObject();
    delete reinterpret_cast<valhalla::tyr::actor_t*>(nativeObject);
  }

 private:
  explicit Actor(): actor(get_config()), env_(nullptr), wrapper_(nullptr) {
  }

  ~Actor() {
    napi_delete_reference(env_, wrapper_);
  }

  boost::property_tree::ptree Configuration;

  boost::property_tree::ptree json_to_pt(const std::string& json) {
    std::stringstream ss;
    ss << json;
    boost::property_tree::ptree pt;
    boost::property_tree::read_json(ss, pt);
    return pt;
  }

  boost::property_tree::ptree make_conf(const std::string config) {
    return json_to_pt(config);
  }

  boost::property_tree::ptree get_config() {
    if (Configuration) {
      return Configuration;
    }
    throw std::exception("Module must be configured before it can be run");
  }

  static napi_value New(napi_env env, napi_callback_info info) {
    napi_status status;

    napi_value target;
    status = napi_get_new_target(env, info, &target);
    if (status != napi_ok) {
      napi_throw_error(env, NULL, "Failed to get new target");
    }
    bool is_constructor = target != nullptr;

    if (is_constructor) {
      // Invoked as constructor: `new MyObject(...)`
      // currently not making use of any args, leaving in case we add some
      size_t argc = 1;
      napi_value args[1];
      napi_value jsthis;
      status = napi_get_cb_info(env, info, &argc, args, &jsthis, nullptr);
      if (status != napi_ok) {
        napi_throw_error(env, NULL, "Failed to parse constructor params");
      }
      // No need to get args to the constructor for now
      Actor* obj = new Actor();

      obj->env_ = env;
      status = napi_wrap(env,
                         jsthis,
                         reinterpret_cast<void*>(obj),
                         Actor::Destructor,
                         nullptr,  // finalize_hint
                         &obj->wrapper_);

      if (status != napi_ok) {
        napi_throw_error(env, NULL, "Failed to wrap Actor");
      }
      return jsthis;
    }

    // Invoked as plain function `MyObject(...)`, turn into construct call.
    // NOTE: for now we could skip the args here because we dont accept any
    // but we'll leave it in case we end up adding some later
    size_t argc_ = 1;
    napi_value args[1];
    status = napi_get_cb_info(env, info, &argc_, args, nullptr, nullptr);
    if (status != napi_ok) {
      napi_throw_error(env, NULL, "Failed to parse constructor params");
    }

    const size_t argc = 1;
    napi_value argv[argc] = {args[0]};

    napi_value cons;
    status = napi_get_reference_value(env, constructor, &cons);
    if (status != napi_ok) {
      napi_throw_error(env, NULL, "Failed to get reference value");
    }

    napi_value instance;
    status = napi_new_instance(env, cons, argc, argv, &instance);
    if (status != napi_ok) {
      napi_throw_error(env, NULL, "Failed to create new instance");
    }

    return instance;
  }

  napi_value Configure(napi_env env, napi_callback_info info) {
    napi_status status;
    size_t argc = 1;
    napi_value argv[1];

    status = napi_get_cb_info(env, info, &argc, argv, NULL, NULL);

    if (status != napi_ok) {
      napi_throw_error(env, NULL, "Failed to parse arguments");
    }

    char buffer[5000];
    size_t buffer_size;

    // parse config string passed in
    status = napi_get_value_string_utf8(env, argv[0], buffer, 5000, &buffer_size);

    if (status != napi_ok) {
      napi_throw_error(env, NULL, "Failed to parse config to string");
    }

    // parse config string into ptree, save in Configuration constant
    const std::string parsed_config_string(buffer, buffer_size);
    Configuration = make_conf(parsed_config_string);
  }

  static napi_value Route(napi_env env, napi_callback_info info) {
    napi_status status;

    size_t argc = 1;
    napi_value args[1];
    napi_value jsthis;
    status = napi_get_cb_info(env, info, &argc, args, &jsthis, nullptr);
    if (status != napi_ok) {
      napi_throw_error(env, NULL, "Failed to parse input params");
    }

    napi_valuetype valuetype;
    status = napi_typeof(env, args[0], &valuetype);
    if (status != napi_ok) {
      napi_throw_error(env, NULL, "Failed to parse input into napi_value");
    }

    //pull out the string argument
    char* buffer = new char[5000];
    size_t bytes_copied;
    if (valuetype != napi_undefined) {
      status = napi_get_value_string_utf8(env, args[0], buffer, 5000, bytes_copied);
      if (status != napi_ok) {
        napi_throw_error(env, NULL, "Failed to parse input to string");
      }
      if (bytes_copied >= 5000) {
        napi_throw_error(env, NULL, "input too long.");
      }
    }

    //get the instance we are working on
    Actor* obj;
    status = napi_unwrap(env, jsthis, reinterpret_cast<void**>(&obj));
    if (status != napi_ok) {
      napi_throw_error(env, NULL, "Failed to unwrap js instance");
    }

    //do the work
    std::string jsonresult;
    try {
      jsonresult = obj->actor.route(std::string(buffer));
    } catch (const std::exception& e) {
      napi_throw_error(env, NULL, e.what());
    }

    //make a javascript string
    napi_value result;
    status = napi_create_string_utf8(env, jsonresult.c_str(), NAPI_AUTO_LENGTH, &result);
    if (status != napi_ok) {
      napi_throw_error(env, NULL, "Failed to create string out of result");
    }

    return result;
  }
  //more here
  static napi_ref constructor;
  valhalla::tyr::actor_t actor;
  napi_env env_;
  napi_ref wrapper_;
};
