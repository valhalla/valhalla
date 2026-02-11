#include "baldr/graphid.h"
#include "baldr/tilehierarchy.h"
#include "midgard/aabb2.h"
#include "midgard/pointll.h"
#include "tile_id_utils.h"

#include <napi.h>

#include <string>
#include <vector>

namespace vb = valhalla::baldr;
namespace vm = valhalla::midgard;

static Napi::FunctionReference graphid_constructor;

class GraphIdWrapper : public Napi::ObjectWrap<GraphIdWrapper> {
public:
  static Napi::Object Init(Napi::Env env, Napi::Object exports) {
    Napi::Function func = DefineClass(env, "GraphId",
                                      {InstanceAccessor("value", &GraphIdWrapper::GetValue, nullptr),
                                       InstanceMethod("tileid", &GraphIdWrapper::TileId),
                                       InstanceMethod("level", &GraphIdWrapper::Level),
                                       InstanceMethod("id", &GraphIdWrapper::Id),
                                       InstanceMethod("is_valid", &GraphIdWrapper::IsValid),
                                       InstanceMethod("tile_base", &GraphIdWrapper::TileBase),
                                       InstanceMethod("tile_value", &GraphIdWrapper::TileValue),
                                       InstanceMethod("add", &GraphIdWrapper::Add),
                                       InstanceMethod("equals", &GraphIdWrapper::Equals),
                                       InstanceMethod("toString", &GraphIdWrapper::ToString),
                                       InstanceMethod("toJSON", &GraphIdWrapper::ToJSON)});

    graphid_constructor = Napi::Persistent(func);
    graphid_constructor.SuppressDestruct();

    exports.Set("GraphId", func);
    return exports;
  }

  static Napi::Object NewInstance(Napi::Env env, const vb::GraphId& gid) {
    return graphid_constructor.New({Napi::Number::New(env, static_cast<double>(gid.value))});
  }

  GraphIdWrapper(const Napi::CallbackInfo& info) : Napi::ObjectWrap<GraphIdWrapper>(info) {
    Napi::Env env = info.Env();

    try {
      if (info.Length() == 0) {
        graph_id_ = vb::GraphId();
      } else if (info.Length() == 1) {
        if (info[0].IsString()) {
          // String constructor: "level/tileid/id"
          std::string str = info[0].As<Napi::String>().Utf8Value();
          graph_id_ = vb::GraphId(str);
        } else if (info[0].IsNumber()) {
          // Numeric value constructor
          uint64_t val = static_cast<uint64_t>(info[0].As<Napi::Number>().Int64Value());
          graph_id_ = vb::GraphId(val);
        } else if (info[0].IsBigInt()) {
          bool lossless;
          uint64_t val = info[0].As<Napi::BigInt>().Uint64Value(&lossless);
          if (!lossless) {
            Napi::TypeError::New(env, "Value too large for uint64").ThrowAsJavaScriptException();
            return;
          }
          graph_id_ = vb::GraphId(val);
        } else {
          Napi::TypeError::New(env, "Expected string, number, or no arguments")
              .ThrowAsJavaScriptException();
          return;
        }
      } else if (info.Length() == 3) {
        // (tileid, level, id) constructor
        if (!info[0].IsNumber() || !info[1].IsNumber() || !info[2].IsNumber()) {
          Napi::TypeError::New(env, "Expected three numbers (tileid, level, id)")
              .ThrowAsJavaScriptException();
          return;
        }
        uint32_t tileid = info[0].As<Napi::Number>().Uint32Value();
        uint32_t level = info[1].As<Napi::Number>().Uint32Value();
        uint32_t id = info[2].As<Napi::Number>().Uint32Value();
        graph_id_ = vb::GraphId(tileid, level, id);
      } else {
        Napi::TypeError::New(env, "Expected 0, 1, or 3 arguments").ThrowAsJavaScriptException();
        return;
      }
    } catch (const std::exception& e) {
      Napi::Error::New(env, e.what()).ThrowAsJavaScriptException();
    }
  }

  vb::GraphId graphId() const {
    return graph_id_;
  }

private:
  vb::GraphId graph_id_;

  Napi::Value GetValue(const Napi::CallbackInfo& info) {
    return Napi::Number::New(info.Env(), static_cast<double>(graph_id_.value));
  }

  Napi::Value TileId(const Napi::CallbackInfo& info) {
    return Napi::Number::New(info.Env(), graph_id_.tileid());
  }

  Napi::Value Level(const Napi::CallbackInfo& info) {
    return Napi::Number::New(info.Env(), graph_id_.level());
  }

  Napi::Value Id(const Napi::CallbackInfo& info) {
    return Napi::Number::New(info.Env(), graph_id_.id());
  }

  Napi::Value IsValid(const Napi::CallbackInfo& info) {
    return Napi::Boolean::New(info.Env(), graph_id_.is_valid());
  }

  Napi::Value TileBase(const Napi::CallbackInfo& info) {
    return NewInstance(info.Env(), graph_id_.tile_base());
  }

  Napi::Value TileValue(const Napi::CallbackInfo& info) {
    return Napi::Number::New(info.Env(), graph_id_.tile_value());
  }

  Napi::Value Add(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();
    if (info.Length() < 1 || !info[0].IsNumber()) {
      Napi::TypeError::New(env, "Expected a number offset").ThrowAsJavaScriptException();
      return env.Undefined();
    }
    uint64_t offset = static_cast<uint64_t>(info[0].As<Napi::Number>().Int64Value());
    try {
      return NewInstance(env, graph_id_ + offset);
    } catch (const std::exception& e) {
      Napi::Error::New(env, e.what()).ThrowAsJavaScriptException();
      return env.Undefined();
    }
  }

  Napi::Value Equals(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();
    if (info.Length() < 1 || !info[0].IsObject()) {
      return Napi::Boolean::New(env, false);
    }
    Napi::Object obj = info[0].As<Napi::Object>();
    if (!obj.InstanceOf(graphid_constructor.Value())) {
      return Napi::Boolean::New(env, false);
    }
    GraphIdWrapper* other = Napi::ObjectWrap<GraphIdWrapper>::Unwrap(obj);
    return Napi::Boolean::New(env, graph_id_ == other->graph_id_);
  }

  Napi::Value ToString(const Napi::CallbackInfo& info) {
    return Napi::String::New(info.Env(), std::to_string(graph_id_));
  }

  Napi::Value ToJSON(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();
    Napi::Object obj = Napi::Object::New(env);
    obj.Set("level", Napi::Number::New(env, graph_id_.level()));
    obj.Set("tileid", Napi::Number::New(env, graph_id_.tileid()));
    obj.Set("id", Napi::Number::New(env, graph_id_.id()));
    obj.Set("value", Napi::Number::New(env, static_cast<double>(graph_id_.value)));
    return obj;
  }
};

Napi::Value GetTileIdFromLonLat(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  // getTileIdFromLonLat(level, [lon, lat])
  if (info.Length() < 2 || !info[0].IsNumber() || !info[1].IsArray()) {
    Napi::TypeError::New(env, "Expected (level, [lon, lat])").ThrowAsJavaScriptException();
    return env.Undefined();
  }

  try {
    uint32_t level = info[0].As<Napi::Number>().Uint32Value();
    valhalla::bindings::check_level(level);

    Napi::Array coord = info[1].As<Napi::Array>();
    if (coord.Length() != 2) {
      Napi::TypeError::New(env, "Coordinate must have 2 elements (lon, lat)")
          .ThrowAsJavaScriptException();
      return env.Undefined();
    }
    double x = coord.Get(static_cast<uint32_t>(0)).As<Napi::Number>().DoubleValue();
    double y = coord.Get(static_cast<uint32_t>(1)).As<Napi::Number>().DoubleValue();
    valhalla::bindings::check_coord(x, y, x, y);

    auto gid = vb::TileHierarchy::GetGraphId(vm::PointLL{x, y}, static_cast<uint8_t>(level));
    return GraphIdWrapper::NewInstance(env, gid);
  } catch (const std::exception& e) {
    Napi::Error::New(env, e.what()).ThrowAsJavaScriptException();
    return env.Undefined();
  }
}

Napi::Value GetTileIdsFromBbox(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  // getTileIdsFromBbox(minx, miny, maxx, maxy, levels?)
  if (info.Length() < 4 || !info[0].IsNumber() || !info[1].IsNumber() || !info[2].IsNumber() ||
      !info[3].IsNumber()) {
    Napi::TypeError::New(env, "Expected four numbers (minx, miny, maxx, maxy)")
        .ThrowAsJavaScriptException();
    return env.Undefined();
  }

  try {
    double minx = info[0].As<Napi::Number>().DoubleValue();
    double miny = info[1].As<Napi::Number>().DoubleValue();
    double maxx = info[2].As<Napi::Number>().DoubleValue();
    double maxy = info[3].As<Napi::Number>().DoubleValue();
    valhalla::bindings::check_coord(minx, miny, maxx, maxy);

    std::vector<uint32_t> levels;
    if (info.Length() > 4 && info[4].IsArray()) {
      Napi::Array levels_arr = info[4].As<Napi::Array>();
      levels.reserve(levels_arr.Length());
      for (uint32_t i = 0; i < levels_arr.Length(); i++) {
        levels.push_back(levels_arr.Get(i).As<Napi::Number>().Uint32Value());
      }
    }
    if (levels.empty()) {
      levels = valhalla::bindings::default_levels();
    }

    const vm::AABB2<vm::PointLL> bbox{minx, miny, maxx, maxy};
    std::vector<vb::GraphId> tile_ids;
    for (const auto level : levels) {
      valhalla::bindings::check_level(level);
      const auto level_tile_ids = vb::TileHierarchy::levels().at(level).tiles.TileList(bbox);
      tile_ids.reserve(tile_ids.size() + level_tile_ids.size());
      for (const auto tid : level_tile_ids) {
        tile_ids.emplace_back(vb::GraphId{static_cast<uint32_t>(tid), level, 0});
      }
    }

    Napi::Array result = Napi::Array::New(env, tile_ids.size());
    for (size_t i = 0; i < tile_ids.size(); i++) {
      result.Set(static_cast<uint32_t>(i), GraphIdWrapper::NewInstance(env, tile_ids[i]));
    }
    return result;
  } catch (const std::exception& e) {
    Napi::Error::New(env, e.what()).ThrowAsJavaScriptException();
    return env.Undefined();
  }
}

Napi::Value GetTileIdsFromRing(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  if (info.Length() < 1 || !info[0].IsArray()) {
    Napi::TypeError::New(env, "Expected an array of [lon, lat] coordinates")
        .ThrowAsJavaScriptException();
    return env.Undefined();
  }

  try {
    Napi::Array ring_arr = info[0].As<Napi::Array>();

    // parse JS coordinate arrays into PointLL
    std::vector<vm::PointLL> ring;
    ring.reserve(ring_arr.Length());
    for (uint32_t i = 0; i < ring_arr.Length(); i++) {
      Napi::Value elem = ring_arr[i];
      if (!elem.IsArray()) {
        Napi::TypeError::New(env, "Each coordinate must be an array of [lon, lat]")
            .ThrowAsJavaScriptException();
        return env.Undefined();
      }
      Napi::Array coord = elem.As<Napi::Array>();
      if (coord.Length() != 2) {
        Napi::TypeError::New(env, "Each coordinate must have 2 elements (lon, lat)")
            .ThrowAsJavaScriptException();
        return env.Undefined();
      }
      double x = coord.Get(static_cast<uint32_t>(0)).As<Napi::Number>().DoubleValue();
      double y = coord.Get(static_cast<uint32_t>(1)).As<Napi::Number>().DoubleValue();
      valhalla::bindings::check_coord(x, y, x, y);
      ring.push_back({x, y});
    }

    // parse optional levels array
    std::vector<uint32_t> levels;
    if (info.Length() > 1 && info[1].IsArray()) {
      Napi::Array levels_arr = info[1].As<Napi::Array>();
      levels.reserve(levels_arr.Length());
      for (uint32_t i = 0; i < levels_arr.Length(); i++) {
        levels.push_back(levels_arr.Get(i).As<Napi::Number>().Uint32Value());
      }
    }

    auto tile_ids = valhalla::bindings::get_tile_ids_from_ring(std::move(ring), std::move(levels));

    // convert result to JS array of GraphId objects
    Napi::Array result = Napi::Array::New(env, tile_ids.size());
    for (size_t i = 0; i < tile_ids.size(); i++) {
      result.Set(static_cast<uint32_t>(i), GraphIdWrapper::NewInstance(env, tile_ids[i]));
    }
    return result;
  } catch (const std::exception& e) {
    Napi::Error::New(env, e.what()).ThrowAsJavaScriptException();
    return env.Undefined();
  }
}

// Init function called from the main module
Napi::Object InitGraphId(Napi::Env env, Napi::Object exports) {
  GraphIdWrapper::Init(env, exports);
  exports.Set("getTileIdFromLonLat",
              Napi::Function::New(env, GetTileIdFromLonLat, "getTileIdFromLonLat"));
  exports.Set("getTileIdsFromBbox",
              Napi::Function::New(env, GetTileIdsFromBbox, "getTileIdsFromBbox"));
  exports.Set("getTileIdsFromRing",
              Napi::Function::New(env, GetTileIdsFromRing, "getTileIdsFromRing"));
  return exports;
}
