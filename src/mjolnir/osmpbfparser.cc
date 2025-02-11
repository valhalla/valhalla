#include "mjolnir/osmpbfparser.h"

#include <osmium/io/pbf_input.hpp>
#include <osmium/visitor.hpp>

namespace OSMPBF {

namespace {
OSMPBF::Tags convert_tags(const osmium::TagList& tags) {
  OSMPBF::Tags result(tags.size());
  for (const auto& tag : tags) {
    result[tag.key()] = tag.value();
  }
  return result;
}

OSMPBF::Relation::MemberType convert_member_type(const osmium::item_type type) {
  switch (type) {
    case osmium::item_type::node:
      return OSMPBF::Relation::MemberType::NODE;
    case osmium::item_type::way:
      return OSMPBF::Relation::MemberType::WAY;
    case osmium::item_type::relation:
      return OSMPBF::Relation::MemberType::RELATION;
    default:
      return OSMPBF::Relation::MemberType::NODE;
  }
}

struct Adapter : public osmium::handler::Handler {
  Callback& callback;
  bool read_changesets;

  Adapter(Callback& callback, bool read_changesets)
      : callback(callback), read_changesets(read_changesets) {
  }

  void way(const osmium::Way& way) {
    std::vector<uint64_t> nodes;
    nodes.reserve(way.nodes().size());
    for (const auto& node : way.nodes()) {
      nodes.push_back(node.ref());
    }
    callback.way_callback(way.id(), convert_tags(way.tags()), nodes);

    if (read_changesets) {
      callback.changeset_callback(way.changeset());
    }
  }

  void node(const osmium::Node& node) {
    callback.node_callback(node.id(), node.location().lon(), node.location().lat(),
                           convert_tags(node.tags()));

    if (read_changesets) {
      callback.changeset_callback(node.changeset());
    }
  }

  void relation(const osmium::Relation& relation) {
    std::vector<Member> members;
    members.reserve(relation.members().size());
    for (const auto& member : relation.members()) {
      members.emplace_back(convert_member_type(member.type()), member.ref(), member.role());
    }
    callback.relation_callback(relation.id(), convert_tags(relation.tags()), members);

    if (read_changesets) {
      callback.changeset_callback(relation.changeset());
    }
  }

  void changeset(const osmium::Changeset& changeset) {
    callback.changeset_callback(changeset.id());
  }
};
} // namespace

void parse(const std::string& file, const Interest interest, Callback& callback) {
  const std::tuple<Interest, osmium::osm_entity_bits::type> mapping[] = {
      {Interest::NODES, osmium::osm_entity_bits::node},
      {Interest::WAYS, osmium::osm_entity_bits::way},
      {Interest::RELATIONS, osmium::osm_entity_bits::relation},
      {Interest::CHANGESETS, osmium::osm_entity_bits::changeset},
  };
  osmium::osm_entity_bits::type read_types = osmium::osm_entity_bits::nothing;
  for (const auto& [our_bit, osmium_bit] : mapping) {
    if ((interest & our_bit) == our_bit) {
      read_types |= osmium_bit;
    }
  }

  osmium::io::Reader reader(file, read_types);

  const bool read_changesets = (interest & Interest::CHANGESETS) == Interest::CHANGESETS;
  Adapter adapter(callback, read_changesets);
  osmium::apply(reader, adapter);

  // Explicit close might throw an exception in case of an error.
  reader.close();
}

} // namespace OSMPBF
