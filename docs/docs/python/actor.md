# Actor

The `Actor` class is the main entry point for calling Valhalla's actions
(route, isochrone, matrix, etc.) from Python. It wraps the C++ `tyr::actor_t`
and accepts both JSON strings and Python `dict` requests.

For request/response schemas, see the
[Turn-by-Turn API reference](../api/turn-by-turn/api-reference.md) and the
sibling pages under [Public APIs](../api/index.md).

::: valhalla.actor.Actor
