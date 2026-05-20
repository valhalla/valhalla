# OpenAPI Specification

The Valhalla API is described in a machine-readable OpenAPI 3.1 spec at [`docs/docs/api/openapi.yaml`](https://github.com/valhalla/valhalla/blob/master/docs/docs/api/openapi.yaml). Use it to explore endpoints interactively, generate client code, or validate requests and responses.

## Browse it locally

The quickest way is Docker — no install required:

=== "Swagger UI"

    ```bash
    docker run --rm -p 8080:8080 \
      -e SWAGGER_JSON_URL=https://raw.githubusercontent.com/valhalla/valhalla/master/docs/docs/api/openapi.yaml \
      swaggerapi/swagger-ui
    ```

    Or mount the local file instead:

    ```bash
    docker run --rm -p 8080:8080 \
      -v $(pwd)/docs/docs/api/openapi.yaml:/openapi.yaml \
      -e SWAGGER_JSON=/openapi.yaml \
      swaggerapi/swagger-ui
    ```

    Open [http://localhost:8080](http://localhost:8080).

=== "Redoc"

    ```bash
    docker run --rm -p 8080:80 \
      -v $(pwd)/docs/docs/api/openapi.yaml:/usr/share/nginx/html/openapi.yaml \
      -e SPEC_URL=openapi.yaml \
      redocly/redoc
    ```

    Open [http://localhost:8080](http://localhost:8080).

=== "Stoplight Elements (npx)"

    ```bash
    npx @stoplight/elements-dev-portal --spec docs/docs/api/openapi.yaml
    ```

## Build a standalone HTML page

Redocly can bundle the spec into a single self-contained HTML file — useful for sharing or hosting without a running server:

```bash
npx @redocly/cli build-docs docs/docs/api/openapi.yaml \
  -o redoc-static.html \
  --title "Valhalla API"
```

Open `redoc-static.html` directly in a browser. No server needed.

To serve it immediately after building:

```bash
npx @redocly/cli build-docs docs/docs/api/openapi.yaml -o redoc-static.html \
  && python3 -m http.server 8080
```

Then open [http://localhost:8080/redoc-static.html](http://localhost:8080/redoc-static.html).

If you just want to browse interactively without a build step, the Docker options above skip straight to serving.

## Validate the spec

```bash
# Using the Redocly CLI (no Docker needed)
npx @redocly/cli lint docs/docs/api/openapi.yaml

# Or via Docker
docker run --rm -v $(pwd):/spec redocly/cli lint /spec/docs/docs/api/openapi.yaml
```

## Generate a client

```bash
# Python client via openapi-generator
docker run --rm -v $(pwd):/local openapitools/openapi-generator-cli generate \
  -i /local/docs/docs/api/openapi.yaml \
  -g python \
  -o /local/generated/python-client
```

Any generator supported by [openapi-generator](https://openapi-generator.tech/docs/generators) works the same way — swap `-g python` for the target language.

## Try requests against a live server

The spec's default server is `https://valhalla.openstreetmap.de`. To point it at a local instance, either edit the `servers` block in `openapi.yaml` or override it in the Swagger UI server dropdown.

```bash
# Quick smoke test with curl (local server on port 8002)
curl -s http://localhost:8002/route \
  -H 'Content-Type: application/json' \
  -d '{"locations":[{"lat":47.141,"lon":9.521},{"lat":47.165,"lon":9.510}],"costing":"auto"}' \
  | jq '.trip.summary'
```

## Keeping the spec in sync

The spec lives alongside the docs in `docs/docs/api/openapi.yaml`. When you add or change an API parameter, update both the relevant Markdown reference (e.g. `api/turn-by-turn/api-reference.md`) and the corresponding schema in `openapi.yaml`.
