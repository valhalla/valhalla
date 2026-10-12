# An isolated environment for tests and sanity checks on CI

ARG RUST_VERSION=1.98
FROM rust:${RUST_VERSION}-slim-trixie AS builder

# Rust 1.98 is built on LLVM 22: clang/lld must be the same major for `-Clinker-plugin-lto`, bump them together.
ARG LLVM_VERSION=22

# Rust tools
RUN rustup component add rustfmt clippy

# LLVM toolchain from apt.llvm.org, as Debian trixie ships only LLVM 19
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates curl gnupg \
    && curl -fsSL https://apt.llvm.org/llvm-snapshot.gpg.key | gpg --dearmor -o /usr/share/keyrings/llvm.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/llvm.gpg] https://apt.llvm.org/trixie/ llvm-toolchain-trixie-${LLVM_VERSION} main" > /etc/apt/sources.list.d/llvm.list

# System dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    pkg-config \
    clang-${LLVM_VERSION} \
    llvm-${LLVM_VERSION} \
    lld-${LLVM_VERSION} \
    # Valhalla build dependencies
    build-essential \
    cmake \
    libboost-dev \
    libprotobuf-dev \
    protobuf-compiler \
    zlib1g-dev

# https://doc.rust-lang.org/beta/rustc/linker-plugin-lto.html
ENV CC=clang-${LLVM_VERSION} CXX=clang++-${LLVM_VERSION} AR=llvm-ar-${LLVM_VERSION} RANLIB=llvm-ranlib-${LLVM_VERSION}
ENV RUSTFLAGS="-Clinker-plugin-lto -Clinker=clang-${LLVM_VERSION} -Clink-arg=-fuse-ld=lld-${LLVM_VERSION}"

WORKDIR /usr/src/app

COPY . .

# Check formatting before building to avoid unnecessary rebuilds
RUN cargo fmt --all --check
RUN cargo fmt --all --check --manifest-path examples/Cargo.toml

RUN cargo clippy -- -Dwarnings
RUN cargo test
RUN cargo build --release

RUN cargo clippy --manifest-path examples/Cargo.toml -- -Dwarnings
RUN cargo test --manifest-path examples/Cargo.toml

# Multi-stage build example:
# ```
# FROM debian:trixie-slim AS runner
# WORKDIR /usr
# # Runtime dependency for valhalla
# RUN apt-get update && apt-get install -y --no-install-recommends libprotobuf-lite32
# # Running integration tests to ensure that all runtime deps are installed correctly
# COPY --from=builder /usr/src/app/target/release/my-app /usr/local/bin/my-app
# ENTRYPOINT [ "my-app" ]
# ```
