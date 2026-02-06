# Stage 1: Build
FROM rust:1.84-slim AS builder

RUN apt-get update && apt-get install -y --no-install-recommends \
    pkg-config libssl-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /build
COPY Cargo.toml Cargo.lock ./
COPY src/ src/

RUN cargo build --release

# Stage 2: Runtime
FROM debian:bookworm-slim

RUN apt-get update && apt-get install -y --no-install-recommends \
    libssl3 wget ca-certificates \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /build/target/release/transport-server /usr/local/bin/transport-server

EXPOSE 8080

CMD ["transport-server"]
