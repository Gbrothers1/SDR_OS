mod config;
mod fanout;
mod health;
mod shm;
mod transport;

use std::net::SocketAddr;

use axum::{routing::get, Router};
use tracing_subscriber::EnvFilter;

use crate::config::Config;

#[tokio::main]
async fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env())
        .init();

    let cfg = Config::from_env();

    let app = Router::new().route("/health", get(health));

    let addr: SocketAddr = cfg.listen_addr.parse().expect("invalid SDR_LISTEN_ADDR");
    tracing::info!("transport-server listening on {addr}");
    let listener = tokio::net::TcpListener::bind(addr).await.unwrap();
    axum::serve(listener, app).await.unwrap();
}

async fn health() -> &'static str {
    "{\"status\":\"ok\"}"
}
