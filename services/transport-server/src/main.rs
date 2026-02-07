mod config;
mod fanout;
mod health;
mod nats;
mod shm;
mod transport;

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Instant;

use axum::extract::ws::{Message, WebSocket};
use axum::extract::{State, WebSocketUpgrade};
use axum::response::IntoResponse;
use axum::routing::get;
use axum::Json;
use axum::Router;
use futures_util::{SinkExt, StreamExt};
use tokio::sync::broadcast;
use tracing_subscriber::EnvFilter;

use crate::config::Config;
use crate::fanout::backpressure::ClientState;
use crate::fanout::broadcast::FrameBroadcast;
use crate::health::{HealthResponse, HealthState};
use crate::nats::subscriber::{telemetry_relay, TelemetryMsg};
use crate::shm::reader::{ReadResult, ShmReader};
use crate::transport::websocket::{encode_video_frame, MSG_TYPE_COMMAND};

/// Video gate state shared between SHM reader and WS handlers.
struct VideoGateState {
    gate_active: AtomicBool,
    estop_active: AtomicBool,
    last_shm_frame_time: std::sync::Mutex<Instant>,
}

/// Shared application state passed to axum handlers.
#[derive(Clone)]
struct AppState {
    frame_broadcast: Arc<FrameBroadcast>,
    telemetry_tx: broadcast::Sender<Arc<TelemetryMsg>>,
    health: HealthState,
    idr_timeout_ms: u64,
    nats_client: Option<async_nats::Client>,
    video_gate: Arc<VideoGateState>,
    video_gate_hold_ms: u64,
    video_gate_estop_ms: u64,
}

#[tokio::main]
async fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info")),
        )
        .init();

    let cfg = Config::from_env();

    // Frame broadcast
    let frame_broadcast = Arc::new(FrameBroadcast::new(cfg.broadcast_capacity));

    // Telemetry broadcast
    let (telemetry_tx, _) = broadcast::channel::<Arc<TelemetryMsg>>(32);

    // Health state
    let health_state = HealthState {
        client_count: Arc::clone(&frame_broadcast.client_count),
        shm_seq: Arc::new(AtomicU64::new(0)),
        last_frame_time: Arc::new(std::sync::Mutex::new(None)),
        start_time: Instant::now(),
    };

    // Connect to NATS (non-fatal if unavailable)
    let nats_client = match async_nats::connect(&cfg.nats_url).await {
        Ok(client) => {
            tracing::info!(url = %cfg.nats_url, "connected to NATS");
            Some(client)
        }
        Err(e) => {
            tracing::warn!(url = %cfg.nats_url, error = %e, "NATS unavailable, commands will not be relayed");
            None
        }
    };

    // Spawn NATS telemetry relay (uses its own subscription on the shared connection)
    if let Some(ref nc) = nats_client {
        let nc = nc.clone();
        let nats_subject = cfg.telemetry_subjects.clone();
        let nats_max_size = cfg.telemetry_max_size;
        let nats_tx = telemetry_tx.clone();
        tokio::spawn(async move {
            telemetry_relay(nc, &nats_subject, nats_max_size, nats_tx).await;
        });
    }

    let video_gate = Arc::new(VideoGateState {
        gate_active: AtomicBool::new(false),
        estop_active: AtomicBool::new(false),
        last_shm_frame_time: std::sync::Mutex::new(Instant::now()),
    });

    // Spawn SHM reader loop
    let shm_health = health_state.clone();
    let shm_broadcast = Arc::clone(&frame_broadcast);
    let shm_path = cfg.shm_path.clone();
    let shm_size = cfg.shm_size;
    let crc_enabled = cfg.crc_enabled;
    let shm_gate = Arc::clone(&video_gate);
    let shm_nats = nats_client.clone();
    tokio::spawn(async move {
        shm_read_loop(shm_path, shm_size, crc_enabled, shm_broadcast, shm_health, shm_gate, shm_nats).await;
    });

    let app_state = AppState {
        frame_broadcast: Arc::clone(&frame_broadcast),
        telemetry_tx: telemetry_tx.clone(),
        health: health_state.clone(),
        idr_timeout_ms: cfg.idr_timeout_ms,
        nats_client,
        video_gate,
        video_gate_hold_ms: cfg.video_gate_hold_ms,
        video_gate_estop_ms: cfg.video_gate_estop_ms,
    };

    // Build router
    let app = Router::new()
        .route("/stream/ws", get(ws_upgrade_handler))
        .route("/health", get(app_health_handler))
        .with_state(app_state);

    let addr: std::net::SocketAddr = cfg.listen_addr.parse().expect("invalid SDR_LISTEN_ADDR");
    tracing::info!("transport-server listening on {addr}");
    let listener = tokio::net::TcpListener::bind(addr).await.unwrap();
    axum::serve(listener, app).await.unwrap();
}

/// Health endpoint that extracts HealthState from AppState.
async fn app_health_handler(State(state): State<AppState>) -> Json<HealthResponse> {
    let last_frame_age_ms = state
        .health
        .last_frame_time
        .lock()
        .ok()
        .and_then(|guard| guard.map(|t| t.elapsed().as_millis() as u64));

    Json(HealthResponse {
        status: "ok",
        clients: state.health.client_count.load(Ordering::Relaxed),
        shm_seq: state.health.shm_seq.load(Ordering::Relaxed),
        last_frame_age_ms,
        uptime_s: state.health.start_time.elapsed().as_secs(),
    })
}

async fn ws_upgrade_handler(
    ws: WebSocketUpgrade,
    State(state): State<AppState>,
) -> impl IntoResponse {
    ws.on_upgrade(move |socket| handle_ws_client(socket, state))
}

async fn handle_ws_client(socket: WebSocket, state: AppState) {
    let (ws_sender, ws_receiver) = socket.split();
    let ws_sender = Arc::new(tokio::sync::Mutex::new(ws_sender));

    let (video_rx, guard) = state.frame_broadcast.subscribe();
    let telemetry_rx = state.telemetry_tx.subscribe();

    tracing::info!(
        clients = state.frame_broadcast.client_count(),
        "client connected"
    );

    unified_send_loop(
        ws_sender,
        ws_receiver,
        video_rx,
        telemetry_rx,
        guard,
        state.idr_timeout_ms,
        state.nats_client.clone(),
        Arc::clone(&state.video_gate),
        state.video_gate_hold_ms,
        state.video_gate_estop_ms,
    )
    .await;

    tracing::info!(
        clients = state.frame_broadcast.client_count(),
        "client disconnected"
    );
}

/// Merged video + telemetry + command send loop per client.
///
/// Uses `tokio::select!` to multiplex video frames, telemetry messages,
/// and inbound 0x04 commands onto a single WebSocket connection.
/// Handles per-client backpressure: if the client lags behind the broadcast,
/// it drops non-keyframes until a keyframe arrives (or the IDR timeout fires).
async fn unified_send_loop(
    ws_sender: Arc<
        tokio::sync::Mutex<futures_util::stream::SplitSink<WebSocket, Message>>,
    >,
    mut ws_receiver: futures_util::stream::SplitStream<WebSocket>,
    mut video_rx: broadcast::Receiver<Arc<crate::shm::reader::Frame>>,
    mut telemetry_rx: broadcast::Receiver<Arc<TelemetryMsg>>,
    _guard: crate::fanout::broadcast::ClientGuard,
    idr_timeout_ms: u64,
    nats_client: Option<async_nats::Client>,
    video_gate: Arc<VideoGateState>,
    video_gate_hold_ms: u64,
    video_gate_estop_ms: u64,
) {
    let mut client_state = ClientState::AwaitingKeyframe {
        since: Instant::now(),
    };
    let idr_timeout = std::time::Duration::from_millis(idr_timeout_ms);

    loop {
        tokio::select! {
            result = video_rx.recv() => {
                match result {
                    Ok(frame) => {
                        match &client_state {
                            ClientState::AwaitingKeyframe { since } => {
                                if since.elapsed() > idr_timeout {
                                    tracing::warn!("IDR timeout, disconnecting client");
                                    break;
                                }
                                if frame.header.is_keyframe() {
                                    client_state = ClientState::Streaming;
                                } else {
                                    continue;
                                }
                            }
                            ClientState::Streaming => {}
                        }

                        let data = encode_video_frame(&frame);
                        let mut sender = ws_sender.lock().await;
                        if sender.send(Message::Binary(data.into())).await.is_err() {
                            break;
                        }
                    }
                    Err(broadcast::error::RecvError::Lagged(n)) => {
                        tracing::warn!(lagged = n, "client lagged, awaiting keyframe");
                        client_state = ClientState::AwaitingKeyframe {
                            since: Instant::now(),
                        };
                    }
                    Err(broadcast::error::RecvError::Closed) => break,
                }
            }
            result = telemetry_rx.recv() => {
                match result {
                    Ok(msg) => {
                        let mut sender = ws_sender.lock().await;
                        if sender.send(Message::Binary(msg.encoded.clone().into())).await.is_err() {
                            break;
                        }
                    }
                    Err(broadcast::error::RecvError::Lagged(_)) => {
                        // Silently drop old telemetry
                    }
                    Err(broadcast::error::RecvError::Closed) => break,
                }
            }
            msg = ws_receiver.next() => {
                match msg {
                    Some(Ok(Message::Binary(data))) => {
                        if data.len() > 1 && data[0] == MSG_TYPE_COMMAND {
                            if let Some(ref nc) = nats_client {
                                if let Ok(cmd) = serde_json::from_slice::<serde_json::Value>(&data[1..]) {
                                    if let Some(action) = cmd.get("action").and_then(|a| a.as_str()) {
                                        let is_always_allowed = matches!(action, "estop" | "estop_clear" | "pause" | "reset");
                                        let is_motion = matches!(action, "set_cmd_vel" | "set_alpha");

                                        // Video gate: block motion commands when video stale
                                        if is_motion && !is_always_allowed {
                                            let stale_ms = video_gate.last_shm_frame_time
                                                .lock().unwrap().elapsed().as_millis() as u64;

                                            if stale_ms > video_gate_estop_ms {
                                                // ESTOP escalation — publish once
                                                if !video_gate.estop_active.swap(true, Ordering::SeqCst) {
                                                    let estop = serde_json::json!({"reason": "video_timeout", "_safety": "estop"});
                                                    let _ = nats::publisher::publish_command(nc, "estop", &serde_json::to_vec(&estop).unwrap()).await;
                                                    let gate_msg = serde_json::json!({"gated": true, "mode": "ESTOP", "stale_ms": stale_ms});
                                                    let _ = nc.publish("telemetry.safety.video_gate".to_string(), serde_json::to_vec(&gate_msg).unwrap().into()).await;
                                                    tracing::warn!(stale_ms, "video gate ESTOP — video stale, blocking all motion");
                                                }
                                                continue;
                                            } else if stale_ms > video_gate_hold_ms {
                                                // HOLD — latch one zero velocity, then drop
                                                if !video_gate.gate_active.swap(true, Ordering::SeqCst) {
                                                    let zero_vel = serde_json::json!({"action": "set_cmd_vel", "data": {"linear_x": 0, "linear_y": 0, "angular_z": 0}, "_safety": "hold"});
                                                    let _ = nats::publisher::publish_command(nc, "set_cmd_vel", &serde_json::to_vec(&zero_vel).unwrap()).await;
                                                    let gate_msg = serde_json::json!({"gated": true, "mode": "HOLD", "stale_ms": stale_ms});
                                                    let _ = nc.publish("telemetry.safety.video_gate".to_string(), serde_json::to_vec(&gate_msg).unwrap().into()).await;
                                                    tracing::warn!(stale_ms, "video gate HOLD — latched zero velocity");
                                                }
                                                continue;
                                            }
                                        }

                                        let payload = &data[1..];
                                        if let Err(e) = nats::publisher::publish_command(nc, action, payload).await {
                                            tracing::warn!(action, error = %e, "failed to publish command to NATS");
                                        }
                                    }
                                }
                            }
                        }
                    }
                    Some(Ok(Message::Close(_))) | None => break,
                    _ => {}
                }
            }
        }
    }
}

async fn shm_read_loop(
    shm_path: String,
    shm_size: usize,
    crc_enabled: bool,
    broadcast: Arc<FrameBroadcast>,
    health: HealthState,
    video_gate: Arc<VideoGateState>,
    nats_client: Option<async_nats::Client>,
) {
    // Wait for the SHM file to appear
    loop {
        if std::path::Path::new(&shm_path).exists() {
            break;
        }
        tracing::debug!(path = %shm_path, "waiting for SHM file");
        tokio::time::sleep(std::time::Duration::from_secs(1)).await;
    }

    // Open mmap
    let file = match std::fs::File::open(&shm_path) {
        Ok(f) => f,
        Err(e) => {
            tracing::error!(path = %shm_path, error = %e, "failed to open SHM file");
            return;
        }
    };

    let mmap = match unsafe { memmap2::Mmap::map(&file) } {
        Ok(m) => m,
        Err(e) => {
            tracing::error!(error = %e, "failed to mmap SHM file");
            return;
        }
    };

    if mmap.len() < shm_size {
        tracing::warn!(
            actual = mmap.len(),
            expected = shm_size,
            "SHM file smaller than expected"
        );
    }

    let mut reader = ShmReader::new(mmap, crc_enabled);
    tracing::info!(path = %shm_path, "SHM reader started");

    loop {
        // No-client optimization: sleep longer and skip broadcast
        if broadcast.client_count() == 0 {
            reader.enter_sleep();
            tokio::time::sleep(std::time::Duration::from_millis(100)).await;
            // Still update seq for health endpoint
            if let ReadResult::Frame(_) = reader.try_read_frame() {
                health.shm_seq.store(reader.last_seq(), Ordering::Relaxed);
                *health.last_frame_time.lock().unwrap() = Some(Instant::now());
                *video_gate.last_shm_frame_time.lock().unwrap() = Instant::now();
            }
            continue;
        }

        match reader.try_read_frame() {
            ReadResult::Frame(frame) => {
                health.shm_seq.store(reader.last_seq(), Ordering::Relaxed);
                *health.last_frame_time.lock().unwrap() = Some(Instant::now());
                *video_gate.last_shm_frame_time.lock().unwrap() = Instant::now();

                // Clear gate if video recovered
                if video_gate.gate_active.swap(false, Ordering::SeqCst) {
                    if let Some(ref nc) = nats_client {
                        let msg = serde_json::json!({"gated": false});
                        let _ = nc.publish("telemetry.safety.video_gate".to_string(), serde_json::to_vec(&msg).unwrap().into()).await;
                        tracing::info!("video gate cleared — video recovered");
                    }
                }
                video_gate.estop_active.store(false, Ordering::SeqCst);

                reader.on_frame_received();
                broadcast.send_frame(frame);
            }
            ReadResult::NoNewFrame => {
                match reader.poll_delay() {
                    Some(d) => tokio::time::sleep(d).await,
                    None => {
                        tokio::task::yield_now().await;
                    }
                }
            }
            ReadResult::TornRead => {
                std::hint::spin_loop();
            }
            ReadResult::CrcMismatch | ReadResult::BadHeader | ReadResult::InvalidLength => {
                tracing::warn!("SHM read error, retrying");
                tokio::task::yield_now().await;
            }
            ReadResult::SkippedNonKeyframe => {
                // Normal during lap recovery
            }
        }
    }
}
