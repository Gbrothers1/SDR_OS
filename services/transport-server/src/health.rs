use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Instant;

use axum::extract::State;
use axum::Json;
use serde::Serialize;

/// Shared health state, updated by the SHM reader loop.
#[derive(Clone)]
pub struct HealthState {
    pub client_count: Arc<std::sync::atomic::AtomicUsize>,
    pub shm_seq: Arc<AtomicU64>,
    pub last_frame_time: Arc<std::sync::Mutex<Option<Instant>>>,
    pub start_time: Instant,
    pub frames_broadcast: Arc<AtomicU64>,
    pub client_lag_total: Arc<AtomicU64>,
    pub frame_size_acc: Arc<AtomicU64>,
    pub frame_size_count: Arc<AtomicU64>,
}

#[derive(Serialize)]
pub struct HealthResponse {
    pub status: &'static str,
    pub clients: usize,
    pub shm_seq: u64,
    pub last_frame_age_ms: Option<u64>,
    pub uptime_s: u64,
    pub frames_broadcast: u64,
    pub client_lag_total: u64,
    pub avg_frame_size_bytes: u64,
}

pub async fn health_handler(State(state): State<HealthState>) -> Json<HealthResponse> {
    let last_frame_age_ms = state
        .last_frame_time
        .lock()
        .ok()
        .and_then(|guard| guard.map(|t| t.elapsed().as_millis() as u64));

    let count = state.frame_size_count.load(Ordering::Relaxed);
    let avg_frame_size = if count > 0 {
        state.frame_size_acc.load(Ordering::Relaxed) / count
    } else {
        0
    };

    Json(HealthResponse {
        status: "ok",
        clients: state.client_count.load(Ordering::Relaxed),
        shm_seq: state.shm_seq.load(Ordering::Relaxed),
        last_frame_age_ms,
        uptime_s: state.start_time.elapsed().as_secs(),
        frames_broadcast: state.frames_broadcast.load(Ordering::Relaxed),
        client_lag_total: state.client_lag_total.load(Ordering::Relaxed),
        avg_frame_size_bytes: avg_frame_size,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_health_response_serializes() {
        let resp = HealthResponse {
            status: "ok",
            clients: 3,
            shm_seq: 42,
            last_frame_age_ms: Some(15),
            uptime_s: 120,
            frames_broadcast: 1000,
            client_lag_total: 5,
            avg_frame_size_bytes: 8192,
        };
        let json = serde_json::to_string(&resp).unwrap();
        assert!(json.contains("\"status\":\"ok\""));
        assert!(json.contains("\"clients\":3"));
        assert!(json.contains("\"shm_seq\":42"));
        assert!(json.contains("\"last_frame_age_ms\":15"));
        assert!(json.contains("\"frames_broadcast\":1000"));
    }

    #[test]
    fn test_health_response_null_frame_age() {
        let resp = HealthResponse {
            status: "ok",
            clients: 0,
            shm_seq: 0,
            last_frame_age_ms: None,
            uptime_s: 0,
            frames_broadcast: 0,
            client_lag_total: 0,
            avg_frame_size_bytes: 0,
        };
        let json = serde_json::to_string(&resp).unwrap();
        assert!(json.contains("\"last_frame_age_ms\":null"));
    }
}
