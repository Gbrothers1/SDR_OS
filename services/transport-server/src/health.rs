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
}

#[derive(Serialize)]
pub struct HealthResponse {
    pub status: &'static str,
    pub clients: usize,
    pub shm_seq: u64,
    pub last_frame_age_ms: Option<u64>,
    pub uptime_s: u64,
}

pub async fn health_handler(State(state): State<HealthState>) -> Json<HealthResponse> {
    let last_frame_age_ms = state
        .last_frame_time
        .lock()
        .ok()
        .and_then(|guard| guard.map(|t| t.elapsed().as_millis() as u64));

    Json(HealthResponse {
        status: "ok",
        clients: state.client_count.load(Ordering::Relaxed),
        shm_seq: state.shm_seq.load(Ordering::Relaxed),
        last_frame_age_ms,
        uptime_s: state.start_time.elapsed().as_secs(),
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
        };
        let json = serde_json::to_string(&resp).unwrap();
        assert!(json.contains("\"status\":\"ok\""));
        assert!(json.contains("\"clients\":3"));
        assert!(json.contains("\"shm_seq\":42"));
        assert!(json.contains("\"last_frame_age_ms\":15"));
    }

    #[test]
    fn test_health_response_null_frame_age() {
        let resp = HealthResponse {
            status: "ok",
            clients: 0,
            shm_seq: 0,
            last_frame_age_ms: None,
            uptime_s: 0,
        };
        let json = serde_json::to_string(&resp).unwrap();
        assert!(json.contains("\"last_frame_age_ms\":null"));
    }
}
