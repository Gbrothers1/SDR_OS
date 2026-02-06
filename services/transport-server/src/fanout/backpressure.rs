use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use tokio::sync::Mutex;

/// Per-client state for lag detection and keyframe recovery.
#[derive(Debug)]
pub enum ClientState {
    /// Normal operation: forwarding all frames.
    Streaming,
    /// Lagged: waiting for a keyframe before resuming.
    AwaitingKeyframe { since: Instant },
}

impl ClientState {
    pub fn is_awaiting_keyframe(&self) -> bool {
        matches!(self, Self::AwaitingKeyframe { .. })
    }
}

/// Global IDR request coalescer.
/// Ensures only one IDR request is sent per `min_interval`.
pub struct IdrCoalescer {
    last_request: Mutex<Instant>,
    min_interval: Duration,
    pending: AtomicBool,
}

impl IdrCoalescer {
    pub fn new(min_interval_ms: u64) -> Self {
        Self {
            // Start in the past so first request always goes through
            last_request: Mutex::new(Instant::now() - Duration::from_secs(60)),
            min_interval: Duration::from_millis(min_interval_ms),
            pending: AtomicBool::new(false),
        }
    }

    /// Request an IDR. Returns true if the request should be sent (not coalesced).
    pub async fn request_idr(&self) -> bool {
        // Fast path: if there's already a pending request, don't bother
        if self.pending.load(Ordering::Relaxed) {
            return false;
        }

        let mut last = self.last_request.lock().await;
        if last.elapsed() >= self.min_interval {
            *last = Instant::now();
            self.pending.store(true, Ordering::Relaxed);
            true
        } else {
            false
        }
    }

    /// Mark the IDR as delivered (keyframe received).
    pub fn idr_received(&self) {
        self.pending.store(false, Ordering::Relaxed);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_client_state_transitions() {
        let state = ClientState::Streaming;
        assert!(!state.is_awaiting_keyframe());

        let state = ClientState::AwaitingKeyframe {
            since: Instant::now(),
        };
        assert!(state.is_awaiting_keyframe());
    }

    #[tokio::test]
    async fn test_idr_coalescer_first_request_allowed() {
        let coalescer = IdrCoalescer::new(500);
        assert!(coalescer.request_idr().await);
    }

    #[tokio::test]
    async fn test_idr_coalescer_second_request_blocked() {
        let coalescer = IdrCoalescer::new(500);
        assert!(coalescer.request_idr().await);
        assert!(!coalescer.request_idr().await);
    }

    #[tokio::test]
    async fn test_idr_coalescer_allows_after_interval() {
        let coalescer = IdrCoalescer::new(20); // 20ms interval
        assert!(coalescer.request_idr().await);

        // Simulate keyframe received (clears pending flag)
        coalescer.idr_received();

        tokio::time::sleep(Duration::from_millis(50)).await;
        assert!(coalescer.request_idr().await);
    }

    #[test]
    fn test_idr_received_resets_pending() {
        let coalescer = IdrCoalescer::new(500);
        coalescer.pending.store(true, Ordering::Relaxed);
        coalescer.idr_received();
        assert!(!coalescer.pending.load(Ordering::Relaxed));
    }
}
