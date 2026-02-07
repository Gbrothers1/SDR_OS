use std::sync::Arc;

use futures_util::StreamExt;
use tokio::sync::broadcast;

use crate::transport::websocket::encode_telemetry;

/// A telemetry message ready for broadcast.
#[derive(Debug, Clone)]
pub struct TelemetryMsg {
    /// Pre-encoded binary WS frame (type byte + subject + payload).
    pub encoded: Vec<u8>,
}

/// Run the NATS telemetry relay loop.
///
/// Subscribes to the configured subject pattern, encodes messages as binary
/// WS frames, and sends them to the telemetry broadcast channel.
pub async fn telemetry_relay(
    client: async_nats::Client,
    subject: &str,
    max_payload_size: usize,
    tx: broadcast::Sender<Arc<TelemetryMsg>>,
) {
    let mut sub = match client.subscribe(subject.to_string()).await {
        Ok(s) => {
            tracing::info!(subject, "subscribed to NATS telemetry");
            s
        }
        Err(e) => {
            tracing::error!(subject, error = %e, "failed to subscribe to NATS telemetry");
            return;
        }
    };

    while let Some(msg) = sub.next().await {
        let subject = msg.subject.as_str();
        let payload = msg.payload.as_ref();

        // Size cap
        if payload.len() > max_payload_size {
            tracing::warn!(
                subject,
                size = payload.len(),
                max = max_payload_size,
                "telemetry message too large, dropping"
            );
            continue;
        }

        // Encode as binary WS frame
        let encoded = match encode_telemetry(subject, payload) {
            Some(e) => e,
            None => {
                tracing::warn!(subject, "telemetry subject too long, dropping");
                continue;
            }
        };

        let msg = Arc::new(TelemetryMsg { encoded });

        // Best-effort send; if no receivers, that's fine
        let _ = tx.send(msg);
    }
}

// Integration tests for NATS relay require nats-server - covered in Task 11.
