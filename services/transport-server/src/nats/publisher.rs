use async_nats::Client;
use bytes::Bytes;

/// Publishes a command to NATS with subject `command.genesis.{action}`.
pub async fn publish_command(
    nats: &Client,
    action: &str,
    payload: &[u8],
) -> Result<(), async_nats::PublishError> {
    let subject = format!("command.genesis.{}", action);
    nats.publish(subject, Bytes::copy_from_slice(payload)).await
}
