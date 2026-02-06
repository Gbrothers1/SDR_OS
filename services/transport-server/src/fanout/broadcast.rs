use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;

use tokio::sync::broadcast;

use crate::shm::reader::Frame;

/// Shared state for the frame broadcast system.
pub struct FrameBroadcast {
    pub sender: broadcast::Sender<Arc<Frame>>,
    pub client_count: Arc<AtomicUsize>,
}

impl FrameBroadcast {
    pub fn new(capacity: usize) -> Self {
        let (sender, _) = broadcast::channel(capacity);
        Self {
            sender,
            client_count: Arc::new(AtomicUsize::new(0)),
        }
    }

    /// Subscribe a new client. Returns a receiver and a guard that decrements
    /// the client count on drop.
    pub fn subscribe(&self) -> (broadcast::Receiver<Arc<Frame>>, ClientGuard) {
        self.client_count.fetch_add(1, Ordering::Relaxed);
        let rx = self.sender.subscribe();
        let guard = ClientGuard {
            count: Arc::clone(&self.client_count),
        };
        (rx, guard)
    }

    pub fn client_count(&self) -> usize {
        self.client_count.load(Ordering::Relaxed)
    }

    /// Send a frame to all subscribers. Returns the number of active receivers.
    pub fn send_frame(&self, frame: Frame) -> usize {
        self.sender.send(Arc::new(frame)).unwrap_or(0)
    }
}

/// RAII guard that decrements the client count when dropped.
pub struct ClientGuard {
    count: Arc<AtomicUsize>,
}

impl Drop for ClientGuard {
    fn drop(&mut self) {
        self.count.fetch_sub(1, Ordering::Relaxed);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shm::protocol::{FrameHeader, CODEC_H264, FLAG_KEYFRAME};

    fn make_frame(seq: u64, keyframe: bool) -> Frame {
        let payload = format!("frame-{seq}").into_bytes();
        Frame {
            header: FrameHeader {
                frame_id: seq,
                frame_seq: seq,
                size: payload.len() as u32,
                flags: if keyframe { FLAG_KEYFRAME } else { 0 },
                codec: CODEC_H264,
                crc32: 0,
                reserved: 0,
            },
            payload,
        }
    }

    #[test]
    fn test_client_count_lifecycle() {
        let bc = FrameBroadcast::new(16);
        assert_eq!(bc.client_count(), 0);

        let (_rx1, g1) = bc.subscribe();
        assert_eq!(bc.client_count(), 1);

        let (_rx2, g2) = bc.subscribe();
        assert_eq!(bc.client_count(), 2);

        drop(g1);
        assert_eq!(bc.client_count(), 1);

        drop(g2);
        assert_eq!(bc.client_count(), 0);
    }

    #[tokio::test]
    async fn test_frame_delivery() {
        let bc = FrameBroadcast::new(16);
        let (mut rx, _guard) = bc.subscribe();

        let frame = make_frame(1, true);
        bc.send_frame(frame);

        let received = rx.recv().await.unwrap();
        assert_eq!(received.header.frame_seq, 1);
        assert!(received.header.is_keyframe());
    }
}
