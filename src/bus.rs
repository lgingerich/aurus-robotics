use std::sync::Arc;
use tokio::sync::broadcast;
use tracing::warn;

/// Broadcast topic with bounded capacity.
/// `T` must be `Send + Sync` because we hop across threads.
#[derive(Debug, Clone)]
pub struct Topic<T> {
    tx: broadcast::Sender<Arc<T>>,
}

impl<T: Send + Sync + 'static> Topic<T> {
    pub fn new(capacity: usize) -> Self {
        let (tx, _) = broadcast::channel(capacity);
        Self { tx }
    }

    pub fn publish(&self, value: T) {
        if self.tx.receiver_count() > 0 {
            if let Err(e) = self.tx.send(Arc::new(value)) {
                warn!("Failed to publish to topic: {}", e);
            }
        }
    }

    pub fn subscribe(&self) -> broadcast::Receiver<Arc<T>> {
        self.tx.subscribe()
    }

    /// Returns a clone of the underlying broadcast sender.
    pub fn get_sender(&self) -> broadcast::Sender<Arc<T>> {
        self.tx.clone()
    }
}
