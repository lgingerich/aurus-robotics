use std::sync::Arc;
use tokio::sync::broadcast;

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

    pub fn publish(&self, msg: T) {
        let _ = self.tx.send(Arc::new(msg));
    }

    pub fn subscribe(&self) -> broadcast::Receiver<Arc<T>> {
        self.tx.subscribe()
    }
}
