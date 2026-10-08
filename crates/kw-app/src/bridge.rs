use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use tokio::sync::oneshot;

/// Generation-based cancellation. A new request bumps the generation; work
/// started under an older generation observes `is_cancelled()` and bails out.
#[derive(Debug, Clone, Default)]
pub struct CancelToken {
    current: Arc<AtomicU64>,
    mine: u64,
}

impl CancelToken {
    pub fn new() -> Self {
        Self::default()
    }

    /// Supersede all in-flight work and return a token for the new request.
    pub fn next(&self) -> CancelToken {
        let generation = self.current.fetch_add(1, Ordering::SeqCst) + 1;
        CancelToken {
            current: Arc::clone(&self.current),
            mine: generation,
        }
    }

    pub fn is_cancelled(&self) -> bool {
        self.current.load(Ordering::SeqCst) != self.mine
    }
}

/// Run CPU-bound `work` on the Rayon pool and await the result from async
/// code without blocking a Tokio worker. If the receiver is dropped the
/// result is discarded.
pub fn spawn_compute<F, R>(work: F) -> oneshot::Receiver<R>
where
    F: FnOnce() -> R + Send + 'static,
    R: Send + 'static,
{
    let (tx, rx) = oneshot::channel();
    rayon::spawn(move || {
        let _ = tx.send(work());
    });
    rx
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn newer_token_cancels_older() {
        let root = CancelToken::new();
        let a = root.next();
        assert!(!a.is_cancelled());
        let b = root.next();
        assert!(a.is_cancelled());
        assert!(!b.is_cancelled());
    }

    #[tokio::test]
    async fn compute_returns_result() {
        let rx = spawn_compute(|| (1..=10u64).sum::<u64>());
        assert_eq!(rx.await.unwrap(), 55);
    }
}
