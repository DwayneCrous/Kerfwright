//! A computation on a background thread whose result the UI polls once per frame.

use std::sync::mpsc::{channel, Receiver, TryRecvError};

pub struct Job<T> {
    rx: Receiver<Result<T, String>>,
    pub label: String,
    started: std::time::Instant,
}

impl<T: Send + 'static> Job<T> {
    pub fn spawn(label: &str, f: impl FnOnce() -> Result<T, String> + Send + 'static) -> Job<T> {
        let (tx, rx) = channel();
        std::thread::Builder::new()
            .name(format!("job: {label}"))
            .stack_size(64 << 20)
            .spawn(move || {
                let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(f))
                    .unwrap_or_else(|_| Err("the computation panicked".into()));
                let _ = tx.send(r);
            })
            .expect("spawn job thread");
        Job {
            rx,
            label: label.into(),
            started: std::time::Instant::now(),
        }
    }

    pub fn elapsed(&self) -> f64 {
        self.started.elapsed().as_secs_f64()
    }

    /// `None` while running; the result once, then the job is spent.
    pub fn poll(&self) -> Option<Result<T, String>> {
        match self.rx.try_recv() {
            Ok(r) => Some(r),
            Err(TryRecvError::Empty) => None,
            Err(TryRecvError::Disconnected) => {
                Some(Err("the computation ended without a result".into()))
            }
        }
    }
}
