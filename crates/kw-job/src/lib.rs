//! Long-running job support shared by the CPU-heavy crates (meshing, FEM, CAM, routing): cooperative
//! cancellation, progress reporting and a thread pool whose size is capped.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;

/// A flag a job polls to find out whether it should stop.
#[derive(Debug, Clone, Default)]
pub struct CancelFlag(Arc<AtomicBool>);

impl CancelFlag {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn cancel(&self) {
        self.0.store(true, Ordering::SeqCst);
    }
    pub fn is_cancelled(&self) -> bool {
        self.0.load(Ordering::SeqCst)
    }
    pub fn reset(&self) {
        self.0.store(false, Ordering::SeqCst);
    }
}

/// Fraction done in parts per million, written by the job and read by whoever shows it.
#[derive(Debug, Clone, Default)]
pub struct Progress(Arc<AtomicU64>);

impl Progress {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn set(&self, fraction: f64) {
        self.0
            .store((fraction.clamp(0.0, 1.0) * 1e6) as u64, Ordering::Relaxed);
    }
    pub fn get(&self) -> f64 {
        self.0.load(Ordering::Relaxed) as f64 / 1e6
    }
}

/// Why a cancellable job returned early.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Cancelled;

impl std::fmt::Display for Cancelled {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "cancelled")
    }
}
impl std::error::Error for Cancelled {}

/// Upper bound on the threads a job may use: `RAYON_NUM_THREADS` if set, else
/// `min(8, cores − 2)`, never less than one and never more than 8 without an explicit request.
pub fn thread_cap() -> usize {
    if let Some(n) = std::env::var("RAYON_NUM_THREADS")
        .ok()
        .and_then(|s| s.parse::<usize>().ok())
        .filter(|n| *n > 0)
    {
        return n;
    }
    let cores = std::thread::available_parallelism().map_or(1, |n| n.get());
    cores.saturating_sub(2).clamp(1, 8)
}

/// A pool of `min(threads, thread_cap())` workers with a large stack (deep recursions in the geometry
/// code).
pub fn par_pool(threads: usize) -> rayon::ThreadPool {
    rayon::ThreadPoolBuilder::new()
        .num_threads(threads.clamp(1, thread_cap()))
        .stack_size(32 << 20)
        .build()
        .expect("thread pool")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cancel_flag_is_shared_between_clones() {
        let a = CancelFlag::new();
        let b = a.clone();
        assert!(!b.is_cancelled());
        a.cancel();
        assert!(b.is_cancelled());
        b.reset();
        assert!(!a.is_cancelled());
    }

    #[test]
    fn progress_is_clamped_and_shared() {
        let p = Progress::new();
        let q = p.clone();
        p.set(0.25);
        assert!((q.get() - 0.25).abs() < 1e-6);
        p.set(7.0);
        assert_eq!(q.get(), 1.0);
    }

    #[test]
    fn pool_respects_the_cap() {
        let cap = thread_cap();
        assert!((1..=64).contains(&cap));
        let pool = par_pool(10_000);
        assert!(pool.current_num_threads() <= cap);
        assert_eq!(par_pool(1).current_num_threads(), 1);
    }
}
