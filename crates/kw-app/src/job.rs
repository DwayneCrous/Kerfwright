//! A computation on a background thread whose result the UI polls once per frame.
//!
//! A job can be cancelled and reports progress. The closure of [`Job::spawn_with`] gets the
//! [`CancelFlag`] to poll (the mesher and the iterative solver do) and a [`Progress`] to write. Cancelling a
//! job also makes [`Job::poll`] answer `Err("cancelled")` at once, so the screen is free again even when the
//! computation has no point at which it can stop; its late result is dropped.

use eframe::egui;
use kw_job::{CancelFlag, Progress};
use std::sync::mpsc::{channel, Receiver, TryRecvError};

pub struct Job<T> {
    rx: Receiver<Result<T, String>>,
    pub label: String,
    started: std::time::Instant,
    cancel: CancelFlag,
    progress: Progress,
}

impl<T: Send + 'static> Job<T> {
    pub fn spawn(label: &str, f: impl FnOnce() -> Result<T, String> + Send + 'static) -> Job<T> {
        Job::spawn_with(label, move |_, _| f())
    }

    /// As [`spawn`](Self::spawn), handing the closure the flag to poll and the progress to write.
    pub fn spawn_with(
        label: &str,
        f: impl FnOnce(&CancelFlag, &Progress) -> Result<T, String> + Send + 'static,
    ) -> Job<T> {
        let (tx, rx) = channel();
        let cancel = CancelFlag::new();
        let progress = Progress::new();
        let (c, p) = (cancel.clone(), progress.clone());
        std::thread::Builder::new()
            .name(format!("job: {label}"))
            .stack_size(64 << 20)
            .spawn(move || {
                let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| f(&c, &p)))
                    .unwrap_or_else(|_| Err("the computation panicked".into()));
                let _ = tx.send(r);
            })
            .expect("spawn job thread");
        Job {
            rx,
            label: label.into(),
            started: std::time::Instant::now(),
            cancel,
            progress,
        }
    }

    pub fn elapsed(&self) -> f64 {
        self.started.elapsed().as_secs_f64()
    }

    /// Ask the job to stop.
    pub fn cancel(&self) {
        self.cancel.cancel();
    }

    /// How far it has got (0 to 1), as far as it says.
    pub fn fraction(&self) -> f64 {
        self.progress.get()
    }

    /// `None` while running; the result once, then the job is spent. A cancelled job answers
    /// `Err("cancelled")` whatever it computed.
    pub fn poll(&self) -> Option<Result<T, String>> {
        if self.cancel.is_cancelled() {
            return Some(Err("cancelled".into()));
        }
        match self.rx.try_recv() {
            Ok(r) => Some(r),
            Err(TryRecvError::Empty) => None,
            Err(TryRecvError::Disconnected) => {
                Some(Err("the computation ended without a result".into()))
            }
        }
    }

    /// A progress bar with the time taken and a Cancel button; true when Cancel was clicked.
    pub fn ui(&self, ui: &mut egui::Ui) -> bool {
        let mut cancel = false;
        ui.horizontal(|ui| {
            let f = self.fraction() as f32;
            if f > 0.0 {
                ui.add(
                    egui::ProgressBar::new(f)
                        .desired_width(110.0)
                        .text(format!("{:.0} s", self.elapsed())),
                );
            } else {
                ui.spinner();
                ui.weak(format!("{:.0} s", self.elapsed()));
            }
            if ui.small_button("Cancel").clicked() {
                self.cancel();
                cancel = true;
            }
        });
        cancel
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn a_job_reports_progress_and_a_cancelled_job_answers_at_once() {
        let job = Job::spawn_with("count", |cancel, progress| {
            for i in 0..1000 {
                if cancel.is_cancelled() {
                    return Err("stopped".into());
                }
                progress.set(i as f64 / 1000.0);
                std::thread::sleep(Duration::from_millis(5));
            }
            Ok(7)
        });
        std::thread::sleep(Duration::from_millis(60));
        assert!(job.poll().is_none());
        assert!(job.fraction() > 0.0, "{}", job.fraction());
        job.cancel();
        assert_eq!(job.poll(), Some(Err("cancelled".to_string())));
    }

    #[test]
    fn a_plain_job_returns_its_value() {
        let job = Job::spawn("sum", || Ok(1 + 2));
        let r = loop {
            if let Some(r) = job.poll() {
                break r;
            }
            std::thread::sleep(Duration::from_millis(2));
        };
        assert_eq!(r, Ok(3));
        let bad: Job<u8> = Job::spawn("panics", || panic!("boom"));
        let r = loop {
            if let Some(r) = bad.poll() {
                break r;
            }
            std::thread::sleep(Duration::from_millis(2));
        };
        assert!(r.is_err());
    }
}
