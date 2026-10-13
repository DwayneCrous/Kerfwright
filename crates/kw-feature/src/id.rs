//! Stable identity of a feature inside a document. Unlike a graph index it survives saving, loading,
//! reordering and removal of other features.

use serde::{Deserialize, Serialize};
use std::fmt;

#[derive(Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct FeatureId(pub u64);

impl fmt::Debug for FeatureId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "F{}", self.0)
    }
}

impl fmt::Display for FeatureId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "F{}", self.0)
    }
}

/// 128-bit FNV-1a: a fast, stable, non-cryptographic content hash (cache keys and change detection).
#[derive(Clone)]
pub struct Hasher128(u128);

impl Default for Hasher128 {
    fn default() -> Self {
        Hasher128(0x6c62272e07bb014262b821756295c58d)
    }
}

impl Hasher128 {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn bytes(&mut self, bytes: &[u8]) -> &mut Self {
        for b in bytes {
            self.0 ^= *b as u128;
            self.0 = self.0.wrapping_mul(0x0000000001000000000000000000013b);
        }
        self
    }
    pub fn u64(&mut self, v: u64) -> &mut Self {
        self.bytes(&v.to_le_bytes())
    }
    pub fn u128(&mut self, v: u128) -> &mut Self {
        self.bytes(&v.to_le_bytes())
    }
    pub fn str(&mut self, s: &str) -> &mut Self {
        self.u64(s.len() as u64).bytes(s.as_bytes())
    }
    /// A float quantised to `1e-7` so that rounding noise does not change the hash.
    pub fn f64(&mut self, v: f64) -> &mut Self {
        let q = (v * 1e7).round();
        let q = if q == 0.0 { 0.0 } else { q }; // -0 → 0
        self.bytes(&q.to_le_bytes())
    }
    pub fn finish(&self) -> u128 {
        // final avalanche so that short inputs differ in all bits
        let mut x = self.0;
        x ^= x >> 67;
        x = x.wrapping_mul(0x9e3779b97f4a7c15f39cc0605cedc835);
        x ^= x >> 59;
        x
    }
}

/// Hash of anything serialisable, through its canonical CBOR encoding.
pub fn hash_serde<T: Serialize>(v: &T) -> u128 {
    let mut buf = Vec::with_capacity(256);
    let _ = ciborium::into_writer(v, &mut buf);
    let mut h = Hasher128::new();
    h.bytes(&buf);
    h.finish()
}
