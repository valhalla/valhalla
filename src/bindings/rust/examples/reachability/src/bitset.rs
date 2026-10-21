/// Fixed-size bitset over the range [0, size), fixed at creation.
/// Fits here because `GraphId::id()` is a dense index into its own tile's node array.
pub struct BitSet(Box<[usize]>);

impl BitSet {
    /// Creates a bitset for indices in range [0, size), all initially unset.
    pub fn new(size: usize) -> Self {
        let word_count = size.div_ceil(usize::BITS as usize);
        Self(vec![0; word_count].into_boxed_slice())
    }

    /// Adds an index and returns true if it wasn't present. Out-of-bounds indices are ignored.
    pub fn insert(&mut self, index: usize) -> bool {
        let word_idx = index / usize::BITS as usize;
        if word_idx < self.0.len() {
            let bit_idx = index % usize::BITS as usize;
            let mask = 1usize << bit_idx;
            let word = &mut self.0[word_idx];
            let was_set = *word & mask != 0;
            *word |= mask;
            !was_set
        } else {
            false // out of bounds
        }
    }

    /// Returns true if the index is in the set. Out-of-bounds indices are treated as absent.
    pub fn contains(&self, index: usize) -> bool {
        let word_idx = index / usize::BITS as usize;
        self.0.get(word_idx).is_some_and(|word| {
            let bit_idx = index % usize::BITS as usize;
            word & (1usize << bit_idx) != 0
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn insert_and_contains() {
        let mut bitset = BitSet::new(126);
        assert!(!bitset.contains(0));
        assert!(!bitset.contains(64));
        assert!(!bitset.contains(125));
        assert!(bitset.insert(0));
        assert!(bitset.insert(64));
        assert!(bitset.insert(125));
        assert!(bitset.contains(0));
        assert!(bitset.contains(64));
        assert!(bitset.contains(125));
        assert!(!bitset.insert(0));
        assert!(!bitset.insert(64));
        assert!(!bitset.insert(125));
    }

    #[test]
    fn rounds_up_to_whole_words() {
        let mut bitset = BitSet::new(126);
        // Within the allocated words, so capacity is effectively 128.
        assert!(!bitset.contains(127));
        assert!(bitset.insert(127));
        assert!(bitset.contains(127));
        assert!(!bitset.insert(127));
    }

    #[test]
    fn out_of_bounds_is_ignored() {
        let mut bitset = BitSet::new(126);
        assert!(!bitset.contains(137));
        assert!(!bitset.contains(230));
        assert!(!bitset.insert(137));
        assert!(!bitset.insert(230));
        assert!(!bitset.contains(230));
    }
}
