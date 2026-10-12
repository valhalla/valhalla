use dary_heap::DaryHeap;

/// A simple priority queue based on a 4-ary min-heap.
pub struct PriorityQueue<Cost: Ord, Item>(DaryHeap<Entry<Cost, Item>, 4>);

impl<Cost, Item> PriorityQueue<Cost, Item>
where
    Cost: Ord,
{
    pub fn new() -> Self {
        Self(DaryHeap::new())
    }

    pub fn push(&mut self, cost: Cost, item: Item) {
        self.0.push(Entry(cost, item));
    }

    pub fn pop(&mut self) -> Option<(Cost, Item)> {
        self.0.pop().map(|entry| (entry.0, entry.1))
    }
}

struct Entry<Cost, Item>(Cost, Item);

impl<Cost, Item> PartialEq for Entry<Cost, Item>
where
    Cost: PartialEq,
{
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

impl<Cost, Item> Eq for Entry<Cost, Item> where Cost: Eq {}

impl<Cost, Item> PartialOrd for Entry<Cost, Item>
where
    Cost: Ord,
{
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl<Cost, Item> Ord for Entry<Cost, Item>
where
    Cost: Ord,
{
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // Reverse order to make heap a min-heap
        other.0.cmp(&self.0)
    }
}

#[cfg(test)]
mod tests {
    use super::PriorityQueue;

    #[test]
    fn pops_in_ascending_cost_order() {
        let mut pq = PriorityQueue::new();
        assert_eq!(pq.pop(), None);

        pq.push(5, "five");
        pq.push(2, "two");
        pq.push(8, "eight");
        pq.push(1, "one");
        assert_eq!(pq.pop(), Some((1, "one")));
        assert_eq!(pq.pop(), Some((2, "two")));
        pq.push(3, "three");
        assert_eq!(pq.pop(), Some((3, "three")));
        assert_eq!(pq.pop(), Some((5, "five")));
        assert_eq!(pq.pop(), Some((8, "eight")));
        assert_eq!(pq.pop(), None);
    }
}
