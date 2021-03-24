use petgraph::visit::{IntoEdges, Visitable, GraphBase};
use std::hash::Hash;
use petgraph::algo::Measure;
use std::collections::{BinaryHeap, HashMap};
use std::collections::hash_map::Entry::{Occupied, Vacant};
use std::cmp::Ordering;
use petgraph::visit::VisitMap;
use petgraph::visit::EdgeRef;
use petgraph::graph::NodeIndex;

#[derive(Copy, Clone, Debug)]
pub struct MinScored<K, T>(pub K, pub T);

impl<K: PartialOrd, T> PartialEq for MinScored<K, T> {
    #[inline]
    fn eq(&self, other: &MinScored<K, T>) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

impl<K: PartialOrd, T> Eq for MinScored<K, T> {}

impl<K: PartialOrd, T> PartialOrd for MinScored<K, T> {
    #[inline]
    fn partial_cmp(&self, other: &MinScored<K, T>) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<K: PartialOrd, T> Ord for MinScored<K, T> {
    #[inline]
    fn cmp(&self, other: &MinScored<K, T>) -> Ordering {
        let a = &self.0;
        let b = &other.0;
        if a == b {
            Ordering::Equal
        } else if a < b {
            Ordering::Greater
        } else if a > b {
            Ordering::Less
        } else if a.ne(a) && b.ne(b) {
            // these are the NaN cases
            Ordering::Equal
        } else if a.ne(a) {
            // Order NaN less, so that it is last in the MinScore order
            Ordering::Less
        } else {
            Ordering::Greater
        }
    }
}

struct PathTracker<G>
    where
        G: GraphBase,
        G::NodeId: Eq + Hash,
{
    came_from: HashMap<G::NodeId, G::NodeId>,
}

impl<G> PathTracker<G>
    where
        G: GraphBase,
        G::NodeId: Eq + Hash,
{
    fn new() -> PathTracker<G> {
        PathTracker {
            came_from: HashMap::new(),
        }
    }

    fn set_predecessor(&mut self, node: G::NodeId, previous: G::NodeId) {
        self.came_from.insert(node, previous);
    }

    fn reconstruct_path_to(&self, last: G::NodeId) -> Vec<G::NodeId> {
        let mut path = vec![last];

        let mut current = last;
        while let Some(&previous) = self.came_from.get(&current) {
            path.push(previous);
            current = previous;
        }

        path.reverse();

        path
    }
}
pub fn astar_multiple_goals<G, F, H, K, IsGoal>(
    graph: G,
    start: G::NodeId,
    mut is_goal: IsGoal,
    mut edge_cost: F,
    mut estimate_cost: H
) -> HashMap<<G as GraphBase>::NodeId, Option<(K)>>
    where
        G: IntoEdges + Visitable,
        IsGoal: FnMut(G::NodeId) -> bool,
        G::NodeId: Eq + Hash,
        F: FnMut(G::EdgeRef) -> K,
        H: FnMut(G::NodeId) -> K,
        K: Measure + Copy,
{
    let mut visited = graph.visit_map();
    let mut visit_next = BinaryHeap::new();
    let mut scores = HashMap::new();
    let mut results = HashMap::new();

    let zero_score = K::default();
    scores.insert(start, zero_score);
    visit_next.push(MinScored(estimate_cost(start), start));

    while let Some(MinScored(_, node)) = visit_next.pop() {
        if is_goal(node) {
            let cost = scores[&node];
            results.insert(node, Some((cost)));
            // println!("Found result. Now we have {:?}", results.len());
        }

        // Don't visit the same node several times, as the first time it was visited it was using
        // the shortest available path.
        if !visited.visit(node) {
            continue;
        }

        // This lookup can be unwrapped without fear of panic since the node was necessarily scored
        // before adding him to `visit_next`.
        let node_score = scores[&node];

        for edge in graph.edges(node) {
            let next = edge.target();
            if visited.is_visited(&next) {
                continue;
            }

            let mut next_score = node_score + edge_cost(edge);

            match scores.entry(next) {
                Occupied(ent) => {
                    let old_score = *ent.get();
                    if next_score < old_score {
                        *ent.into_mut() = next_score;
                    } else {
                        next_score = old_score;
                    }
                }
                Vacant(ent) => {
                    ent.insert(next_score);
                }
            }

            let next_estimate_score = next_score + estimate_cost(next);
            visit_next.push(MinScored(next_estimate_score, next));
        }
    }

    results
}


pub fn dijkstra<G, F, K, IsGoal>(
    graph: G,
    start: G::NodeId,
    mut is_goal: IsGoal,
    mut edge_cost: F,
) -> HashMap<<G as GraphBase>::NodeId, Option<(K)>>
    where
        G: IntoEdges + Visitable,
        G::NodeId: Eq + Hash,
        IsGoal: FnMut(G::NodeId) -> bool,
        F: FnMut(G::EdgeRef) -> K,
        K: Measure + Copy,
{
    let mut visited = graph.visit_map();
    let mut scores = HashMap::new();
    let mut results = HashMap::new();

    let mut visit_next = BinaryHeap::new();
    let zero_score = K::default();
    scores.insert(start, zero_score);
    visit_next.push(MinScored(zero_score, start));
    while let Some(MinScored(node_score, node)) = visit_next.pop() {
        if visited.is_visited(&node) {
            continue;
        }
        if is_goal(node) {
            let cost = scores[&node];
            results.insert(node, Some((cost)));
            // println!("Found result. Now we have {:?}", results.len());
        }
        for edge in graph.edges(node) {
            let next = edge.target();
            if visited.is_visited(&next) {
                continue;
            }
            let next_score = node_score + edge_cost(edge);
            match scores.entry(next) {
                Occupied(ent) => {
                    if next_score < *ent.get() {
                        *ent.into_mut() = next_score;
                        visit_next.push(MinScored(next_score, next));
                        //predecessor.insert(next.clone(), node.clone());
                    }
                }
                Vacant(ent) => {
                    ent.insert(next_score);
                    visit_next.push(MinScored(next_score, next));
                    //predecessor.insert(next.clone(), node.clone());
                }
            }
        }
        visited.visit(node);
    }
    results
}