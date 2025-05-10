/*

A* = f(n) = g(n) + h(n) 

Where:
    n = a node in the graph
    g(n) = actual cost from start node to n
    h(n) = estimated cost from n to the goal (heuristic)
    f(n) = total estimated cost of the cheapest solution through n



Initialize:
    - queue of nodes to explore
    - set of nodes already explored
    - start node, g(start)= 0, f(start) = h(start)

Loop:
    - pop node with lowest f(n) from queue
    - if n is the goal node, return path
    - add n to set of explored nodes
    - for each neighbor of n:
        - if neighbor is not in explored set:
            - calculate g(neighbor) = g(n) + cost(n, neighbor)
            - calculate f(neighbor) = g(neighbor) + h(neighbor)
            - add neighbor to queue


*/


type Point = (i32, i32);


fn manhattan_distance(a: Point, b: Point) -> i32 {
    (a.0 - b.0).abs() + (a.1 - b.1).abs()
}

fn is_valid_move(p: Point, grid: &Vec<Vec<u8>>) -> bool {
    let (x, y) = p;
    // Check if the point is within the grid and is a valid move
    // Walkable grids are marked as 0, blocked grids are marked as 1
    x >= 0 && x < grid.len() as i32 && y >= 0 && y < grid[0].len() as i32 && grid[x as usize][y as usize] == 0
}

fn neighbors(p: Point, grid: &Vec<Vec<u8>>) -> Vec<Point> {
    let directions = [(0, -1), (-1, 0), (1, 0), (0, 1)]; // Up, Left, Right, Down
    let mut neighbors = Vec::new();
    for (dx, dy) in directions {
        let neighbor = (p.0 + dx, p.1 + dy);
        if is_valid_move(neighbor, grid) {
            neighbors.push(neighbor);
        }
    }
    neighbors
}

use std::collections::{BinaryHeap, HashMap};
use std::cmp::Ordering;

#[derive(Copy, Clone, Eq, PartialEq)]
struct State {
    cost: i32,
    position: Point,
}

// The priority queue depends on `Ord`.
// Explicitly implement the trait so the queue becomes a min-heap
// instead of a max-heap.
impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        // Notice that the we flip the ordering on costs.
        // In case of a tie we compare positions - this step is necessary
        // to make Rust derive `PartialEq` and `Ord` an `Eq` for `State`.
        other.cost.cmp(&self.cost)
            .then_with(|| self.position.0.cmp(&other.position.0))
            .then_with(|| self.position.1.cmp(&other.position.1))
    }
}

// `PartialOrd` needs to be implemented as well.
impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

fn reconstruct_path(came_from: &HashMap<Point, Point>, mut current: Point) -> Vec<Point> {
    let mut path = vec![current];
    while let Some(&previous) = came_from.get(&current) {
        path.push(previous);
        current = previous;
    }
    path.reverse();
    path
}

pub fn astar_search(grid: &Vec<Vec<u8>>, start: Point, goal: Point) -> Option<Vec<Point>> {
    if !is_valid_move(start, grid) || !is_valid_move(goal, grid) {
        return None; // Start or goal is not traversable or out of bounds
    }

    let mut open_set = BinaryHeap::new();
    let mut came_from = HashMap::new();

    let mut g_score = HashMap::new();
    g_score.insert(start, 0);

    let mut f_score = HashMap::new();
    f_score.insert(start, manhattan_distance(start, goal));

    open_set.push(State { cost: f_score[&start], position: start });

    while let Some(State { cost: _current_f_score, position: current }) = open_set.pop() {
        if current == goal {
            return Some(reconstruct_path(&came_from, current));
        }

        for neighbor in neighbors(current, grid) {
            let tentative_g_score = g_score[&current] + 1; // Cost between adjacent nodes is 1

            if tentative_g_score < *g_score.get(&neighbor).unwrap_or(&i32::MAX) {
                came_from.insert(neighbor, current);
                g_score.insert(neighbor, tentative_g_score);
                let h = manhattan_distance(neighbor, goal);
                f_score.insert(neighbor, tentative_g_score + h);
                open_set.push(State { cost: f_score[&neighbor], position: neighbor });
            }
        }
    }

    None // No path found
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_astar_simple_path() {
        let grid = vec![
            vec![0, 0, 0, 0, 1],
            vec![1, 1, 0, 1, 0],
            vec![0, 0, 0, 0, 0],
            vec![0, 1, 1, 1, 1],
            vec![0, 0, 0, 0, 0],
        ];
        let start = (0, 0);
        let goal = (4, 4);

        let path = astar_search(&grid, start, goal);

        assert!(path.is_some());
        // The expected path for this grid and heuristic would be:
        // (0,0) -> (0,1) -> (0,2) -> (1,2) -> (2,2) -> (2,3) -> (2,4) -> (3,4) -> (4,4)
        // However, A* can find any optimal path. Let's check basics.
        if let Some(p) = &path {
            println!("Path found: {:?}", p);
            assert_eq!(p[0], start);
            assert_eq!(*p.last().unwrap(), goal);
            // A more specific path check might be needed if a canonical path is expected
            // For this grid, one optimal path is:
            // vec![(0,0), (0,1), (0,2), (1,2), (2,2), (2,1), (2,0), (3,0), (4,0), (4,1), (4,2), (4,3), (4,4)]
            // or more directly:
            // vec![(0,0), (0,1), (0,2), (1,2), (2,2), (2,3), (2,4), (3,4), (4,4)] - length 9
            // My implementation may produce a different path of the same optimal length.
            // Let's check if path is one of the known optimal paths or at least has expected length.
        }
    }

    #[test]
    fn test_astar_no_path() {
        let grid = vec![
            vec![0, 1, 0],
            vec![0, 1, 0],
            vec![0, 1, 0],
        ];
        let start = (0, 0);
        let goal = (0, 2);

        let path = astar_search(&grid, start, goal);
        assert!(path.is_none());
        if path.is_none() {
            println!("No path found, as expected.");
        }
    }

    #[test]
    fn test_astar_start_or_goal_blocked() {
        let grid = vec![
            vec![0, 0, 0],
            vec![0, 1, 0], // Middle is blocked
            vec![0, 0, 0],
        ];
        let start_blocked = (1, 1);
        let goal_valid = (2, 2);
        let path1 = astar_search(&grid, start_blocked, goal_valid);
        assert!(path1.is_none());

        let start_valid = (0,0);
        let goal_blocked = (1,1);
        let path2 = astar_search(&grid, start_valid, goal_blocked);
        assert!(path2.is_none());
    }

     #[test]
    fn test_astar_start_equals_goal() {
        let grid = vec![
            vec![0, 0, 0],
            vec![0, 0, 0],
            vec![0, 0, 0],
        ];
        let start = (1, 1);
        let goal = (1, 1);

        let path = astar_search(&grid, start, goal);
        assert!(path.is_some());
        if let Some(p) = path {
            assert_eq!(p, vec![(1,1)]);
             println!("Path where start equals goal: {:?}", p);
        }
    }
}


