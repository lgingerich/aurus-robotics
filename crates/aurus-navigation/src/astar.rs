use crate::map::costmap::{Costmap2D, CellCost};
use std::collections::{BinaryHeap, HashMap};
use std::cmp::Ordering;

type Point = (usize, usize);

/// Calculates the Manhattan distance between two points
fn manhattan_distance(a: Point, b: Point) -> usize {
    a.0.abs_diff(b.0) + a.1.abs_diff(b.1)
}

/// Checks if a move is valid based on the costmap
fn is_valid_move(p: Point, costmap: &Costmap2D) -> bool {
    let (x, y) = p;
    if x >= costmap.width || y >= costmap.height {
        return false;
    }
    
    match costmap.get_cost(x, y) {
        Ok(cost) => match cost {
            CellCost::Free => true,
            CellCost::Inflated(_) => true,
            CellCost::Inscribed | CellCost::Lethal | CellCost::Unknown => false,
        },
        Err(_) => false,
    }
}

/// Gets the cost of moving to a cell
fn get_move_cost(p: Point, costmap: &Costmap2D) -> usize {
    let (x, y) = p;
    match costmap.get_cost(x, y) {
        Ok(cost) => match cost {
            CellCost::Free => 1,
            CellCost::Inflated(c) => 1 + c as usize, // Higher cost for inflated areas
            CellCost::Inscribed | CellCost::Lethal | CellCost::Unknown => usize::MAX,
        },
        Err(_) => usize::MAX,
    }
}

/// Gets valid neighboring cells
fn neighbors(p: Point, costmap: &Costmap2D) -> Vec<Point> {
    const DIRECTIONS: [(isize, isize); 4] = [(0, -1), (-1, 0), (1, 0), (0, 1)]; // Up, Left, Right, Down
    let mut neighbors_vec = Vec::new();
    let current_x_isize = p.0 as isize;
    let current_y_isize = p.1 as isize;

    for (dx, dy) in DIRECTIONS {
        let nx_isize = current_x_isize + dx;
        let ny_isize = current_y_isize + dy;

        if nx_isize >= 0 && ny_isize >= 0 {
            let neighbor_candidate = (nx_isize as usize, ny_isize as usize);
            if is_valid_move(neighbor_candidate, costmap) {
                neighbors_vec.push(neighbor_candidate);
            }
        }
    }
    neighbors_vec
}

#[derive(Copy, Clone, Eq, PartialEq)]
struct State {
    cost: usize,
    position: Point,
}

impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.cmp(&self.cost)
            .then_with(|| self.position.0.cmp(&other.position.0))
            .then_with(|| self.position.1.cmp(&other.position.1))
    }
}

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

/// Finds a path from start to goal using A* algorithm
/// 
/// # Arguments
/// * `costmap` - The costmap to plan in
/// * `start` - Starting point in grid coordinates
/// * `goal` - Goal point in grid coordinates
/// 
/// # Returns
/// * `Option<Vec<Point>>` - The path if found, None otherwise
pub fn astar_search(costmap: &Costmap2D, start: Point, goal: Point) -> Option<Vec<Point>> {
    if !is_valid_move(start, costmap) || !is_valid_move(goal, costmap) {
        return None;
    }

    let mut open_set = BinaryHeap::new();
    let mut came_from = HashMap::new();

    let mut g_score: HashMap<Point, usize> = HashMap::new();
    g_score.insert(start, 0);

    let mut f_score: HashMap<Point, usize> = HashMap::new();
    f_score.insert(start, manhattan_distance(start, goal));

    open_set.push(State { cost: f_score[&start], position: start });

    while let Some(State { cost: _current_f_score, position: current }) = open_set.pop() {
        if current == goal {
            return Some(reconstruct_path(&came_from, current));
        }

        for neighbor in neighbors(current, costmap) {
            let cost_to_neighbor = get_move_cost(neighbor, costmap);
            if cost_to_neighbor == usize::MAX { // Skip impassable neighbors
                continue;
            }
            let tentative_g_score = g_score[&current] + cost_to_neighbor;

            if tentative_g_score < *g_score.get(&neighbor).unwrap_or(&usize::MAX) {
                came_from.insert(neighbor, current);
                g_score.insert(neighbor, tentative_g_score);
                let h = manhattan_distance(neighbor, goal);
                f_score.insert(neighbor, tentative_g_score + h);
                open_set.push(State { cost: f_score[&neighbor], position: neighbor });
            }
        }
    }

    None
}

/// Finds a path from start to goal using A* algorithm with world coordinates
/// 
/// # Arguments
/// * `costmap` - The costmap to plan in
/// * `start_world` - Starting point in world coordinates (meters)
/// * `goal_world` - Goal point in world coordinates (meters)
/// 
/// # Returns
/// * `Option<Vec<(f32, f32)>>` - The path in world coordinates if found, None otherwise
pub fn astar_search_world(costmap: &Costmap2D, start_world: (f32, f32), goal_world: (f32, f32)) -> Option<Vec<(f32, f32)>> {
    // Convert world coordinates to grid coordinates
    let start_grid = costmap.world_to_grid(start_world.0, start_world.1)?;
    let goal_grid = costmap.world_to_grid(goal_world.0, goal_world.1)?;

    // Find path in grid coordinates
    let grid_path = astar_search(costmap, start_grid, goal_grid)?;

    // Convert path back to world coordinates
    let world_path: Vec<(f32, f32)> = grid_path.iter()
        .filter_map(|&(x, y)| costmap.grid_to_world(x, y))
        .collect();

    Some(world_path)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::map::costmap::Costmap2D; // Ensure Costmap2D is in scope for tests
    use crate::map::costmap::CellCost; // Ensure CellCost is in scope for tests

    #[test]
    fn test_astar_simple_path() {
        let mut costmap = Costmap2D::new(10, 10, 0.1, 0.0, 0.0).unwrap();
        
        // Create a simple obstacle pattern
        for x in 3..7 {
            for y in 3..7 {
                costmap.set_cost(x, y, CellCost::Lethal).unwrap();
            }
        }

        let start = (0, 0);
        let goal = (9, 9);

        let path = astar_search(&costmap, start, goal);
        assert!(path.is_some());
        
        if let Some(p) = &path {
            assert_eq!(p[0], start);
            assert_eq!(*p.last().unwrap(), goal);
        }
    }

    #[test]
    fn test_astar_world_coordinates() {
        let mut costmap = Costmap2D::new(10, 10, 0.1, -0.5, -0.5).unwrap();
        
        // Create a simple obstacle pattern (e.g., a box in the middle)
        // Obstacle from grid (3,3) to (6,6)
        for x in 3..=6 { // Grid coordinates
            for y in 3..=6 { // Grid coordinates
                costmap.set_cost(x, y, CellCost::Lethal).unwrap();
            }
        }

        // Start and goal in world coordinates, ensuring they are not on obstacles
        // And are reasonably far apart to require pathfinding around the obstacle.
        let start_world = (-0.45, -0.45); // Corresponds to grid (0,0) approx.
        let goal_world = (0.45, 0.45);   // Corresponds to grid (9,9) approx.

        let path = astar_search_world(&costmap, start_world, goal_world);
        assert!(path.is_some(), "Path should be found");
        
        if let Some(p) = &path {
            assert!(!p.is_empty(), "Path should not be empty");
            // Check start point (approximate due to float conversions)
            assert!((p[0].0 - start_world.0).abs() < costmap.resolution * 1.5, "Start X mismatch"); // Increased tolerance
            assert!((p[0].1 - start_world.1).abs() < costmap.resolution * 1.5, "Start Y mismatch"); // Increased tolerance
            
            // Check goal point (approximate)
            let last_point = p.last().unwrap();
            assert!((last_point.0 - goal_world.0).abs() < costmap.resolution * 1.5, "Goal X mismatch"); // Increased tolerance
            assert!((last_point.1 - goal_world.1).abs() < costmap.resolution * 1.5, "Goal Y mismatch"); // Increased tolerance

            // Optional: Verify that the path does not go through lethal cells
            for point_world in p {
                if let Some((gx, gy)) = costmap.world_to_grid(point_world.0, point_world.1) {
                    let cell_cost = costmap.get_cost(gx, gy).unwrap_or(&CellCost::Unknown);
                    assert_ne!(*cell_cost, CellCost::Lethal, "Path goes through a lethal cell at grid ({}, {}) from world ({:.2}, {:.2})", gx, gy, point_world.0, point_world.1);
                } else {
                    // This case should ideally not happen if path points are derived from grid_to_world
                    panic!("Path point ({:.2}, {:.2}) is outside map bounds", point_world.0, point_world.1);
                }
            }
        }
    }

    #[test]
    fn test_astar_no_path() {
        let mut costmap = Costmap2D::new(5, 5, 0.1, 0.0, 0.0).unwrap();
        
        // Create a wall blocking the path
        for y in 0..5 {
            costmap.set_cost(2, y, CellCost::Lethal).unwrap();
        }

        let start = (0, 0);
        let goal = (4, 4);

        let path = astar_search(&costmap, start, goal);
        assert!(path.is_none());
    }
}


