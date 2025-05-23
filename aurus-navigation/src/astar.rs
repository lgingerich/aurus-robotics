use crate::map::costmap::{Costmap2D, CellCost};
use crate::map::{GridPoint, WorldPoint};
use std::collections::{BinaryHeap, HashMap};
use std::cmp::Ordering;

/// Calculates the Manhattan distance between two grid points.
fn manhattan_distance(a: GridPoint, b: GridPoint) -> usize {
    a.x.abs_diff(b.x) + a.y.abs_diff(b.y)
}

/// Checks if a move to a grid point is valid based on the costmap.
fn is_valid_move(p: GridPoint, costmap: &Costmap2D) -> bool {
    // costmap.get_cost already checks bounds
    match costmap.get_cost(p) {
        Ok(cost) => match cost {
            CellCost::Free | CellCost::Inflated(_) => true,
            CellCost::Inscribed | CellCost::Lethal | CellCost::Unknown => false,
        },
        Err(_) => false, // Out of bounds or other error
    }
}

/// Gets the cost of moving to a grid cell.
fn get_move_cost(p: GridPoint, costmap: &Costmap2D) -> usize {
    match costmap.get_cost(p) {
        Ok(cost) => match cost {
            CellCost::Free => 1,
            CellCost::Inflated(c) => 1 + *c as usize, // Dereference c before casting
            CellCost::Inscribed | CellCost::Lethal | CellCost::Unknown => usize::MAX,
        },
        Err(_) => usize::MAX, // Out of bounds or other error
    }
}

/// Gets valid neighboring grid cells.
fn neighbors(p: GridPoint, costmap: &Costmap2D) -> Vec<GridPoint> {
    const DIRECTIONS: [(isize, isize); 4] = [(0, -1), (-1, 0), (1, 0), (0, 1)]; // Up, Left, Right, Down
    let mut neighbors_vec = Vec::new();
    let current_x_isize = p.x as isize;
    let current_y_isize = p.y as isize;

    for (dx, dy) in DIRECTIONS {
        let nx_isize = current_x_isize + dx;
        let ny_isize = current_y_isize + dy;

        if nx_isize >= 0 && ny_isize >= 0 {
            let neighbor_candidate = GridPoint::new(nx_isize as usize, ny_isize as usize);
            // No need to check bounds here, is_valid_move will handle it via costmap.get_cost
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
    position: GridPoint, // Use GridPoint
}

impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.cmp(&self.cost)
            .then_with(|| self.position.x.cmp(&other.position.x)) // Compare GridPoint fields
            .then_with(|| self.position.y.cmp(&other.position.y))
    }
}

impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Reconstructs the path from a map of `came_from` links.
fn reconstruct_path(came_from: &HashMap<GridPoint, GridPoint>, mut current: GridPoint) -> Vec<GridPoint> {
    let mut path = vec![current];
    while let Some(&previous) = came_from.get(&current) {
        path.push(previous);
        current = previous;
    }
    path.reverse();
    path
}

/// Finds a path from a start grid point to a goal grid point using the A* algorithm.
/// 
/// # Arguments
/// * `costmap` - The costmap to plan in.
/// * `start` - Starting point in grid coordinates.
/// * `goal` - Goal point in grid coordinates.
/// 
/// # Returns
/// * `Option<Vec<GridPoint>>` - The path if found, `None` otherwise.
pub fn astar_search(costmap: &Costmap2D, start: GridPoint, goal: GridPoint) -> Option<Vec<GridPoint>> {
    if !is_valid_move(start, costmap) || !is_valid_move(goal, costmap) {
        return None;
    }

    let mut open_set = BinaryHeap::new();
    let mut came_from: HashMap<GridPoint, GridPoint> = HashMap::new();

    let mut g_score: HashMap<GridPoint, usize> = HashMap::new();
    g_score.insert(start, 0);

    let mut f_score: HashMap<GridPoint, usize> = HashMap::new();
    f_score.insert(start, manhattan_distance(start, goal));

    open_set.push(State { cost: f_score[&start], position: start });

    while let Some(State { position: current, .. }) = open_set.pop() {
        if current == goal {
            return Some(reconstruct_path(&came_from, current));
        }

        for neighbor in neighbors(current, costmap) {
            let cost_to_neighbor = get_move_cost(neighbor, costmap);
            if cost_to_neighbor == usize::MAX { // Skip impassable neighbors
                continue;
            }
            let tentative_g_score = g_score.get(&current).unwrap_or(&0) + cost_to_neighbor;

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

/// Finds a path from a start world point to a goal world point using A* algorithm.
/// 
/// Coordinates are converted to grid coordinates for pathfinding, then back to world coordinates.
/// # Arguments
/// * `costmap` - The costmap to plan in.
/// * `start_world` - Starting point in world coordinates (meters).
/// * `goal_world` - Goal point in world coordinates (meters).
/// 
/// # Returns
/// * `Option<Vec<WorldPoint>>` - The path in world coordinates if found, `None` otherwise.
pub fn astar_search_world(costmap: &Costmap2D, start_world: WorldPoint, goal_world: WorldPoint) -> Option<Vec<WorldPoint>> {
    // Convert world coordinates to grid coordinates
    let start_grid = costmap.world_to_grid(start_world)?;
    let goal_grid = costmap.world_to_grid(goal_world)?;

    // Find path in grid coordinates
    let grid_path = astar_search(costmap, start_grid, goal_grid)?;

    // Convert path back to world coordinates
    // The path consists of the centers of the grid cells.
    let world_path: Vec<WorldPoint> = grid_path.iter()
        .filter_map(|&grid_p| costmap.grid_to_world(grid_p))
        .collect();

    if world_path.len() != grid_path.len() {
        // This might happen if a grid point in the path somehow becomes invalid for grid_to_world conversion,
        // though it shouldn't if astar_search only returns valid points within the costmap.
        return None; 
    }

    Some(world_path)
}

#[cfg(test)]
mod tests {
    use super::*;
    // Ensure Costmap2D, CellCost, GridPoint, WorldPoint are in scope for tests
    use crate::map::costmap::{Costmap2D, CellCost};
    use crate::map::{GridPoint, WorldPoint};

    #[test]
    fn test_astar_simple_path() {
        let origin = WorldPoint::new(0.0, 0.0);
        let mut costmap = Costmap2D::new(10, 10, 0.1, origin).unwrap();
        
        // Create a simple obstacle pattern (a box in the middle)
        for x in 3..7 {
            for y in 3..7 {
                costmap.set_cost(GridPoint::new(x, y), CellCost::Lethal).unwrap();
            }
        }

        let start = GridPoint::new(0, 0);
        let goal = GridPoint::new(9, 9);

        let path = astar_search(&costmap, start, goal);
        assert!(path.is_some(), "Path should be found for simple case");
        
        if let Some(p) = &path {
            assert_eq!(p[0], start, "Path start mismatch");
            assert_eq!(*p.last().unwrap(), goal, "Path goal mismatch");
        }
    }

    #[test]
    fn test_astar_world_coordinates() {
        let origin = WorldPoint::new(-0.5, -0.5);
        let mut costmap = Costmap2D::new(10, 10, 0.1, origin).unwrap();
        
        for x in 3..=6 {
            for y in 3..=6 {
                costmap.set_cost(GridPoint::new(x, y), CellCost::Lethal).unwrap();
            }
        }

        let start_world = WorldPoint::new(-0.45, -0.45); // Approx center of grid (0,0)
        let goal_world = WorldPoint::new(0.45, 0.45);   // Approx center of grid (9,9)

        let path = astar_search_world(&costmap, start_world, goal_world);
        assert!(path.is_some(), "Path should be found for world coords");
        
        if let Some(p) = &path {
            assert!(!p.is_empty(), "Path should not be empty");
            // Check start point (approximate due to float conversions and cell-center mapping)
            let expected_start_grid = costmap.world_to_grid(start_world).unwrap();
            let path_start_grid = costmap.world_to_grid(p[0]).unwrap();
            assert_eq!(path_start_grid, expected_start_grid, "Path start world point maps to wrong grid cell");

            // Check goal point (approximate)
            let expected_goal_grid = costmap.world_to_grid(goal_world).unwrap();
            let path_goal_grid = costmap.world_to_grid(*p.last().unwrap()).unwrap();
            assert_eq!(path_goal_grid, expected_goal_grid, "Path goal world point maps to wrong grid cell");

            for point_world in p {
                if let Some(grid_cell) = costmap.world_to_grid(*point_world) {
                    let cell_cost = costmap.get_cost(grid_cell).unwrap_or(&CellCost::Unknown);
                    assert_ne!(*cell_cost, CellCost::Lethal, 
                        "Path goes through a lethal cell at grid ({}, {}) from world ({:.2}, {:.2})", 
                        grid_cell.x, grid_cell.y, point_world.x, point_world.y);
                } else {
                    panic!("Path point ({:.2}, {:.2}) is outside map bounds or invalid", point_world.x, point_world.y);
                }
            }
        }
    }

    #[test]
    fn test_astar_no_path() {
        let origin = WorldPoint::new(0.0, 0.0);
        let mut costmap = Costmap2D::new(5, 5, 0.1, origin).unwrap();
        
        // Create a wall blocking the path
        for y_coord in 0..5 {
            costmap.set_cost(GridPoint::new(2, y_coord), CellCost::Lethal).unwrap();
        }

        let start = GridPoint::new(0, 0);
        let goal = GridPoint::new(4, 4);

        let path = astar_search(&costmap, start, goal);
        assert!(path.is_none(), "Path should not be found when blocked");
    }
}


