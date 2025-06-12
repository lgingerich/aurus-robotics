use crate::map::costmap::{CellCost, CostMap};
use aurus_common::{GridPoint, WorldPoint};
use nalgebra::SVector;

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};
use std::fmt;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Represents the result of an A* pathfinding operation with metadata.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PathResult<T, const D: usize> {
    /// The computed path, if one was found.
    pub path: Option<Vec<T>>,
    /// The total cost of the path.
    pub total_cost: Option<usize>,
    /// The number of nodes explored during the search.
    pub nodes_explored: usize,
    /// The length of the path (number of waypoints).
    pub path_length: usize,
}

impl<T, const D: usize> PathResult<T, D> {
    /// Creates a new PathResult for a successful path.
    pub fn success(path: Vec<T>, total_cost: usize, nodes_explored: usize) -> Self {
        let path_length = path.len();
        Self {
            path: Some(path),
            total_cost: Some(total_cost),
            nodes_explored,
            path_length,
        }
    }

    /// Creates a new PathResult for a failed path search.
    pub fn failure(nodes_explored: usize) -> Self {
        Self {
            path: None,
            total_cost: None,
            nodes_explored,
            path_length: 0,
        }
    }

    /// Returns true if a path was found.
    pub fn is_success(&self) -> bool {
        self.path.is_some()
    }

    /// Returns the path if one was found.
    pub fn into_path(self) -> Option<Vec<T>> {
        self.path
    }
}

impl<T: fmt::Debug, const D: usize> fmt::Display for PathResult<T, D> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match &self.path {
            Some(_) => write!(
                f,
                "PathResult {{ success: true, path_length: {}, total_cost: {}, nodes_explored: {} }}",
                self.path_length,
                self.total_cost.unwrap_or(0),
                self.nodes_explored
            ),
            None => write!(
                f,
                "PathResult {{ success: false, nodes_explored: {} }}",
                self.nodes_explored
            ),
        }
    }
}

/// Calculates the Manhattan distance between two grid points.
pub fn manhattan_distance<const D: usize>(a: &SVector<usize, D>, b: &SVector<usize, D>) -> usize {
    // element-wise |x-y|, then Σ
    a.zip_map(b, |x, y| x.abs_diff(y)).sum()
}

/// Checks if a move to a grid point is valid based on the costmap.
fn is_valid_move<const D: usize>(p: &SVector<usize, D>, costmap: &CostMap<D>) -> bool {
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
fn get_move_cost<const D: usize>(p: &SVector<usize, D>, costmap: &CostMap<D>) -> usize {
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
fn neighbors<const D: usize>(
    p: &SVector<usize, D>,
    costmap: &CostMap<D>,
) -> Vec<SVector<usize, D>> {
    let mut neighbors_vec = Vec::new();

    // Generate all possible directions (2*D directions for D-dimensional space)
    // For each dimension, we can move +1 or -1
    for dim in 0..D {
        // Move in positive direction
        let mut neighbor_pos = *p;
        if neighbor_pos[dim] < usize::MAX {
            neighbor_pos[dim] += 1;
            if is_valid_move(&neighbor_pos, costmap) {
                neighbors_vec.push(neighbor_pos);
            }
        }

        // Move in negative direction
        let mut neighbor_neg = *p;
        if neighbor_neg[dim] > 0 {
            neighbor_neg[dim] -= 1;
            if is_valid_move(&neighbor_neg, costmap) {
                neighbors_vec.push(neighbor_neg);
            }
        }
    }

    neighbors_vec
}

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
struct State<const D: usize> {
    cost: usize,
    position: SVector<usize, D>,
}

impl<const D: usize> Ord for State<D> {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.cmp(&self.cost).then_with(|| {
            // Compare positions lexicographically
            for i in 0..D {
                match self.position[i].cmp(&other.position[i]) {
                    Ordering::Equal => continue,
                    other => return other,
                }
            }
            Ordering::Equal
        })
    }
}

impl<const D: usize> PartialOrd for State<D> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<const D: usize> fmt::Display for State<D> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "State {{ cost: {}, position: [", self.cost)?;
        for (i, coord) in self.position.iter().enumerate() {
            if i > 0 {
                write!(f, ", ")?;
            }
            write!(f, "{}", coord)?;
        }
        write!(f, "] }}")
    }
}

/// Reconstructs the path from a map of `came_from` links.
fn reconstruct_path<const D: usize>(
    came_from: &HashMap<SVector<usize, D>, SVector<usize, D>>,
    mut current: SVector<usize, D>,
) -> Vec<SVector<usize, D>> {
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
/// * `Option<Vec<SVector<usize, D>>>` - The path if found, `None` otherwise.
pub fn astar_search<const D: usize>(
    costmap: &CostMap<D>,
    start: &SVector<usize, D>,
    goal: &SVector<usize, D>,
) -> Option<Vec<SVector<usize, D>>> {
    let result = astar_search_detailed(costmap, start, goal);
    result.into_path()
}

/// Finds a path from a start grid point to a goal grid point using the A* algorithm with detailed results.
///
/// # Arguments
/// * `costmap` - The costmap to plan in.
/// * `start` - Starting point in grid coordinates.
/// * `goal` - Goal point in grid coordinates.
///
/// # Returns
/// * `PathResult<SVector<usize, D>, D>` - Detailed pathfinding result with metadata.
pub fn astar_search_detailed<const D: usize>(
    costmap: &CostMap<D>,
    start: &SVector<usize, D>,
    goal: &SVector<usize, D>,
) -> PathResult<SVector<usize, D>, D> {
    let mut nodes_explored = 0;

    if !is_valid_move(start, costmap) || !is_valid_move(goal, costmap) {
        return PathResult::failure(nodes_explored);
    }

    let mut open_set = BinaryHeap::new();
    let mut came_from: HashMap<SVector<usize, D>, SVector<usize, D>> = HashMap::new();

    let mut g_score: HashMap<SVector<usize, D>, usize> = HashMap::new();
    g_score.insert(*start, 0);

    let mut f_score: HashMap<SVector<usize, D>, usize> = HashMap::new();
    f_score.insert(*start, manhattan_distance(start, goal));

    open_set.push(State {
        cost: f_score[start],
        position: *start,
    });

    while let Some(State {
        position: current, ..
    }) = open_set.pop()
    {
        nodes_explored += 1;

        if current == *goal {
            let path = reconstruct_path(&came_from, current);
            let total_cost = *g_score.get(&current).unwrap_or(&0);
            return PathResult::success(path, total_cost, nodes_explored);
        }

        for neighbor in neighbors(&current, costmap) {
            let cost_to_neighbor = get_move_cost(&neighbor, costmap);
            if cost_to_neighbor == usize::MAX {
                // Skip impassable neighbors
                continue;
            }
            let tentative_g_score = g_score.get(&current).unwrap_or(&0) + cost_to_neighbor;

            if tentative_g_score < *g_score.get(&neighbor).unwrap_or(&usize::MAX) {
                came_from.insert(neighbor, current);
                g_score.insert(neighbor, tentative_g_score);
                let h = manhattan_distance(&neighbor, goal);
                f_score.insert(neighbor, tentative_g_score + h);
                open_set.push(State {
                    cost: f_score[&neighbor],
                    position: neighbor,
                });
            }
        }
    }

    PathResult::failure(nodes_explored)
}

/// Converts a GridPoint to SVector for use with CostMap API.
fn grid_point_to_svector<const D: usize>(point: GridPoint<D>) -> SVector<usize, D> {
    let mut svector = SVector::<usize, D>::zeros();
    for i in 0..D {
        svector[i] = point.coords[i];
    }
    svector
}

/// Converts an SVector to GridPoint.
fn svector_to_grid_point<const D: usize>(svector: &SVector<usize, D>) -> GridPoint<D> {
    let mut coords = [0usize; D];
    for i in 0..D {
        coords[i] = svector[i];
    }
    GridPoint::from(coords)
}

/// Converts a WorldPoint to SVector for use with CostMap API.
fn world_point_to_svector<const D: usize>(point: WorldPoint<D>) -> SVector<f32, D> {
    let mut svector = SVector::<f32, D>::zeros();
    for i in 0..D {
        svector[i] = point.coords[i];
    }
    svector
}

/// Converts an SVector to WorldPoint.
fn svector_to_world_point<const D: usize>(svector: &SVector<f32, D>) -> WorldPoint<D> {
    let mut coords = [0.0f32; D];
    for i in 0..D {
        coords[i] = svector[i];
    }
    WorldPoint::from(coords)
}

/// Finds a path from a start grid point to a goal grid point using A* algorithm.
/// This is a convenience wrapper that handles the conversion between GridPoint and SVector.
///
/// # Arguments
/// * `costmap` - The costmap to plan in.
/// * `start` - Starting point in grid coordinates.
/// * `goal` - Goal point in grid coordinates.
///
/// # Returns
/// * `Option<Vec<GridPoint<D>>>` - The path if found, `None` otherwise.
pub fn astar_search_grid<const D: usize>(
    costmap: &CostMap<D>,
    start: GridPoint<D>,
    goal: GridPoint<D>,
) -> Option<Vec<GridPoint<D>>> {
    let result = astar_search_grid_detailed(costmap, start, goal);
    result.into_path()
}

/// Finds a path from a start grid point to a goal grid point using A* algorithm with detailed results.
/// This is a convenience wrapper that handles the conversion between GridPoint and SVector.
///
/// # Arguments
/// * `costmap` - The costmap to plan in.
/// * `start` - Starting point in grid coordinates.
/// * `goal` - Goal point in grid coordinates.
///
/// # Returns
/// * `PathResult<GridPoint<D>, D>` - Detailed pathfinding result with metadata.
pub fn astar_search_grid_detailed<const D: usize>(
    costmap: &CostMap<D>,
    start: GridPoint<D>,
    goal: GridPoint<D>,
) -> PathResult<GridPoint<D>, D> {
    let start_svector = grid_point_to_svector(start);
    let goal_svector = grid_point_to_svector(goal);

    let result = astar_search_detailed(costmap, &start_svector, &goal_svector);

    match result.path {
        Some(path) => {
            let grid_path = path.iter().map(|p| svector_to_grid_point(p)).collect();
            PathResult::success(
                grid_path,
                result.total_cost.unwrap_or(0),
                result.nodes_explored,
            )
        }
        None => PathResult::failure(result.nodes_explored),
    }
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
/// * `Option<Vec<WorldPoint<D>>>` - The path in world coordinates if found, `None` otherwise.
pub fn astar_search_world<const D: usize>(
    costmap: &CostMap<D>,
    start_world: WorldPoint<D>,
    goal_world: WorldPoint<D>,
) -> Option<Vec<WorldPoint<D>>> {
    let result = astar_search_world_detailed(costmap, start_world, goal_world);
    result.into_path()
}

/// Finds a path from a start world point to a goal world point using A* algorithm with detailed results.
///
/// Coordinates are converted to grid coordinates for pathfinding, then back to world coordinates.
/// # Arguments
/// * `costmap` - The costmap to plan in.
/// * `start_world` - Starting point in world coordinates (meters).
/// * `goal_world` - Goal point in world coordinates (meters).
///
/// # Returns
/// * `PathResult<WorldPoint<D>, D>` - Detailed pathfinding result with metadata.
pub fn astar_search_world_detailed<const D: usize>(
    costmap: &CostMap<D>,
    start_world: WorldPoint<D>,
    goal_world: WorldPoint<D>,
) -> PathResult<WorldPoint<D>, D> {
    // Convert world coordinates to grid coordinates
    let start_world_svector = world_point_to_svector(start_world);
    let goal_world_svector = world_point_to_svector(goal_world);

    let start_grid = match costmap.world_to_grid(&start_world_svector) {
        Some(grid) => grid,
        None => return PathResult::failure(0),
    };

    let goal_grid = match costmap.world_to_grid(&goal_world_svector) {
        Some(grid) => grid,
        None => return PathResult::failure(0),
    };

    // Find path in grid coordinates
    let result = astar_search_detailed(costmap, &start_grid, &goal_grid);

    match result.path {
        Some(grid_path) => {
            // Convert path back to world coordinates
            // The path consists of the centers of the grid cells.
            let world_path: Vec<WorldPoint<D>> = grid_path
                .iter()
                .filter_map(|grid_p| {
                    costmap
                        .grid_to_world(grid_p)
                        .map(|world_svector| svector_to_world_point(&world_svector))
                })
                .collect();

            if world_path.len() != grid_path.len() {
                // This might happen if a grid point in the path somehow becomes invalid for grid_to_world conversion,
                // though it shouldn't if astar_search only returns valid points within the costmap.
                return PathResult::failure(result.nodes_explored);
            }

            PathResult::success(
                world_path,
                result.total_cost.unwrap_or(0),
                result.nodes_explored,
            )
        }
        None => PathResult::failure(result.nodes_explored),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    // Ensure CostMap, CellCost, GridPoint, WorldPoint are in scope for tests
    use crate::map::costmap::{CellCost, CostMap};
    use aurus_common::{GridPoint, WorldPoint};
    use nalgebra::SVector;

    // 2D Tests
    #[test]
    fn test_astar_2d_simple_path() {
        let dims = SVector::<usize, 2>::new(10, 10);
        let resolution = SVector::<f32, 2>::new(0.1, 0.1);
        let origin = SVector::<f32, 2>::new(0.0, 0.0);
        let mut costmap = CostMap::<2>::new(dims, resolution, origin).unwrap();

        // Create a simple obstacle pattern (a box in the middle)
        for x in 3..7 {
            for y in 3..7 {
                let coords = SVector::<usize, 2>::new(x, y);
                costmap.set_cost(&coords, CellCost::Lethal).unwrap();
            }
        }

        let start = GridPoint::<2>::from([0, 0]);
        let goal = GridPoint::<2>::from([9, 9]);

        let path = astar_search_grid(&costmap, start, goal);
        assert!(path.is_some(), "Path should be found for simple case");

        if let Some(p) = &path {
            assert_eq!(p[0], start, "Path start mismatch");
            assert_eq!(*p.last().unwrap(), goal, "Path goal mismatch");
            // Path should go around the obstacle
            assert!(
                p.len() > 12,
                "Path should be longer than direct diagonal due to obstacle"
            );
        }
    }

    #[test]
    fn test_astar_2d_world_coordinates() {
        let dims = SVector::<usize, 2>::new(10, 10);
        let resolution = SVector::<f32, 2>::new(0.1, 0.1);
        let origin = SVector::<f32, 2>::new(-0.5, -0.5);
        let mut costmap = CostMap::<2>::new(dims, resolution, origin).unwrap();

        for x in 3..=6 {
            for y in 3..=6 {
                let coords = SVector::<usize, 2>::new(x, y);
                costmap.set_cost(&coords, CellCost::Lethal).unwrap();
            }
        }

        let start_world = WorldPoint::<2>::from([-0.45, -0.45]); // Approx center of grid (0,0)
        let goal_world = WorldPoint::<2>::from([0.45, 0.45]); // Approx center of grid (9,9)

        let path = astar_search_world(&costmap, start_world, goal_world);
        assert!(path.is_some(), "Path should be found for world coords");

        if let Some(p) = &path {
            assert!(!p.is_empty(), "Path should not be empty");
            // Check that start and end are approximately correct
            let start_diff = (p[0].coords - start_world.coords).norm();
            let end_diff = (p.last().unwrap().coords - goal_world.coords).norm();
            assert!(
                start_diff < 0.1,
                "Start position should be close to requested start"
            );
            assert!(
                end_diff < 0.1,
                "End position should be close to requested goal"
            );
        }
    }

    #[test]
    fn test_astar_2d_no_path() {
        let dims = SVector::<usize, 2>::new(5, 5);
        let resolution = SVector::<f32, 2>::new(0.1, 0.1);
        let origin = SVector::<f32, 2>::new(0.0, 0.0);
        let mut costmap = CostMap::<2>::new(dims, resolution, origin).unwrap();

        // Create a wall blocking the path
        for y_coord in 0..5 {
            let coords = SVector::<usize, 2>::new(2, y_coord);
            costmap.set_cost(&coords, CellCost::Lethal).unwrap();
        }

        let start = GridPoint::<2>::from([0, 0]);
        let goal = GridPoint::<2>::from([4, 4]);

        let path = astar_search_grid(&costmap, start, goal);
        assert!(path.is_none(), "Path should not be found when blocked");
    }

    #[test]
    fn test_astar_2d_inflated_costs() {
        let dims = SVector::<usize, 2>::new(8, 8);
        let resolution = SVector::<f32, 2>::new(0.1, 0.1);
        let origin = SVector::<f32, 2>::new(0.0, 0.0);
        let mut costmap = CostMap::<2>::new(dims, resolution, origin).unwrap();

        // Create lethal obstacle
        let coords = SVector::<usize, 2>::new(4, 4);
        costmap.set_cost(&coords, CellCost::Lethal).unwrap();

        // Create inflated costs around it
        for x in 3..=5 {
            for y in 3..=5 {
                if x != 4 || y != 4 {
                    let coords = SVector::<usize, 2>::new(x, y);
                    costmap.set_cost(&coords, CellCost::Inflated(50)).unwrap();
                }
            }
        }

        let start = GridPoint::<2>::from([0, 0]);
        let goal = GridPoint::<2>::from([7, 7]);

        let path = astar_search_grid(&costmap, start, goal);
        assert!(
            path.is_some(),
            "Path should be found through inflated areas"
        );

        if let Some(p) = &path {
            // Path should avoid the lethal cell but may go through inflated ones
            for point in p {
                let coords = SVector::<usize, 2>::new(point.coords[0], point.coords[1]);
                let cost = costmap.get_cost(&coords).unwrap();
                assert!(
                    !matches!(cost, CellCost::Lethal),
                    "Path should not go through lethal cells"
                );
            }
        }
    }

    // 3D Tests
    #[test]
    fn test_astar_3d_simple_path() {
        let dims = SVector::<usize, 3>::new(6, 6, 6);
        let resolution = SVector::<f32, 3>::new(0.1, 0.1, 0.1);
        let origin = SVector::<f32, 3>::new(0.0, 0.0, 0.0);
        let mut costmap = CostMap::<3>::new(dims, resolution, origin).unwrap();

        // Create a 3D obstacle (a cube in the middle)
        for x in 2..4 {
            for y in 2..4 {
                for z in 2..4 {
                    let coords = SVector::<usize, 3>::new(x, y, z);
                    costmap.set_cost(&coords, CellCost::Lethal).unwrap();
                }
            }
        }

        let start = GridPoint::<3>::from([0, 0, 0]);
        let goal = GridPoint::<3>::from([5, 5, 5]);

        let path = astar_search_grid(&costmap, start, goal);
        assert!(path.is_some(), "3D path should be found");

        if let Some(p) = &path {
            assert_eq!(p[0], start, "3D path start mismatch");
            assert_eq!(*p.last().unwrap(), goal, "3D path goal mismatch");
            // Path should go around the 3D obstacle
            assert!(
                p.len() > 8,
                "3D path should be longer than direct path due to obstacle"
            );
        }
    }

    #[test]
    fn test_astar_3d_world_coordinates() {
        let dims = SVector::<usize, 3>::new(8, 8, 8);
        let resolution = SVector::<f32, 3>::new(0.1, 0.1, 0.1);
        let origin = SVector::<f32, 3>::new(-0.4, -0.4, -0.4);
        let mut costmap = CostMap::<3>::new(dims, resolution, origin).unwrap();

        // Create a 3D obstacle
        for x in 3..5 {
            for y in 3..5 {
                for z in 3..5 {
                    let coords = SVector::<usize, 3>::new(x, y, z);
                    costmap.set_cost(&coords, CellCost::Lethal).unwrap();
                }
            }
        }

        let start_world = WorldPoint::<3>::from([-0.35, -0.35, -0.35]);
        let goal_world = WorldPoint::<3>::from([0.35, 0.35, 0.35]);

        let path = astar_search_world(&costmap, start_world, goal_world);
        assert!(path.is_some(), "3D world coordinate path should be found");

        if let Some(p) = &path {
            assert!(!p.is_empty(), "3D path should not be empty");
            // Check that start and end are approximately correct
            let start_diff = (p[0].coords - start_world.coords).norm();
            let end_diff = (p.last().unwrap().coords - goal_world.coords).norm();
            assert!(
                start_diff < 0.1,
                "3D start position should be close to requested start"
            );
            assert!(
                end_diff < 0.1,
                "3D end position should be close to requested goal"
            );
        }
    }

    #[test]
    fn test_astar_3d_no_path() {
        let dims = SVector::<usize, 3>::new(5, 5, 5);
        let resolution = SVector::<f32, 3>::new(0.1, 0.1, 0.1);
        let origin = SVector::<f32, 3>::new(0.0, 0.0, 0.0);
        let mut costmap = CostMap::<3>::new(dims, resolution, origin).unwrap();

        // Create a 3D wall blocking the path (a plane at x=2)
        for y in 0..5 {
            for z in 0..5 {
                let coords = SVector::<usize, 3>::new(2, y, z);
                costmap.set_cost(&coords, CellCost::Lethal).unwrap();
            }
        }

        let start = GridPoint::<3>::from([0, 0, 0]);
        let goal = GridPoint::<3>::from([4, 4, 4]);

        let path = astar_search_grid(&costmap, start, goal);
        assert!(
            path.is_none(),
            "3D path should not be found when blocked by wall"
        );
    }

    #[test]
    fn test_astar_3d_layer_navigation() {
        let dims = SVector::<usize, 3>::new(6, 6, 6);
        let resolution = SVector::<f32, 3>::new(0.1, 0.1, 0.1);
        let origin = SVector::<f32, 3>::new(0.0, 0.0, 0.0);
        let mut costmap = CostMap::<3>::new(dims, resolution, origin).unwrap();

        // Block the bottom layer (z=0) except for start and goal
        for x in 0..6 {
            for y in 0..6 {
                if !(x == 0 && y == 0) && !(x == 5 && y == 5) {
                    let coords = SVector::<usize, 3>::new(x, y, 0);
                    costmap.set_cost(&coords, CellCost::Lethal).unwrap();
                }
            }
        }

        let start = GridPoint::<3>::from([0, 0, 0]);
        let goal = GridPoint::<3>::from([5, 5, 0]);

        let path = astar_search_grid(&costmap, start, goal);
        assert!(
            path.is_some(),
            "3D path should be found by going up and around"
        );

        if let Some(p) = &path {
            // Path should go up to avoid blocked cells
            let max_z = p.iter().map(|point| point.coords[2]).max().unwrap();
            assert!(
                max_z > 0,
                "Path should use higher z levels to avoid obstacles"
            );
        }
    }

    // Edge case tests for both dimensions
    #[test]
    fn test_manhattan_distance_2d() {
        let a = SVector::<usize, 2>::new(0, 0);
        let b = SVector::<usize, 2>::new(3, 4);
        assert_eq!(manhattan_distance(&a, &b), 7);
    }

    #[test]
    fn test_manhattan_distance_3d() {
        let a = SVector::<usize, 3>::new(0, 0, 0);
        let b = SVector::<usize, 3>::new(3, 4, 5);
        assert_eq!(manhattan_distance(&a, &b), 12);
    }

    #[test]
    fn test_astar_same_start_goal_2d() {
        let dims = SVector::<usize, 2>::new(5, 5);
        let resolution = SVector::<f32, 2>::new(0.1, 0.1);
        let origin = SVector::<f32, 2>::new(0.0, 0.0);
        let costmap = CostMap::<2>::new(dims, resolution, origin).unwrap();

        let start = GridPoint::<2>::from([2, 2]);
        let goal = GridPoint::<2>::from([2, 2]);

        let path = astar_search_grid(&costmap, start, goal);
        assert!(
            path.is_some(),
            "Path should be found when start equals goal"
        );

        if let Some(p) = &path {
            assert_eq!(p.len(), 1, "Path should contain only the start/goal point");
            assert_eq!(p[0], start);
        }
    }

    #[test]
    fn test_astar_same_start_goal_3d() {
        let dims = SVector::<usize, 3>::new(5, 5, 5);
        let resolution = SVector::<f32, 3>::new(0.1, 0.1, 0.1);
        let origin = SVector::<f32, 3>::new(0.0, 0.0, 0.0);
        let costmap = CostMap::<3>::new(dims, resolution, origin).unwrap();

        let start = GridPoint::<3>::from([2, 2, 2]);
        let goal = GridPoint::<3>::from([2, 2, 2]);

        let path = astar_search_grid(&costmap, start, goal);
        assert!(
            path.is_some(),
            "3D path should be found when start equals goal"
        );

        if let Some(p) = &path {
            assert_eq!(
                p.len(),
                1,
                "3D path should contain only the start/goal point"
            );
            assert_eq!(p[0], start);
        }
    }

    #[test]
    fn test_path_result_detailed() {
        let dims = SVector::<usize, 2>::new(5, 5);
        let resolution = SVector::<f32, 2>::new(0.1, 0.1);
        let origin = SVector::<f32, 2>::new(0.0, 0.0);
        let mut costmap = CostMap::<2>::new(dims, resolution, origin).unwrap();

        // Add some obstacles to make the search more interesting
        let coords = SVector::<usize, 2>::new(2, 1);
        costmap.set_cost(&coords, CellCost::Lethal).unwrap();
        let coords = SVector::<usize, 2>::new(2, 2);
        costmap.set_cost(&coords, CellCost::Lethal).unwrap();

        let start = GridPoint::<2>::from([0, 0]);
        let goal = GridPoint::<2>::from([4, 4]);

        let result = astar_search_grid_detailed(&costmap, start, goal);

        assert!(result.is_success(), "Path should be found");
        assert!(result.nodes_explored > 0, "Should have explored some nodes");
        assert!(result.path_length > 0, "Path should have length");
        assert!(result.total_cost.is_some(), "Should have total cost");

        // Test display
        let display_str = format!("{}", result);
        assert!(
            display_str.contains("success: true"),
            "Display should show success"
        );
        assert!(
            display_str.contains("nodes_explored"),
            "Display should show nodes explored"
        );

        // Test failure case
        let mut blocked_costmap = CostMap::<2>::new(dims, resolution, origin).unwrap();
        // Block entire middle column
        for y in 0..5 {
            let coords = SVector::<usize, 2>::new(2, y);
            blocked_costmap.set_cost(&coords, CellCost::Lethal).unwrap();
        }

        let blocked_result = astar_search_grid_detailed(&blocked_costmap, start, goal);
        assert!(
            !blocked_result.is_success(),
            "Path should not be found when blocked"
        );
        assert_eq!(
            blocked_result.path_length, 0,
            "Failed path should have zero length"
        );
        assert!(
            blocked_result.total_cost.is_none(),
            "Failed path should have no cost"
        );

        let blocked_display = format!("{}", blocked_result);
        assert!(
            blocked_display.contains("success: false"),
            "Display should show failure"
        );
    }
}
