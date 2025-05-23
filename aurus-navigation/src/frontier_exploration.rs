use crate::map::{Costmap2D, GridPoint, CellCost};
use std::collections::HashSet;
use rand::seq::SliceRandom; // Added for random selection

/// Represents a frontier cell, which is an unknown cell adjacent to a known free cell.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct Frontier {
    /// The point representing the frontier cell itself (which is unknown).
    pub point: GridPoint,
    /// Points adjacent to this frontier cell that are known to be free.
    /// This can be useful for determining approachability or for path planning.
    pub adjacent_free_points: Vec<GridPoint>,
}

impl Frontier {
    /// Creates a new frontier.
    pub fn new(point: GridPoint, adjacent_free_points: Vec<GridPoint>) -> Self {
        Self { point, adjacent_free_points }
    }
}

/// Manages the frontier exploration process.
pub struct FrontierExploration;

impl FrontierExploration {
    /// Finds all frontier cells in the given costmap.
    ///
    /// A frontier cell is defined as an unknown cell that is adjacent (4-connectivity)
    /// to a known free cell.
    ///
    /// # Arguments
    /// * `costmap` - A reference to the `Costmap2D` to search for frontiers.
    ///
    /// # Returns
    /// * `Vec<Frontier>` - A vector of identified frontiers.
    pub fn find_frontiers(costmap: &Costmap2D) -> Vec<Frontier> {
        let mut frontiers = Vec::new();
        let width = costmap.get_width();
        let height = costmap.get_height();

        for y in 0..height {
            for x in 0..width {
                let current_point = GridPoint::new(x, y);
                match costmap.get_cost(current_point) {
                    Ok(&CellCost::Unknown) => {
                        // This cell is unknown, check its neighbors
                        let mut adjacent_free = Vec::new();
                        let neighbors = Self::get_neighbors(current_point, width, height);

                        for neighbor_pt in neighbors {
                            if let Ok(&CellCost::Free) = costmap.get_cost(neighbor_pt) {
                                adjacent_free.push(neighbor_pt);
                            }
                        }

                        if !adjacent_free.is_empty() {
                            frontiers.push(Frontier::new(current_point, adjacent_free));
                        }
                    }
                    Ok(_) => { /* Not an unknown cell, skip */ }
                    Err(_) => { /* Error getting cost (e.g. out of bounds, should not happen here), skip */ }
                }
            }
        }
        frontiers
    }

    /// Helper function to get 4-connectivity neighbors of a grid point.
    fn get_neighbors(point: GridPoint, map_width: usize, map_height: usize) -> Vec<GridPoint> {
        let mut neighbors = Vec::new();
        let x = point.x;
        let y = point.y;

        // Check top
        if y > 0 {
            neighbors.push(GridPoint::new(x, y - 1));
        }
        // Check bottom
        if y < map_height - 1 {
            neighbors.push(GridPoint::new(x, y + 1));
        }
        // Check left
        if x > 0 {
            neighbors.push(GridPoint::new(x - 1, y));
        }
        // Check right
        if x < map_width - 1 {
            neighbors.push(GridPoint::new(x + 1, y));
        }
        neighbors
    }

    /// Selects the closest frontier to the given current position based on Manhattan distance.
    ///
    /// # Arguments
    /// * `frontiers` - A slice of `Frontier` structs to choose from.
    /// * `current_position` - The `GridPoint` representing the current position.
    ///
    /// # Returns
    /// * `Option<&Frontier>` - The closest frontier, or None if the list is empty.
    pub fn select_closest_frontier<'a>(
        frontiers: &'a [Frontier],
        current_position: GridPoint,
    ) -> Option<&'a Frontier> {
        frontiers
            .iter()
            .min_by_key(|frontier| {
                let dx = (frontier.point.x as isize - current_position.x as isize).abs();
                let dy = (frontier.point.y as isize - current_position.y as isize).abs();
                dx + dy // Manhattan distance
            })
    }

    /// Selects the frontier with the largest number of adjacent free cells.
    /// This can be a heuristic for a more promising or open frontier.
    ///
    /// # Arguments
    /// * `frontiers` - A slice of `Frontier` structs to choose from.
    ///
    /// # Returns
    /// * `Option<&Frontier>` - The frontier with the most adjacent free points, or None if the list is empty.
    pub fn select_largest_frontier<'a>(frontiers: &'a [Frontier]) -> Option<&'a Frontier> {
        frontiers
            .iter()
            .max_by_key(|frontier| frontier.adjacent_free_points.len())
    }

    /// Selects a random frontier from the list.
    ///
    /// # Arguments
    /// * `frontiers` - A slice of `Frontier` structs to choose from.
    /// * `rng` - A mutable reference to a random number generator.
    ///
    /// # Returns
    /// * `Option<&Frontier>` - A randomly selected frontier, or None if the list is empty.
    pub fn select_random_frontier<'a, R: rand::Rng + ?Sized>(
        frontiers: &'a [Frontier],
        rng: &mut R,
    ) -> Option<&'a Frontier> {
        frontiers.choose(rng)
    }
}

// Basic tests module
#[cfg(test)]
mod tests {
    use super::*;
    use crate::map::{Costmap2D, WorldPoint, CellCost};
    use rand::thread_rng;

    fn create_test_map() -> Costmap2D {
        let mut costmap = Costmap2D::new(5, 5, 1.0, WorldPoint::new(0.0, 0.0)).unwrap();
        // . U . . .
        // F U F . .
        // . U . . .
        // . . . . .
        // . . . . .
        // U = Unknown, F = Free

        // Row 0
        costmap.set_cost(GridPoint::new(1, 0), CellCost::Unknown).unwrap();
        // Row 1
        costmap.set_cost(GridPoint::new(0, 1), CellCost::Free).unwrap();
        costmap.set_cost(GridPoint::new(1, 1), CellCost::Unknown).unwrap();
        costmap.set_cost(GridPoint::new(2, 1), CellCost::Free).unwrap();
        // Row 2
        costmap.set_cost(GridPoint::new(1, 2), CellCost::Unknown).unwrap();
        
        // Set all other cells to Free initially for simplicity, then mark specific ones
        for r in 0..5 {
            for c in 0..5 {
                if costmap.get_cost(GridPoint::new(c,r)).unwrap().as_u8() == CellCost::Free.as_u8() && // only if it's default free
                   !((c==0 && r==1) || (c==2 && r==1)) // and not one of our explicitly Free cells
                {
                    // Check if it's one of the Unknown cells we set
                    if !((c==1 && r==0) || (c==1 && r==1) || (c==1 && r==2)) {
                         costmap.set_cost(GridPoint::new(c, r), CellCost::Lethal).unwrap(); // Make others non-frontiers
                    }
                }
            }
        }
        // Reset specifically the ones we want Free
        costmap.set_cost(GridPoint::new(0,0), CellCost::Free).unwrap();
        costmap.set_cost(GridPoint::new(0,1), CellCost::Free).unwrap();
        costmap.set_cost(GridPoint::new(2,1), CellCost::Free).unwrap();


        // Final check for test map setup:
        // Costmap legend: F=Free, U=Unknown, L=Lethal
        // Expected map state:
        // F U L L L
        // F U F L L
        // L U L L L
        // L L L L L
        // L L L L L
        //
        // Frontiers should be:
        // (1,0) because of (0,0) which is Free (incorrect logic in my description above, fixed in setup)
        // (1,1) because of (0,1) and (2,1) which are Free
        // (1,2) because of no adjacent free cells (incorrect logic in description, fixed in setup)
        // After fixing map:
        //
        // Map:
        //  (0,0)F (1,0)U (2,0)L (3,0)L (4,0)L
        //  (0,1)F (1,1)U (2,1)F (3,1)L (4,1)L
        //  (0,2)L (1,2)U (2,2)L (3,2)L (4,2)L
        //  (0,3)L (1,3)L (2,3)L (3,3)L (4,3)L
        //  (0,4)L (1,4)L (2,4)L (3,4)L (4,4)L

        // Clear map and set specific cells for simpler frontier test
        let mut costmap = Costmap2D::new(5, 5, 1.0, WorldPoint::new(0.0, 0.0)).unwrap();
        for r in 0..5 { // Initialize all to Lethal
            for c in 0..5 {
                costmap.set_cost(GridPoint::new(c, r), CellCost::Lethal).unwrap();
            }
        }
        // Set Free cells
        costmap.set_cost(GridPoint::new(0, 1), CellCost::Free).unwrap();
        costmap.set_cost(GridPoint::new(2, 1), CellCost::Free).unwrap();
        costmap.set_cost(GridPoint::new(1, 3), CellCost::Free).unwrap();

        // Set Unknown cells (potential frontiers)
        costmap.set_cost(GridPoint::new(1, 1), CellCost::Unknown).unwrap(); // Frontier
        costmap.set_cost(GridPoint::new(1, 2), CellCost::Unknown).unwrap(); // Frontier
        costmap.set_cost(GridPoint::new(3, 3), CellCost::Unknown).unwrap(); // Not a frontier (no adjacent free)

        // Expected:
        // L L L L L
        // F U F L L  (1,1) is Unknown, (0,1)F, (2,1)F -> Frontier
        // L U L L L  (1,2) is Unknown, (1,1)U, (1,3)F -> Frontier
        // L F L U L  (3,3) is Unknown, no adjacent Free -> Not Frontier
        // L L L L L

        costmap
    }

    #[test]
    fn test_find_frontiers() {
        let costmap = create_test_map();

        // Print map for debugging (optional)
        // for r in 0..costmap.get_height() {
        //     for c in 0..costmap.get_width() {
        //         match costmap.get_cost(GridPoint::new(c,r)).unwrap() {
        //             CellCost::Free => print!("F "),
        //             CellCost::Unknown => print!("U "),
        //             CellCost::Lethal => print!("L "),
        //             _ => print!("X "),
        //         }
        //     }
        //     println!();
        // }

        let frontiers = FrontierExploration::find_frontiers(&costmap);

        // Expected frontiers:
        // GridPoint::new(1,1) with adjacent_free [GridPoint::new(0,1), GridPoint::new(2,1)]
        // GridPoint::new(1,2) with adjacent_free [GridPoint::new(1,1) - NO, this is U, GridPoint::new(1,3) - YES]
        
        // Sort for consistent comparison
        let mut found_points: Vec<GridPoint> = frontiers.iter().map(|f| f.point).collect();
        found_points.sort_by(|a, b| (a.y, a.x).cmp(&(b.y, b.x)));

        let expected_points = vec![
            GridPoint::new(1, 1),
            GridPoint::new(1, 2),
        ];
        assert_eq!(found_points, expected_points, "Mismatch in identified frontier points");

        // Check details for GridPoint::new(1,1)
        let frontier_1_1 = frontiers.iter().find(|f| f.point == GridPoint::new(1,1)).expect("Frontier (1,1) not found");
        let mut adj_free_1_1 = frontier_1_1.adjacent_free_points.clone();
        adj_free_1_1.sort_by(|a, b| (a.y, a.x).cmp(&(b.y, b.x)));
        let expected_adj_free_1_1 = vec![GridPoint::new(0,1), GridPoint::new(2,1)];
         assert_eq!(adj_free_1_1, expected_adj_free_1_1, "Mismatch in adjacent free for (1,1)");

        // Check details for GridPoint::new(1,2)
        let frontier_1_2 = frontiers.iter().find(|f| f.point == GridPoint::new(1,2)).expect("Frontier (1,2) not found");
        let mut adj_free_1_2 = frontier_1_2.adjacent_free_points.clone();
        adj_free_1_2.sort_by(|a, b| (a.y, a.x).cmp(&(b.y, b.x)));
        let expected_adj_free_1_2 = vec![GridPoint::new(1,3)]; // Only (1,3) is Free. (1,1) is Unknown.
        assert_eq!(adj_free_1_2, expected_adj_free_1_2, "Mismatch in adjacent free for (1,2)");

    }

     #[test]
    fn test_get_neighbors() {
        let map_width = 5;
        let map_height = 5;

        // Corner case: (0,0)
        let mut neighbors_0_0 = FrontierExploration::get_neighbors(GridPoint::new(0,0), map_width, map_height);
        neighbors_0_0.sort_by_key(|p| (p.y, p.x)); // Sort for consistent comparison
        assert_eq!(neighbors_0_0, vec![GridPoint::new(1,0), GridPoint::new(0,1)]);

        // Edge case: (2,0)
        let mut neighbors_2_0 = FrontierExploration::get_neighbors(GridPoint::new(2,0), map_width, map_height);
        neighbors_2_0.sort_by_key(|p| (p.y, p.x));
        assert_eq!(neighbors_2_0, vec![GridPoint::new(1,0), GridPoint::new(3,0), GridPoint::new(2,1)]);
        
        // Center case: (2,2)
        let mut neighbors_2_2 = FrontierExploration::get_neighbors(GridPoint::new(2,2), map_width, map_height);
        neighbors_2_2.sort_by_key(|p| (p.y, p.x));
        assert_eq!(neighbors_2_2, vec![GridPoint::new(2,1), GridPoint::new(1,2), GridPoint::new(3,2), GridPoint::new(2,3)]);
    }

    fn create_test_map_for_selection() -> (Costmap2D, Vec<Frontier>) {
        let mut costmap = Costmap2D::new(10, 10, 1.0, WorldPoint::new(0.0, 0.0)).unwrap();
        for r in 0..10 { // Initialize all to Lethal
            for c in 0..10 {
                costmap.set_cost(GridPoint::new(c, r), CellCost::Lethal).unwrap();
            }
        }

        // Setup a scenario with a few distinct frontiers
        // F1: (2,2) - Unknown, adjacent to (1,2)F, (2,1)F
        // F2: (7,7) - Unknown, adjacent to (7,6)F
        // F3: (2,8) - Unknown, adjacent to (1,8)F, (2,7)F, (3,8)F (largest)

        // Free cells
        costmap.set_cost(GridPoint::new(1, 2), CellCost::Free).unwrap();
        costmap.set_cost(GridPoint::new(2, 1), CellCost::Free).unwrap();
        costmap.set_cost(GridPoint::new(7, 6), CellCost::Free).unwrap();
        costmap.set_cost(GridPoint::new(1, 8), CellCost::Free).unwrap();
        costmap.set_cost(GridPoint::new(2, 7), CellCost::Free).unwrap();
        costmap.set_cost(GridPoint::new(3, 8), CellCost::Free).unwrap();

        // Unknown cells (frontiers)
        costmap.set_cost(GridPoint::new(2, 2), CellCost::Unknown).unwrap(); // F1
        costmap.set_cost(GridPoint::new(7, 7), CellCost::Unknown).unwrap(); // F2
        costmap.set_cost(GridPoint::new(2, 8), CellCost::Unknown).unwrap(); // F3

        let frontiers = FrontierExploration::find_frontiers(&costmap);
        (costmap, frontiers)
    }

    #[test]
    fn test_select_closest_frontier() {
        let (_costmap, frontiers) = create_test_map_for_selection();
        let current_pos = GridPoint::new(0, 0);

        let selected = FrontierExploration::select_closest_frontier(&frontiers, current_pos);
        assert!(selected.is_some(), "Should select a frontier");
        // F1 (2,2) dist = 2+2=4
        // F2 (7,7) dist = 7+7=14
        // F3 (2,8) dist = 2+8=10
        assert_eq!(selected.unwrap().point, GridPoint::new(2,2), "Closest should be (2,2)");

        let current_pos_near_f3 = GridPoint::new(2, 6);
        // F1 (2,2) dist = 0+4=4
        // F2 (7,7) dist = 5+1=6
        // F3 (2,8) dist = 0+2=2
        let selected_near_f3 = FrontierExploration::select_closest_frontier(&frontiers, current_pos_near_f3);
        assert!(selected_near_f3.is_some(), "Should select a frontier");
        assert_eq!(selected_near_f3.unwrap().point, GridPoint::new(2,8), "Closest to (2,6) should be (2,8)");
        
        assert!(FrontierExploration::select_closest_frontier(&[], current_pos).is_none(), "Should return None for empty frontiers");
    }

    #[test]
    fn test_select_largest_frontier() {
        let (_costmap, frontiers) = create_test_map_for_selection();
        
        // F1 (2,2) has 2 adjacent free: (1,2), (2,1)
        // F2 (7,7) has 1 adjacent free: (7,6)
        // F3 (2,8) has 3 adjacent free: (1,8), (2,7), (3,8)

        let selected = FrontierExploration::select_largest_frontier(&frontiers);
        assert!(selected.is_some(), "Should select a frontier");
        assert_eq!(selected.unwrap().point, GridPoint::new(2,8), "Largest should be (2,8) with 3 adj free");
        
        assert!(FrontierExploration::select_largest_frontier(&[]).is_none(), "Should return None for empty frontiers");
    }

    #[test]
    fn test_select_random_frontier() {
        let (_costmap, frontiers) = create_test_map_for_selection();
        let mut rng = thread_rng();

        assert!(!frontiers.is_empty(), "Test setup should have frontiers");

        let selected = FrontierExploration::select_random_frontier(&frontiers, &mut rng);
        assert!(selected.is_some(), "Should select a frontier from a non-empty list");
        
        // Check if the selected frontier is one of the actual frontiers
        let selected_point = selected.unwrap().point;
        assert!(frontiers.iter().any(|f| f.point == selected_point), "Selected frontier must be in the original list");

        assert!(FrontierExploration::select_random_frontier(&[], &mut rng).is_none(), "Should return None for empty frontiers");
    }
}
