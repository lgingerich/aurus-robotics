#![warn(missing_docs)]

// NOTES / TODO:
// - Not no_std compatible
// - Does not support negative coordinates (is this needed?)

use crate::error::NavigationError;
use super::{GridPoint, WorldPoint};

/// Represents the cost of traversing a cell in the costmap.
/// The cost values are used for path planning and obstacle avoidance.
#[repr(u8)]
#[derive(Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum CellCost {
    /// Free cell, no cost to traverse
    Free = 0,
    /// Cell with inflated obstacle cost (1 - 252)
    /// Higher values indicate higher traversal cost
    Inflated(u8),
    /// Cell that is guaranteed to be in collision with obstacles
    /// The robot cannot fit through this cell
    Inscribed = 253,
    /// Cell that contains an obstacle or is too close to an obstacle to be traversable
    Lethal = 254,
    /// Unknown cell - no information available about this cell
    Unknown = 255,
}

impl CellCost {
    /// Converts the CellCost to its u8 representation
    pub fn as_u8(&self) -> u8 {
        match self {
            CellCost::Free => 0,
            CellCost::Inflated(cost) => *cost,
            CellCost::Inscribed => 253,
            CellCost::Lethal => 254,
            CellCost::Unknown => 255,
        }
    }

    /// Creates a CellCost from a u8 value
    pub fn from_u8(value: u8) -> Self {
        match value {
            0 => CellCost::Free,
            253 => CellCost::Inscribed,
            254 => CellCost::Lethal,
            255 => CellCost::Unknown,
            _ => CellCost::Inflated(value),
        }
    }
}

impl std::fmt::Display for CellCost {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CellCost::Free => write!(f, "Free"),
            CellCost::Inflated(cost) => write!(f, "Inflated({})", cost),
            CellCost::Inscribed => write!(f, "Inscribed"),
            CellCost::Lethal => write!(f, "Lethal"),
            CellCost::Unknown => write!(f, "Unknown"),
        }
    }
}

/// A 2D costmap representing the environment with associated costs for each cell.
/// The costmap is used for path planning and obstacle avoidance.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Costmap2D {
    /// Width of the costmap in cells
    width: usize,
    /// Height of the costmap in cells
    height: usize,
    /// Resolution of the costmap in meters per cell
    resolution: f32,
    /// X coordinate of the origin in world coordinates (meters)
    origin_x: f32,
    /// Y coordinate of the origin in world coordinates (meters)
    origin_y: f32,
    /// Vector storing the cost values for each cell
    data: Vec<CellCost>,
}

impl Costmap2D {
    /// Creates a new Costmap2D with the specified dimensions and resolution.
    /// 
    /// # Arguments
    /// * `width` - Width of the costmap in cells
    /// * `height` - Height of the costmap in cells
    /// * `resolution` - Resolution of the costmap in meters per cell
    /// * `origin` - The world coordinates (meters) of the cell (0,0) in the costmap (bottom-left corner).
    /// 
    /// # Returns
    /// * `Result<Self, NavigationError>` - The created Costmap2D or an error if parameters are invalid
    pub fn new(width: usize, height: usize, resolution: f32, origin: WorldPoint) -> Result<Self, NavigationError> {
        if resolution <= 0.0 {
            return Err(NavigationError::InvalidResolution("Resolution must be positive"));
        }
        if width == 0 || height == 0 {
            return Err(NavigationError::InvalidDimensions("Width and height must be non-zero"));
        }

        // Check for potential overflow in data allocation
        if width.checked_mul(height).is_none() {
            return Err(NavigationError::InvalidDimensions("Map dimensions too large, would cause overflow"));
        }

        Ok(Costmap2D {
            width,
            height,
            resolution,
            origin_x: origin.x,
            origin_y: origin.y,
            data: vec![CellCost::Free; width * height]
        })
    }

    /// Calculates the index in the data vector for a given grid point
    fn get_index(&self, p: GridPoint) -> usize {
        p.y * self.width + p.x
    }

    /// Converts world coordinates (in meters) to grid coordinates (in cells).
    /// Returns None if the world coordinates are outside the map bounds.
    /// 
    /// # Arguments
    /// * `world_p` - Point in world frame (meters)
    /// 
    /// # Returns
    /// * `Option<GridPoint>` - Grid coordinates if within bounds, None otherwise
    pub fn world_to_grid(&self, world_p: WorldPoint) -> Option<GridPoint> {
        // Convert to grid coordinates
        let grid_x_f = (world_p.x - self.origin_x) / self.resolution;
        let grid_y_f = (world_p.y - self.origin_y) / self.resolution;

        // Ensure the point is not effectively outside due to floating point issues at the boundary
        // A point exactly on the max boundary is considered outside.
        if grid_x_f < 0.0 || grid_y_f < 0.0 || grid_x_f >= self.width as f32 || grid_y_f >= self.height as f32 {
            return None;
        }

        let grid_x = grid_x_f.floor() as usize;
        let grid_y = grid_y_f.floor() as usize;

        // Final check to ensure casted usize is within bounds
        // This is somewhat redundant given the f32 check above but safe
        if grid_x < self.width && grid_y < self.height {
            Some(GridPoint::new(grid_x, grid_y))
        } else {
            None
        }
    }

    /// Converts grid coordinates (in cells) to world coordinates (in meters).
    /// Returns the center point of the cell in world coordinates.
    /// 
    /// # Arguments
    /// * `grid_p` - Point in grid frame (cells)
    /// 
    /// # Returns
    /// * `Option<WorldPoint>` - World coordinates if within bounds, None otherwise
    pub fn grid_to_world(&self, grid_p: GridPoint) -> Option<WorldPoint> {
        if grid_p.x < self.width && grid_p.y < self.height {
            let world_x = self.origin_x + (grid_p.x as f32 + 0.5) * self.resolution;
            let world_y = self.origin_y + (grid_p.y as f32 + 0.5) * self.resolution;
            Some(WorldPoint::new(world_x, world_y))
        } else {
            None
        }
    }

    /// Gets the cost at a world coordinate position.
    /// 
    /// # Arguments
    /// * `world_p` - Point in world frame (meters)
    /// 
    /// # Returns
    /// * `Result<&CellCost, NavigationError>` - The cost at the specified position or an error if out of bounds
    pub fn get_cost_at_world(&self, world_p: WorldPoint) -> Result<&CellCost, NavigationError> {
        if let Some(grid_p) = self.world_to_grid(world_p) {
            self.get_cost(grid_p)
        } else {
            Err(NavigationError::InvalidWorldCoordinates("World coordinates outside map bounds"))
        }
    }

    /// Sets the cost at a world coordinate position.
    /// 
    /// # Arguments
    /// * `world_p` - Point in world frame (meters)
    /// * `cost` - The cost to set at the specified position
    /// 
    /// # Returns
    /// * `Result<(), NavigationError>` - Success or error if out of bounds
    pub fn set_cost_at_world(&mut self, world_p: WorldPoint, cost: CellCost) -> Result<(), NavigationError> {
        if let Some(grid_p) = self.world_to_grid(world_p) {
            self.set_cost(grid_p, cost)
        } else {
            Err(NavigationError::InvalidWorldCoordinates("World coordinates outside map bounds"))
        }
    }

    /// Gets the world coordinates of the map's origin (bottom-left corner).
    ///
    /// # Returns
    /// * `WorldPoint` - The origin of the map in world coordinates.
    pub fn get_origin(&self) -> WorldPoint {
        WorldPoint::new(self.origin_x, self.origin_y)
    }

    /// Gets the world coordinates of the map's bounds.
    /// 
    /// # Returns
    /// * `(WorldPoint, WorldPoint)` - The minimum and maximum world points (bottom-left and top-right corners)
    pub fn get_world_bounds(&self) -> (WorldPoint, WorldPoint) {
        let min_p = WorldPoint::new(self.origin_x, self.origin_y);
        let max_x = self.origin_x + (self.width as f32 * self.resolution);
        let max_y = self.origin_y + (self.height as f32 * self.resolution);
        let max_p = WorldPoint::new(max_x, max_y);
        (min_p, max_p)
    }

    /// Gets the cost at a grid coordinate position.
    /// 
    /// # Arguments
    /// * `p` - Point in grid frame (cells)
    /// 
    /// # Returns
    /// * `Result<&CellCost, NavigationError>` - The cost at the specified position or an error if out of bounds
    pub fn get_cost(&self, p: GridPoint) -> Result<&CellCost, NavigationError> {
        if p.x < self.width && p.y < self.height {
            let index = self.get_index(p);
            Ok(&self.data[index])
        } else {
            Err(NavigationError::OutOfBounds("Costmap index out of bounds")) // TODO: Should I return the input coordinates in this error?
        }
    }

    /// Gets a mutable reference to the cost at a grid coordinate position.
    /// 
    /// # Arguments
    /// * `p` - Point in grid frame (cells)
    /// 
    /// # Returns
    /// * `Result<&mut CellCost, NavigationError>` - Mutable reference to the cost or an error if out of bounds
    pub fn get_cost_mut(&mut self, p: GridPoint) -> Result<&mut CellCost, NavigationError> {
        if p.x < self.width && p.y < self.height {
            let index = self.get_index(p);
            Ok(&mut self.data[index])
        } else {
            Err(NavigationError::OutOfBounds("Costmap index out of bounds"))
        }
    }

    /// Sets the cost at a grid coordinate position.
    /// 
    /// # Arguments
    /// * `p` - Point in grid frame (cells)
    /// * `cost` - The cost to set at the specified position
    /// 
    /// # Returns
    /// * `Result<(), NavigationError>` - Success or error if out of bounds
    pub fn set_cost(&mut self, p: GridPoint, cost: CellCost) -> Result<(), NavigationError> {
        if p.x < self.width && p.y < self.height {
            let index = self.get_index(p);
            self.data[index] = cost;
            Ok(())
        } else {
            Err(NavigationError::OutOfBounds("Costmap index out of bounds"))
        }
    }

    /// Gets a reference to the underlying cost data.
    /// 
    /// # Returns
    /// * `&[CellCost]` - Slice containing all cost values
    pub fn get_data(&self) -> &[CellCost] {
        &self.data
    }

    /// Gets a mutable reference to the underlying cost data.
    /// 
    /// # Returns
    /// * `&mut [CellCost]` - Mutable slice containing all cost values
    pub fn get_data_mut(&mut self) -> &mut [CellCost] {
        &mut self.data
    }

    /// Inflates obstacles in the costmap by a specified radius.
    /// This creates a safety margin around obstacles.
    /// 
    /// # Arguments
    /// * `radius` - Inflation radius in meters
    /// 
    /// # Returns
    /// * `Result<(), NavigationError>` - Success or error if radius is invalid
    pub fn inflate_obstacles(&mut self, radius: f32) -> Result<(), NavigationError> {
        if radius <= 0.0 {
            return Err(NavigationError::InvalidResolution("Inflation radius must be positive"));
        }

        let radius_cells = (radius / self.resolution).ceil() as usize;
        let mut new_data = self.data.clone();

        // Iterate through all cells
        for y_idx in 0..self.height {
            for x_idx in 0..self.width {
                let current_grid_p = GridPoint::new(x_idx, y_idx);
                if let Ok(cost_val) = self.get_cost(current_grid_p) {
                    if *cost_val == CellCost::Lethal {
                        // Inflate this obstacle
                        // Use checked arithmetic to avoid underflow/overflow
                        let start_y = y_idx.saturating_sub(radius_cells);
                        let end_y = (y_idx + radius_cells).min(self.height - 1);
                        let start_x = x_idx.saturating_sub(radius_cells);
                        let end_x = (x_idx + radius_cells).min(self.width - 1);

                        for ny_idx in start_y..=end_y {
                            for nx_idx in start_x..=end_x {
                                let neighbor_grid_p = GridPoint::new(nx_idx, ny_idx);
                                let dx = nx_idx as f32 - x_idx as f32;
                                let dy = ny_idx as f32 - y_idx as f32;
                                let distance_sq_cells = dx * dx + dy * dy;
                                // Optimization: Check squared distance first if radius_cells is large
                                // if distance_sq_cells <= (radius_cells as f32 * radius_cells as f32) {
                                    let distance_meters = distance_sq_cells.sqrt() * self.resolution;
                                    
                                    if distance_meters <= radius {
                                        // Calculate inflation cost based on distance
                                        // Ensure cost is between 1 and 252 for Inflated variant
                                        let inflation_factor = 1.0 - (distance_meters / radius);
                                        let raw_inflation_cost = inflation_factor * 252.0;
                                        // Ensure cost is at least 1 if distance_meters < radius, otherwise 0 if distance_meters == radius.
                                        // Lethal cells (distance_meters == 0) will be overwritten by higher costs from neighbors,
                                        // or remain lethal if no closer obstacle.
                                        // This logic might need refinement if we want to preserve the original lethal cell center.
                                        let inflation_u8 = raw_inflation_cost.max(1.0).min(252.0) as u8;
                                        
                                        let idx = self.get_index(neighbor_grid_p);

                                        // Only update if new cost is higher (more restrictive) or current is Free
                                        // Avoid overwriting a Lethal cell with an Inflated cost unless it's the center of inflation
                                        match new_data[idx] {
                                            CellCost::Free => {
                                                new_data[idx] = CellCost::Inflated(inflation_u8);
                                            }
                                            CellCost::Inflated(current_inflation) => {
                                                if inflation_u8 > current_inflation {
                                                    new_data[idx] = CellCost::Inflated(inflation_u8);
                                                }
                                            }
                                            // Do not overwrite Inscribed or Lethal unless it's the inflation source itself
                                            // The source cell itself (current_grid_p) might be overwritten here which is fine if it was Lethal.
                                            // But other existing Lethal/Inscribed cells in the radius should not be downgraded.
                                            // This logic becomes tricky. For now, let's assume simpler overwrite for inflated.
                                            // A more robust approach would be to only inflate into Free or less-inflated cells.
                                             _ => { // Covers Lethal, Inscribed, Unknown - for now, we inflate over them
                                                // if neighbor_grid_p != current_grid_p { // Don't downgrade other obstacles
                                                // if new_data[idx] != CellCost::Lethal && new_data[idx] != CellCost::Inscribed {
                                                new_data[idx] = CellCost::Inflated(inflation_u8);
                                                // }
                                                // }
                                             } 
                                        }
                                    }
                                // }
                            }
                        }
                    }
                }
            }
        }
        
        // Second pass: ensure original lethal obstacles remain lethal
        // as inflation might have overwritten them with Inflated(0) if radius is small.
        for y_idx in 0..self.height {
            for x_idx in 0..self.width {
                let p = GridPoint::new(x_idx, y_idx);
                if let Ok(original_cost) = self.get_cost(p) {
                    if *original_cost == CellCost::Lethal {
                        new_data[self.get_index(p)] = CellCost::Lethal;
                    }
                }
            }
        }

        self.data = new_data;
        Ok(())
    }

    /// Clears the entire costmap, setting all cells to Free.
    pub fn clear(&mut self) {
        self.data.fill(CellCost::Free);
    }

    /// Sets all cells to Unknown.
    pub fn reset_to_unknown(&mut self) {
        self.data.fill(CellCost::Unknown);
    }
}

impl std::fmt::Display for Costmap2D {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "Costmap2D ({}x{}, resolution: {:.3}m)", 
                self.width, self.height, self.resolution)?;
        writeln!(f, "Origin: ({:.3}, {:.3})", self.origin_x, self.origin_y)?;
        
        // Print the costmap grid
        for y_idx in 0..self.height {
            for x_idx in 0..self.width {
                if let Ok(cost_val) = self.get_cost(GridPoint::new(x_idx, y_idx)) {
                    write!(f, "{:3} ", cost_val.as_u8())?;
                }
            }
            writeln!(f)?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_costmap_creation() {
        let origin = WorldPoint::new(0.0, 0.0);
        let costmap = Costmap2D::new(10, 10, 0.1, origin).unwrap();
        assert_eq!(costmap.width, 10);
        assert_eq!(costmap.height, 10);
        assert_eq!(costmap.resolution, 0.1);
        assert_eq!(costmap.origin_x, 0.0);
        assert_eq!(costmap.origin_y, 0.0);
    }

    #[test]
    fn test_invalid_creation() {
        let origin = WorldPoint::new(0.0, 0.0);
        assert!(matches!(
            Costmap2D::new(0, 10, 0.1, origin),
            Err(NavigationError::InvalidDimensions(_))
        ));
        assert!(matches!(
            Costmap2D::new(10, 0, 0.1, origin),
            Err(NavigationError::InvalidDimensions(_))
        ));
        assert!(matches!(
            Costmap2D::new(10, 10, 0.0, origin),
            Err(NavigationError::InvalidResolution(_))
        ));
    }

    #[test]
    fn test_cost_operations() {
        let origin = WorldPoint::new(0.0, 0.0);
        let mut costmap = Costmap2D::new(5, 5, 0.1, origin).unwrap();
        let p = GridPoint::new(2, 2);
        
        // Test setting and getting costs
        costmap.set_cost(p, CellCost::Lethal).unwrap();
        assert_eq!(*costmap.get_cost(p).unwrap(), CellCost::Lethal);
        
        // Test out of bounds
        let out_of_bounds_p1 = GridPoint::new(5, 2);
        let out_of_bounds_p2 = GridPoint::new(2, 5);
        assert!(matches!(
            costmap.set_cost(out_of_bounds_p1, CellCost::Lethal),
            Err(NavigationError::OutOfBounds(_))
        ));
        assert!(matches!(
            costmap.get_cost(out_of_bounds_p2),
            Err(NavigationError::OutOfBounds(_))
        ));
    }

    #[test]
    fn test_coordinate_conversion() {
        let origin = WorldPoint::new(-0.5, -0.5);
        let costmap = Costmap2D::new(10, 10, 0.1, origin).unwrap();
        
        // World (0.0, 0.0) should map to grid (5, 5)
        let world_p1 = WorldPoint::new(0.0, 0.0);
        let grid_p1 = costmap.world_to_grid(world_p1).unwrap();
        assert_eq!(grid_p1, GridPoint::new(5, 5));
        
        // Grid (5, 5) should map to world center of cell (0.0, 0.0)
        // Origin is (-0.5, -0.5), so cell (0,0) center is (-0.45, -0.45)
        // Cell (5,5) center is (-0.5 + (5+0.5)*0.1, -0.5 + (5+0.5)*0.1) = (0.05, 0.05)
        let world_p_center1 = costmap.grid_to_world(GridPoint::new(5, 5)).unwrap();
        assert!((world_p_center1.x - 0.05).abs() < 1e-6);
        assert!((world_p_center1.y - 0.05).abs() < 1e-6);
        
        // World (-0.5, -0.5) should map to grid (0,0) (origin, bottom-left corner of cell (0,0))
        let world_p2 = WorldPoint::new(-0.5, -0.5);
        let grid_p2 = costmap.world_to_grid(world_p2).unwrap();
        assert_eq!(grid_p2, GridPoint::new(0, 0));

        // Grid (0,0) should map to its center in world coords
        // (-0.5 + (0+0.5)*0.1, -0.5 + (0+0.5)*0.1) = (-0.45, -0.45)
        let world_p_center2 = costmap.grid_to_world(GridPoint::new(0, 0)).unwrap();
        assert!((world_p_center2.x - (-0.45)).abs() < 1e-6);
        assert!((world_p_center2.y - (-0.45)).abs() < 1e-6);
        
        // Test out of bounds world point (just outside top-right)
        let world_max_x = origin.x + costmap.width as f32 * costmap.resolution;
        let world_max_y = origin.y + costmap.height as f32 * costmap.resolution;
        assert!(costmap.world_to_grid(WorldPoint::new(world_max_x, world_max_y)).is_none()); // Exactly on boundary is out
        assert!(costmap.world_to_grid(WorldPoint::new(world_max_x + 0.01, world_max_y + 0.01)).is_none());
        
        // Test out of bounds grid point
        assert!(costmap.grid_to_world(GridPoint::new(10, 10)).is_none());
        assert!(costmap.grid_to_world(GridPoint::new(costmap.width, costmap.height -1)).is_none());
        assert!(costmap.grid_to_world(GridPoint::new(costmap.width -1, costmap.height)).is_none());

        // Test world point that maps to the last valid cell (width-1, height-1)
        let last_cell_world_x = origin.x + ((costmap.width -1) as f32 + 0.5) * costmap.resolution;
        let last_cell_world_y = origin.y + ((costmap.height -1) as f32 + 0.5) * costmap.resolution;
        let grid_last = costmap.world_to_grid(WorldPoint::new(last_cell_world_x, last_cell_world_y)).unwrap();
        assert_eq!(grid_last, GridPoint::new(costmap.width-1, costmap.height-1));

        // Test a point that is on the boundary between cells, should go to lower-left
        let boundary_x = origin.x + 1.0 * costmap.resolution; // Boundary between cell 0 and 1
        let boundary_y = origin.y + 1.0 * costmap.resolution; // Boundary between cell 0 and 1
        let on_boundary_grid = costmap.world_to_grid(WorldPoint::new(boundary_x, boundary_y)).unwrap();
        assert_eq!(on_boundary_grid, GridPoint::new(1,1)); // Should map to cell (1,1)
    }

    #[test]
    fn test_inflation() {
        let origin = WorldPoint::new(0.0, 0.0);
        let mut costmap = Costmap2D::new(5, 5, 0.1, origin).unwrap();
        
        // Set a lethal obstacle in the center
        costmap.set_cost(GridPoint::new(2, 2), CellCost::Lethal).unwrap();
        
        // Inflate with a small radius
        costmap.inflate_obstacles(0.15).unwrap(); // Radius covers 1 cell + a bit of the next
        
        // Center should remain lethal
        assert_eq!(*costmap.get_cost(GridPoint::new(2,2)).unwrap(), CellCost::Lethal);

        // Cells directly adjacent (distance 0.1m) should be inflated
        // Inflation cost = (1 - (0.1/0.15)) * 252 = (1 - 0.666) * 252 = 0.333 * 252 = 84
        let expected_inflation_adjacent = CellCost::Inflated(((1.0 - (0.1/0.15)) * 252.0).max(1.0).min(252.0) as u8);

        assert_eq!(*costmap.get_cost(GridPoint::new(1, 2)).unwrap(), expected_inflation_adjacent);
        assert_eq!(*costmap.get_cost(GridPoint::new(3, 2)).unwrap(), expected_inflation_adjacent);
        assert_eq!(*costmap.get_cost(GridPoint::new(2, 1)).unwrap(), expected_inflation_adjacent);
        assert_eq!(*costmap.get_cost(GridPoint::new(2, 3)).unwrap(), expected_inflation_adjacent);
        
        // Corner cells (distance sqrt(2)*0.1 = 0.1414m) should also be inflated
        // Inflation cost = (1 - (0.1414/0.15)) * 252 = (1-0.9426)*252 = 0.0574*252 = 14
        let expected_inflation_diagonal = CellCost::Inflated(((1.0 - ( (2.0f32.sqrt() * 0.1) /0.15)) * 252.0).max(1.0).min(252.0) as u8);
        assert_eq!(*costmap.get_cost(GridPoint::new(1, 1)).unwrap(), expected_inflation_diagonal);
        assert_eq!(*costmap.get_cost(GridPoint::new(3, 3)).unwrap(), expected_inflation_diagonal);
        assert_eq!(*costmap.get_cost(GridPoint::new(1, 3)).unwrap(), expected_inflation_diagonal);
        assert_eq!(*costmap.get_cost(GridPoint::new(3, 1)).unwrap(), expected_inflation_diagonal);

        // Cells further away should be Free or less inflated
        // Cell (0,0) is distance sqrt( (0.2^2) + (0.2^2) ) = sqrt(0.08) = 0.2828m, > 0.15m, so Free
        assert_eq!(*costmap.get_cost(GridPoint::new(0, 0)).unwrap(), CellCost::Free);
    }

    #[test]
    fn test_display() {
        let origin = WorldPoint::new(0.0, 0.0);
        let mut costmap = Costmap2D::new(3, 3, 0.1, origin).unwrap();
        costmap.set_cost(GridPoint::new(1, 1), CellCost::Lethal).unwrap();
        
        let display_str = format!("{}", costmap);
        assert!(display_str.contains("Costmap2D (3x3"));
        assert!(display_str.contains("254")); // Lethal cost value
    }
}
