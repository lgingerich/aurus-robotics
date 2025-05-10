#![warn(missing_docs)]

use crate::error::NavigationError;

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
    /// * `origin_x` - X coordinate of the origin in world coordinates (meters)
    /// * `origin_y` - Y coordinate of the origin in world coordinates (meters)
    /// 
    /// # Returns
    /// * `Result<Self, NavigationError>` - The created Costmap2D or an error if parameters are invalid
    pub fn new(width: usize, height: usize, resolution: f32, origin_x: f32, origin_y: f32) -> Result<Self, NavigationError> {
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
            origin_x,
            origin_y,
            data: vec![CellCost::Free; width * height]
        })
    }

    /// Calculates the index in the data vector for a given x,y coordinate
    fn get_index(&self, x: usize, y: usize) -> usize {
        y * self.width + x
    }

    /// Converts world coordinates (in meters) to grid coordinates (in cells).
    /// Returns None if the world coordinates are outside the map bounds.
    /// 
    /// # Arguments
    /// * `world_x` - X coordinate in world frame (meters)
    /// * `world_y` - Y coordinate in world frame (meters)
    /// 
    /// # Returns
    /// * `Option<(usize, usize)>` - Grid coordinates (x,y) if within bounds, None otherwise
    pub fn world_to_grid(&self, world_x: f32, world_y: f32) -> Option<(usize, usize)> {
        // Convert to grid coordinates
        let grid_x = ((world_x - self.origin_x) / self.resolution).floor() as isize;
        let grid_y = ((world_y - self.origin_y) / self.resolution).floor() as isize;

        // Check bounds
        if grid_x >= 0 && grid_x < self.width as isize && 
           grid_y >= 0 && grid_y < self.height as isize {
            Some((grid_x as usize, grid_y as usize))
        } else {
            None
        }
    }

    /// Converts grid coordinates (in cells) to world coordinates (in meters).
    /// Returns the center point of the cell in world coordinates.
    /// 
    /// # Arguments
    /// * `grid_x` - X coordinate in grid frame (cells)
    /// * `grid_y` - Y coordinate in grid frame (cells)
    /// 
    /// # Returns
    /// * `Option<(f32, f32)>` - World coordinates (x,y) if within bounds, None otherwise
    pub fn grid_to_world(&self, grid_x: usize, grid_y: usize) -> Option<(f32, f32)> {
        if grid_x < self.width && grid_y < self.height {
            let world_x = self.origin_x + (grid_x as f32 + 0.5) * self.resolution;
            let world_y = self.origin_y + (grid_y as f32 + 0.5) * self.resolution;
            Some((world_x, world_y))
        } else {
            None
        }
    }

    /// Gets the cost at a world coordinate position.
    /// 
    /// # Arguments
    /// * `world_x` - X coordinate in world frame (meters)
    /// * `world_y` - Y coordinate in world frame (meters)
    /// 
    /// # Returns
    /// * `Result<&CellCost, NavigationError>` - The cost at the specified position or an error if out of bounds
    pub fn get_cost_at_world(&self, world_x: f32, world_y: f32) -> Result<&CellCost, NavigationError> {
        if let Some((grid_x, grid_y)) = self.world_to_grid(world_x, world_y) {
            self.get_cost(grid_x, grid_y)
        } else {
            Err(NavigationError::InvalidWorldCoordinates("World coordinates outside map bounds"))
        }
    }

    /// Sets the cost at a world coordinate position.
    /// 
    /// # Arguments
    /// * `world_x` - X coordinate in world frame (meters)
    /// * `world_y` - Y coordinate in world frame (meters)
    /// * `cost` - The cost to set at the specified position
    /// 
    /// # Returns
    /// * `Result<(), NavigationError>` - Success or error if out of bounds
    pub fn set_cost_at_world(&mut self, world_x: f32, world_y: f32, cost: CellCost) -> Result<(), NavigationError> {
        if let Some((grid_x, grid_y)) = self.world_to_grid(world_x, world_y) {
            self.set_cost(grid_x, grid_y, cost)
        } else {
            Err(NavigationError::InvalidWorldCoordinates("World coordinates outside map bounds"))
        }
    }

    /// Gets the world coordinates of the map's bounds.
    /// 
    /// # Returns
    /// * `((f32, f32), (f32, f32))` - The minimum and maximum (x,y) coordinates in world frame (meters)
    pub fn get_world_bounds(&self) -> ((f32, f32), (f32, f32)) {
        let min_x = self.origin_x;
        let min_y = self.origin_y;
        let max_x = self.origin_x + (self.width as f32 * self.resolution);
        let max_y = self.origin_y + (self.height as f32 * self.resolution);
        ((min_x, min_y), (max_x, max_y))
    }

    /// Gets the cost at a grid coordinate position.
    /// 
    /// # Arguments
    /// * `x` - X coordinate in grid frame (cells)
    /// * `y` - Y coordinate in grid frame (cells)
    /// 
    /// # Returns
    /// * `Result<&CellCost, NavigationError>` - The cost at the specified position or an error if out of bounds
    pub fn get_cost(&self, x: usize, y: usize) -> Result<&CellCost, NavigationError> {
        if x < self.width && y < self.height {
            let index = self.get_index(x, y);
            Ok(&self.data[index])
        } else {
            Err(NavigationError::OutOfBounds("Costmap index out of bounds")) // TODO: Should I return the input coordinates in this error?
        }
    }

    /// Gets a mutable reference to the cost at a grid coordinate position.
    /// 
    /// # Arguments
    /// * `x` - X coordinate in grid frame (cells)
    /// * `y` - Y coordinate in grid frame (cells)
    /// 
    /// # Returns
    /// * `Result<&mut CellCost, NavigationError>` - Mutable reference to the cost or an error if out of bounds
    pub fn get_cost_mut(&mut self, x: usize, y: usize) -> Result<&mut CellCost, NavigationError> {
        if x < self.width && y < self.height {
            let index = self.get_index(x, y);
            Ok(&mut self.data[index])
        } else {
            Err(NavigationError::OutOfBounds("Costmap index out of bounds"))
        }
    }

    /// Sets the cost at a grid coordinate position.
    /// 
    /// # Arguments
    /// * `x` - X coordinate in grid frame (cells)
    /// * `y` - Y coordinate in grid frame (cells)
    /// * `cost` - The cost to set at the specified position
    /// 
    /// # Returns
    /// * `Result<(), NavigationError>` - Success or error if out of bounds
    pub fn set_cost(&mut self, x: usize, y: usize, cost: CellCost) -> Result<(), NavigationError> {
        if x < self.width && y < self.height {
            let index = self.get_index(x, y);
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
        for y in 0..self.height {
            for x in 0..self.width {
                if let Ok(cost) = self.get_cost(x, y) {
                    if *cost == CellCost::Lethal {
                        // Inflate this obstacle
                        // Use checked arithmetic to avoid underflow/overflow
                        let start_y = y.saturating_sub(radius_cells);
                        let end_y = (y + radius_cells).min(self.height - 1);
                        let start_x = x.saturating_sub(radius_cells);
                        let end_x = (x + radius_cells).min(self.width - 1);

                        for ny in start_y..=end_y {
                            for nx in start_x..=end_x {
                                let dx = nx as f32 - x as f32;
                                let dy = ny as f32 - y as f32;
                                let distance = (dx * dx + dy * dy).sqrt() * self.resolution;
                                
                                if distance <= radius {
                                    let inflation_cost = ((1.0 - (distance / radius)) * 252.0) as u8;
                                    let idx = self.get_index(nx, ny);
                                    new_data[idx] = CellCost::Inflated(inflation_cost);
                                }
                            }
                        }
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
        for y in 0..self.height {
            for x in 0..self.width {
                if let Ok(cost) = self.get_cost(x, y) {
                    write!(f, "{:3} ", cost.as_u8())?;
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
        let costmap = Costmap2D::new(10, 10, 0.1, 0.0, 0.0).unwrap();
        assert_eq!(costmap.width, 10);
        assert_eq!(costmap.height, 10);
        assert_eq!(costmap.resolution, 0.1);
        assert_eq!(costmap.origin_x, 0.0);
        assert_eq!(costmap.origin_y, 0.0);
    }

    #[test]
    fn test_invalid_creation() {
        assert!(matches!(
            Costmap2D::new(0, 10, 0.1, 0.0, 0.0),
            Err(NavigationError::InvalidDimensions(_))
        ));
        assert!(matches!(
            Costmap2D::new(10, 0, 0.1, 0.0, 0.0),
            Err(NavigationError::InvalidDimensions(_))
        ));
        assert!(matches!(
            Costmap2D::new(10, 10, 0.0, 0.0, 0.0),
            Err(NavigationError::InvalidResolution(_))
        ));
    }

    #[test]
    fn test_cost_operations() {
        let mut costmap = Costmap2D::new(5, 5, 0.1, 0.0, 0.0).unwrap();
        
        // Test setting and getting costs
        costmap.set_cost(2, 2, CellCost::Lethal).unwrap();
        assert_eq!(*costmap.get_cost(2, 2).unwrap(), CellCost::Lethal);
        
        // Test out of bounds
        assert!(matches!(
            costmap.set_cost(5, 2, CellCost::Lethal),
            Err(NavigationError::OutOfBounds(_))
        ));
        assert!(matches!(
            costmap.get_cost(2, 5),
            Err(NavigationError::OutOfBounds(_))
        ));
    }

    #[test]
    fn test_coordinate_conversion() {
        let costmap = Costmap2D::new(10, 10, 0.1, -0.5, -0.5).unwrap();
        
        // Test world to grid conversion
        // World (0.0, 0.0) should map to grid (5, 5) because:
        // grid_x = (0.0 - (-0.5)) / 0.1 = 5
        let (grid_x, grid_y) = costmap.world_to_grid(0.0, 0.0).unwrap();
        assert_eq!(grid_x, 5);
        assert_eq!(grid_y, 5);
        
        // Test grid to world conversion
        // Grid (5, 5) should map to world (0.05, 0.05) because:
        // world_x = -0.5 + (5 + 0.5) * 0.1 = 0.05
        let (world_x, world_y) = costmap.grid_to_world(5, 5).unwrap();
        assert!((world_x - 0.05).abs() < 1e-6);
        assert!((world_y - 0.05).abs() < 1e-6);
        
        // Test some other conversions
        let (grid_x, grid_y) = costmap.world_to_grid(-0.5, -0.5).unwrap();
        assert_eq!(grid_x, 0);
        assert_eq!(grid_y, 0);
        
        let (world_x, world_y) = costmap.grid_to_world(0, 0).unwrap();
        assert!((world_x - (-0.45)).abs() < 1e-6);  // -0.5 + (0 + 0.5) * 0.1
        assert!((world_y - (-0.45)).abs() < 1e-6);
        
        // Test out of bounds
        assert!(costmap.world_to_grid(1.0, 1.0).is_none());
        assert!(costmap.grid_to_world(10, 10).is_none());
    }

    #[test]
    fn test_inflation() {
        let mut costmap = Costmap2D::new(5, 5, 0.1, 0.0, 0.0).unwrap();
        
        // Set a lethal obstacle in the center
        costmap.set_cost(2, 2, CellCost::Lethal).unwrap();
        
        // Inflate with a small radius
        costmap.inflate_obstacles(0.15).unwrap();
        
        // Check that surrounding cells are inflated
        assert!(matches!(
            costmap.get_cost(1, 2).unwrap(),
            CellCost::Inflated(_)
        ));
        assert!(matches!(
            costmap.get_cost(2, 1).unwrap(),
            CellCost::Inflated(_)
        ));
        
        // Check that corner cells are still free
        assert_eq!(*costmap.get_cost(0, 0).unwrap(), CellCost::Free);
    }

    #[test]
    fn test_display() {
        let mut costmap = Costmap2D::new(3, 3, 0.1, 0.0, 0.0).unwrap();
        costmap.set_cost(1, 1, CellCost::Lethal).unwrap();
        
        let display = format!("{}", costmap);
        assert!(display.contains("Costmap2D (3x3"));
        assert!(display.contains("254")); // Lethal cost value
    }
}
