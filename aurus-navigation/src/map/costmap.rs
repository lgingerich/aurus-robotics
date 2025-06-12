//! Cost map implementation for navigation and path planning.
//!
//! This module provides a generic n-dimensional cost map structure that can be used
//! for representing occupancy grids, obstacle maps, and cost fields for path planning
//! algorithms. The cost map supports various cell types including free space, obstacles,
//! inflated costs, and unknown areas.

#![warn(missing_docs)]

// NOTES / TODO:
// - Not no_std compatible
// - Does not support negative coordinates (is this needed?)

use crate::error::NavigationError;
use nalgebra::SVector;

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

/// Generic *n*-dimensional cost-map.
///
/// `D` is the dimensionality (2 for 2-D, 3 for 3-D, …).
#[derive(Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CostMap<const D: usize> {
    /// Size in cells along each axis (x, y, z, …).
    dims: SVector<usize, D>,
    /// Resolution (world meters per cell) along each axis.
    resolution: SVector<f32, D>,
    /// Origin of cell (0, 0, …) in world coordinates.
    origin: SVector<f32, D>,
    /// Vector storing the cost values for each cell
    data: Vec<CellCost>,
}

impl<const D: usize> CostMap<D> {
    /// Creates a new n-dimensional CostMap with the specified dimensions and resolution.
    ///
    /// # Arguments
    /// * `dims` - Size in cells along each axis
    /// * `resolution` - Resolution (world meters per cell) along each axis
    /// * `origin` - Origin of cell (0, 0, …) in world coordinates
    ///
    /// # Returns
    /// * `Result<Self, NavigationError>` - The created CostMap or an error if parameters are invalid
    pub fn new(
        dims: SVector<usize, D>,
        resolution: SVector<f32, D>,
        origin: SVector<f32, D>,
    ) -> Result<Self, NavigationError> {
        // Validate dimensions
        for i in 0..D {
            if dims[i] == 0 {
                return Err(NavigationError::InvalidDimensions(
                    "All dimensions must be non-zero",
                ));
            }
            if resolution[i] <= 0.0 {
                return Err(NavigationError::InvalidResolution(
                    "All resolutions must be positive",
                ));
            }
        }

        // Check for potential overflow in data allocation
        let total_cells = dims
            .iter()
            .try_fold(1usize, |acc, &dim| acc.checked_mul(dim))
            .ok_or(NavigationError::InvalidDimensions(
                "Map dimensions too large, would cause overflow",
            ))?;

        Ok(CostMap {
            dims,
            resolution,
            origin,
            data: vec![CellCost::Free; total_cells],
        })
    }

    /// Gets the dimensions of the costmap.
    pub fn get_dims(&self) -> &SVector<usize, D> {
        &self.dims
    }

    /// Gets the resolution of the costmap.
    pub fn get_resolution(&self) -> &SVector<f32, D> {
        &self.resolution
    }

    /// Gets the origin of the costmap.
    pub fn get_origin(&self) -> &SVector<f32, D> {
        &self.origin
    }

    /// Calculates the linear index for n-dimensional coordinates.
    fn get_index(&self, coords: &SVector<usize, D>) -> usize {
        let mut index = 0;
        let mut stride = 1;

        for i in 0..D {
            index += coords[i] * stride;
            stride *= self.dims[i];
        }

        index
    }

    /// Converts world coordinates to grid coordinates.
    /// Returns None if the world coordinates are outside the map bounds.
    pub fn world_to_grid(&self, world_coords: &SVector<f32, D>) -> Option<SVector<usize, D>> {
        let mut grid_coords = SVector::<usize, D>::zeros();

        for i in 0..D {
            let grid_f = (world_coords[i] - self.origin[i]) / self.resolution[i];

            if grid_f < 0.0 || grid_f >= self.dims[i] as f32 {
                return None;
            }

            grid_coords[i] = grid_f.floor() as usize;

            if grid_coords[i] >= self.dims[i] {
                return None;
            }
        }

        Some(grid_coords)
    }

    /// Converts grid coordinates to world coordinates (center of cell).
    pub fn grid_to_world(&self, grid_coords: &SVector<usize, D>) -> Option<SVector<f32, D>> {
        // Check bounds
        for i in 0..D {
            if grid_coords[i] >= self.dims[i] {
                return None;
            }
        }

        let mut world_coords = SVector::<f32, D>::zeros();
        for i in 0..D {
            world_coords[i] = self.origin[i] + (grid_coords[i] as f32 + 0.5) * self.resolution[i];
        }

        Some(world_coords)
    }

    /// Gets the cost at grid coordinates.
    pub fn get_cost(&self, grid_coords: &SVector<usize, D>) -> Result<&CellCost, NavigationError> {
        // Check bounds
        for i in 0..D {
            if grid_coords[i] >= self.dims[i] {
                return Err(NavigationError::OutOfBounds(
                    "Grid coordinates out of bounds",
                ));
            }
        }

        let index = self.get_index(grid_coords);
        Ok(&self.data[index])
    }

    /// Sets the cost at grid coordinates.
    pub fn set_cost(
        &mut self,
        grid_coords: &SVector<usize, D>,
        cost: CellCost,
    ) -> Result<(), NavigationError> {
        // Check bounds
        for i in 0..D {
            if grid_coords[i] >= self.dims[i] {
                return Err(NavigationError::OutOfBounds(
                    "Grid coordinates out of bounds",
                ));
            }
        }

        let index = self.get_index(grid_coords);
        self.data[index] = cost;
        Ok(())
    }

    /// Gets the cost at world coordinates.
    pub fn get_cost_at_world(
        &self,
        world_coords: &SVector<f32, D>,
    ) -> Result<&CellCost, NavigationError> {
        if let Some(grid_coords) = self.world_to_grid(world_coords) {
            self.get_cost(&grid_coords)
        } else {
            Err(NavigationError::InvalidWorldCoordinates(
                "World coordinates outside map bounds",
            ))
        }
    }

    /// Sets the cost at world coordinates.
    pub fn set_cost_at_world(
        &mut self,
        world_coords: &SVector<f32, D>,
        cost: CellCost,
    ) -> Result<(), NavigationError> {
        if let Some(grid_coords) = self.world_to_grid(world_coords) {
            self.set_cost(&grid_coords, cost)
        } else {
            Err(NavigationError::InvalidWorldCoordinates(
                "World coordinates outside map bounds",
            ))
        }
    }

    /// Gets a reference to the underlying cost data.
    pub fn get_data(&self) -> &[CellCost] {
        &self.data
    }

    /// Gets a mutable reference to the underlying cost data.
    pub fn get_data_mut(&mut self) -> &mut [CellCost] {
        &mut self.data
    }

    /// Clears the entire costmap, setting all cells to Free.
    pub fn clear(&mut self) {
        self.data.fill(CellCost::Free);
    }

    /// Sets all cells to Unknown.
    pub fn reset_to_unknown(&mut self) {
        self.data.fill(CellCost::Unknown);
    }

    /// Gets the total number of cells in the costmap.
    pub fn total_cells(&self) -> usize {
        self.data.len()
    }

    /// Inflates obstacles in the costmap by a specified radius.
    /// This creates a safety margin around obstacles.
    ///
    /// # Arguments
    /// * `radius` - Inflation radius in meters (applied uniformly across all dimensions)
    ///
    /// # Returns
    /// * `Result<(), NavigationError>` - Success or error if radius is invalid
    pub fn inflate_obstacles(&mut self, radius: f32) -> Result<(), NavigationError> {
        if radius <= 0.0 {
            return Err(NavigationError::InvalidResolution(
                "Inflation radius must be positive",
            ));
        }

        let mut new_data = self.data.clone();

        // Iterate through all cells
        let mut current_coords = SVector::<usize, D>::zeros();

        loop {
            if let Ok(cost_val) = self.get_cost(&current_coords) {
                if *cost_val == CellCost::Lethal {
                    // Inflate this obstacle
                    self.inflate_around_point(&mut new_data, &current_coords, radius)?;
                }
            }

            // Increment coordinates (n-dimensional counter)
            if !self.increment_coords(&mut current_coords) {
                break;
            }
        }

        // Second pass: ensure original lethal obstacles remain lethal
        current_coords = SVector::<usize, D>::zeros();
        loop {
            if let Ok(original_cost) = self.get_cost(&current_coords) {
                if *original_cost == CellCost::Lethal {
                    let index = self.get_index(&current_coords);
                    new_data[index] = CellCost::Lethal;
                }
            }

            if !self.increment_coords(&mut current_coords) {
                break;
            }
        }

        self.data = new_data;
        Ok(())
    }

    /// Helper function to inflate around a specific point.
    fn inflate_around_point(
        &self,
        data: &mut [CellCost],
        center: &SVector<usize, D>,
        radius: f32,
    ) -> Result<(), NavigationError> {
        // Calculate radius in cells for each dimension
        let mut radius_cells = SVector::<usize, D>::zeros();
        for i in 0..D {
            radius_cells[i] = (radius / self.resolution[i]).ceil() as usize;
        }

        // Calculate bounds for iteration
        let mut start_coords = SVector::<usize, D>::zeros();
        let mut end_coords = SVector::<usize, D>::zeros();

        for i in 0..D {
            start_coords[i] = center[i].saturating_sub(radius_cells[i]);
            end_coords[i] = (center[i] + radius_cells[i]).min(self.dims[i] - 1);
        }

        // Initialize neighbor_coords to start_coords
        let mut neighbor_coords = start_coords;

        loop {
            // Calculate distance from center
            let mut distance_sq = 0.0f32;
            for i in 0..D {
                let diff = (neighbor_coords[i] as f32 - center[i] as f32) * self.resolution[i];
                distance_sq += diff * diff;
            }

            let distance = distance_sq.sqrt();

            if distance <= radius {
                // Calculate inflation cost based on distance
                let inflation_factor = 1.0 - (distance / radius);
                let raw_inflation_cost = inflation_factor * 252.0;
                let inflation_u8 = raw_inflation_cost.max(1.0).min(252.0) as u8;

                let idx = self.get_index(&neighbor_coords);

                // Only update if new cost is higher or current is Free
                match data[idx] {
                    CellCost::Free => {
                        data[idx] = CellCost::Inflated(inflation_u8);
                    }
                    CellCost::Inflated(current_inflation) => {
                        if inflation_u8 > current_inflation {
                            data[idx] = CellCost::Inflated(inflation_u8);
                        }
                    }
                    _ => {
                        data[idx] = CellCost::Inflated(inflation_u8);
                    }
                }
            }

            // Increment neighbor_coords within bounds
            if !self.increment_coords_bounded(&mut neighbor_coords, &start_coords, &end_coords) {
                break;
            }
        }

        Ok(())
    }

    /// Helper function to increment n-dimensional coordinates.
    /// Returns false when all coordinates have been exhausted.
    fn increment_coords(&self, coords: &mut SVector<usize, D>) -> bool {
        for i in 0..D {
            coords[i] += 1;
            if coords[i] < self.dims[i] {
                return true;
            }
            coords[i] = 0;
        }
        false
    }

    /// Helper function to increment n-dimensional coordinates within bounds.
    fn increment_coords_bounded(
        &self,
        coords: &mut SVector<usize, D>,
        start: &SVector<usize, D>,
        end: &SVector<usize, D>,
    ) -> bool {
        for i in 0..D {
            coords[i] += 1;
            if coords[i] <= end[i] {
                return true;
            }
            coords[i] = start[i];
        }
        false
    }
}

// Type aliases for common dimensions
/// 2D costmap using the generic implementation.
pub type CostMap2D = CostMap<2>;

/// 3D costmap using the generic implementation.
pub type CostMap3D = CostMap<3>;

impl std::fmt::Display for CostMap2D {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            "CostMap2D ({}x{}, resolution: {:.3}m)",
            self.dims[0], self.dims[1], self.resolution[0]
        )?;
        writeln!(f, "Origin: ({:.3}, {:.3})", self.origin[0], self.origin[1])?;

        // Print the costmap grid
        for y_idx in 0..self.dims[1] {
            for x_idx in 0..self.dims[0] {
                let coords = SVector::<usize, 2>::new(x_idx, y_idx);
                if let Ok(cost_val) = self.get_cost(&coords) {
                    write!(f, "{:3} ", cost_val.as_u8())?;
                }
            }
            writeln!(f)?;
        }
        Ok(())
    }
}

impl std::fmt::Display for CostMap3D {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            "CostMap3D ({}x{}x{}, resolution: {:.3}x{:.3}x{:.3}m)",
            self.dims[0],
            self.dims[1],
            self.dims[2],
            self.resolution[0],
            self.resolution[1],
            self.resolution[2]
        )?;
        writeln!(
            f,
            "Origin: ({:.3}, {:.3}, {:.3})",
            self.origin[0], self.origin[1], self.origin[2]
        )?;
        writeln!(f)?;

        // Print each Z-layer separately
        for z_idx in 0..self.dims[2] {
            writeln!(
                f,
                "Layer Z={} (height: {:.3}m):",
                z_idx,
                self.origin[2] + (z_idx as f32 + 0.5) * self.resolution[2]
            )?;

            // Print the grid for this Z-layer
            for y_idx in 0..self.dims[1] {
                for x_idx in 0..self.dims[0] {
                    let coords = SVector::<usize, 3>::new(x_idx, y_idx, z_idx);
                    if let Ok(cost_val) = self.get_cost(&coords) {
                        write!(f, "{:3} ", cost_val.as_u8())?;
                    }
                }
                writeln!(f)?;
            }
            writeln!(f)?; // Extra line between layers
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_costmap_creation() {
        let dims = SVector::<usize, 2>::new(10, 10);
        let resolution = SVector::<f32, 2>::new(0.1, 0.1);
        let origin = SVector::<f32, 2>::new(0.0, 0.0);
        let costmap = CostMap2D::new(dims, resolution, origin).unwrap();
        assert_eq!(costmap.dims[0], 10);
        assert_eq!(costmap.dims[1], 10);
        assert_eq!(costmap.resolution[0], 0.1);
        assert_eq!(costmap.origin[0], 0.0);
    }

    #[test]
    fn test_invalid_creation() {
        let resolution = SVector::<f32, 2>::new(0.1, 0.1);
        let origin = SVector::<f32, 2>::new(0.0, 0.0);

        assert!(matches!(
            CostMap2D::new(SVector::<usize, 2>::new(0, 10), resolution, origin),
            Err(NavigationError::InvalidDimensions(_))
        ));
        assert!(matches!(
            CostMap2D::new(SVector::<usize, 2>::new(10, 0), resolution, origin),
            Err(NavigationError::InvalidDimensions(_))
        ));
        assert!(matches!(
            CostMap2D::new(
                SVector::<usize, 2>::new(10, 10),
                SVector::<f32, 2>::new(0.0, 0.1),
                origin
            ),
            Err(NavigationError::InvalidResolution(_))
        ));
    }

    #[test]
    fn test_cost_operations() {
        let dims = SVector::<usize, 2>::new(5, 5);
        let resolution = SVector::<f32, 2>::new(0.1, 0.1);
        let origin = SVector::<f32, 2>::new(0.0, 0.0);
        let mut costmap = CostMap2D::new(dims, resolution, origin).unwrap();
        let coords = SVector::<usize, 2>::new(2, 2);

        // Test setting and getting costs
        costmap.set_cost(&coords, CellCost::Lethal).unwrap();
        assert_eq!(*costmap.get_cost(&coords).unwrap(), CellCost::Lethal);

        // Test out of bounds
        let out_of_bounds_1 = SVector::<usize, 2>::new(5, 2);
        let out_of_bounds_2 = SVector::<usize, 2>::new(2, 5);
        assert!(matches!(
            costmap.set_cost(&out_of_bounds_1, CellCost::Lethal),
            Err(NavigationError::OutOfBounds(_))
        ));
        assert!(matches!(
            costmap.get_cost(&out_of_bounds_2),
            Err(NavigationError::OutOfBounds(_))
        ));
    }

    #[test]
    fn test_coordinate_conversion() {
        let dims = SVector::<usize, 2>::new(10, 10);
        let resolution = SVector::<f32, 2>::new(0.1, 0.1);
        let origin = SVector::<f32, 2>::new(-0.5, -0.5);
        let costmap = CostMap2D::new(dims, resolution, origin).unwrap();

        // World (0.0, 0.0) should map to grid (5, 5)
        let world_coords = SVector::<f32, 2>::new(0.0, 0.0);
        let grid_coords = costmap.world_to_grid(&world_coords).unwrap();
        assert_eq!(grid_coords, SVector::<usize, 2>::new(5, 5));

        // Grid (5, 5) should map to world center of cell (0.05, 0.05)
        let grid_coords = SVector::<usize, 2>::new(5, 5);
        let world_coords = costmap.grid_to_world(&grid_coords).unwrap();
        assert!((world_coords[0] - 0.05).abs() < 1e-6);
        assert!((world_coords[1] - 0.05).abs() < 1e-6);

        // Test out of bounds
        assert!(
            costmap
                .world_to_grid(&SVector::<f32, 2>::new(1.0, 1.0))
                .is_none()
        );
        assert!(
            costmap
                .grid_to_world(&SVector::<usize, 2>::new(10, 10))
                .is_none()
        );
    }

    #[test]
    fn test_display_2d() {
        let dims = SVector::<usize, 2>::new(3, 3);
        let resolution = SVector::<f32, 2>::new(0.1, 0.1);
        let origin = SVector::<f32, 2>::new(0.0, 0.0);
        let mut costmap = CostMap2D::new(dims, resolution, origin).unwrap();
        let coords = SVector::<usize, 2>::new(1, 1);
        costmap.set_cost(&coords, CellCost::Lethal).unwrap();

        let display_str = format!("{}", costmap);
        assert!(display_str.contains("CostMap2D (3x3"));
        assert!(display_str.contains("254")); // Lethal cost value
    }

    #[test]
    fn test_display_3d() {
        let dims = SVector::<usize, 3>::new(2, 2, 2);
        let resolution = SVector::<f32, 3>::new(0.1, 0.1, 0.2);
        let origin = SVector::<f32, 3>::new(0.0, 0.0, 0.0);
        let mut costmap = CostMap3D::new(dims, resolution, origin).unwrap();

        // Set a lethal cost in the first layer
        let coords_layer0 = SVector::<usize, 3>::new(0, 0, 0);
        costmap.set_cost(&coords_layer0, CellCost::Lethal).unwrap();

        // Set an inflated cost in the second layer
        let coords_layer1 = SVector::<usize, 3>::new(1, 1, 1);
        costmap
            .set_cost(&coords_layer1, CellCost::Inflated(100))
            .unwrap();

        let display_str = format!("{}", costmap);
        assert!(display_str.contains("CostMap3D (2x2x2"));
        assert!(display_str.contains("0.100x0.100x0.200m")); // Resolution
        assert!(display_str.contains("Layer Z=0"));
        assert!(display_str.contains("Layer Z=1"));
        assert!(display_str.contains("254")); // Lethal cost value
        assert!(display_str.contains("100")); // Inflated cost value
    }
}
