/// Represents a point in grid coordinates (cell indices).
#[derive(Debug, Default, Copy, Clone, PartialEq, Eq, Hash)]
pub struct GridPoint {
    /// The x-coordinate (column index) in the grid.
    pub x: usize,
    /// The y-coordinate (row index) in the grid.
    pub y: usize,
}

impl GridPoint {
    /// Creates a new `GridPoint`.
    #[must_use]
    pub const fn new(x: usize, y: usize) -> Self {
        Self { x, y }
    }
}

/// Represents a point in world coordinates (meters).
#[derive(Debug, Default, Clone, Copy, PartialEq, PartialOrd)]
pub struct WorldPoint {
    /// The x-coordinate in meters.
    pub x: f32,
    /// The y-coordinate in meters.
    pub y: f32,
}

impl WorldPoint {
    /// Creates a new `WorldPoint`.
    #[must_use]
    pub const fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }
} 