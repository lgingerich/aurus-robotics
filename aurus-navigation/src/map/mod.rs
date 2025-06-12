//! Map-related functionality for navigation.
//!
//! This module provides costmap implementations and related utilities
//! for representing and working with occupancy grids and cost maps.

pub mod costmap;

pub use costmap::{CostMap, CostMap2D, CostMap3D};
