//! Example demonstrating seamless integration between aurus-common geometric types
//! and navigation algorithms using generic dimensions.

use aurus_common::{WorldPoint, GridPoint, CoordinateConversion, navigation};

fn main() {
    println!("=== Aurus Common Navigation Integration Example ===\n");

    // Demonstrate 2D coordinate conversion
    println!("2D Coordinate Conversion:");
    let world_2d: WorldPoint<2> = WorldPoint::from([1.5, 2.5]);
    println!("WorldPoint<2>: ({}, {})", world_2d.x, world_2d.y);
    
    let svector_2d = world_2d.to_svector();
    println!("As SVector<f32, 2>: {:?}", svector_2d);
    
    let world_back = WorldPoint::<2>::from_svector(&svector_2d);
    println!("Back to WorldPoint<2>: ({}, {})\n", world_back.x, world_back.y);

    // Demonstrate 3D coordinate conversion
    println!("3D Coordinate Conversion:");
    let world_3d: WorldPoint<3> = WorldPoint::from([1.0, 2.0, 3.0]);
    println!("WorldPoint<3>: ({}, {}, {})", world_3d.x, world_3d.y, world_3d.z);
    
    let svector_3d = world_3d.to_svector();
    println!("As SVector<f32, 3>: {:?}", svector_3d);
    
    let world_back_3d = WorldPoint::<3>::from_svector(&svector_3d);
    println!("Back to WorldPoint<3>: ({}, {}, {})\n", 
             world_back_3d.x, world_back_3d.y, world_back_3d.z);

    // Demonstrate grid point conversion
    println!("Grid Point Conversion:");
    let grid_2d: GridPoint<2> = GridPoint::from([10, 20]);
    println!("GridPoint<2>: ({}, {})", grid_2d.x, grid_2d.y);
    
    let grid_svector = grid_2d.to_svector();
    println!("As SVector<usize, 2>: {:?}", grid_svector);
    
    let grid_back = GridPoint::<2>::from_svector(&grid_svector);
    println!("Back to GridPoint<2>: ({}, {})\n", grid_back.x, grid_back.y);

    // Demonstrate path conversion utilities
    println!("Path Conversion Utilities:");
    let world_path = vec![
        WorldPoint::<2>::from([0.0, 0.0]),
        WorldPoint::<2>::from([1.0, 1.0]),
        WorldPoint::<2>::from([2.0, 0.0]),
    ];
    
    println!("Original world path:");
    for (i, point) in world_path.iter().enumerate() {
        println!("  Point {}: ({}, {})", i, point.x, point.y);
    }
    
    // Convert to SVector path (for use with navigation algorithms)
    let svector_path = navigation::world_points_to_svector_path(&world_path);
    println!("\nAs SVector path (for navigation algorithms):");
    for (i, svec) in svector_path.iter().enumerate() {
        println!("  SVector {}: {:?}", i, svec);
    }
    
    // Convert back to world points
    let world_path_back = navigation::svector_path_to_world_points(&svector_path);
    println!("\nConverted back to world points:");
    for (i, point) in world_path_back.iter().enumerate() {
        println!("  Point {}: ({}, {})", i, point.x, point.y);
    }

    // Demonstrate how this would work with costmap dimensions
    println!("\n=== Integration with Navigation Algorithms ===");
    println!("This system allows seamless integration with:");
    println!("- CostMap<D> for any dimension D");
    println!("- A* pathfinding algorithms that work with SVector<T, D>");
    println!("- Automatic conversion between WorldPoint<D>, GridPoint<D>, and SVector<T, D>");
    println!("- Type-safe operations that preserve dimensionality");
    
    println!("\nExample workflow:");
    println!("1. Define world coordinates as WorldPoint<D>");
    println!("2. Convert to SVector for costmap operations");
    println!("3. Run A* pathfinding with SVector coordinates");
    println!("4. Convert result back to WorldPoint<D> for application use");
    
    println!("\nAll conversions are zero-cost and maintain type safety!");
} 