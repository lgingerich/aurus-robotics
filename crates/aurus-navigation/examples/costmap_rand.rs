use aurus_navigation::map::costmap::{Costmap2D};
use aurus_navigation::map::{GridPoint, WorldPoint};

fn main() {
    // Create a 20x20 costmap with 0.1m resolution, centered at (-1.0, -1.0) in world coordinates,
    // with 15 random obstacles, each with a max dimension of 4 cells.
    let mut costmap = Costmap2D::new_rand(20, 20, 0.1, WorldPoint::new(-1.0, -1.0), 15, 2).unwrap();

    println!("Costmap after random generation:");
    println!("{}", costmap);

    // Inflate obstacles with a 0.2m radius (2 cells)
    match costmap.inflate_obstacles(0.2) {
        Ok(_) => println!("\nObstacles inflated with 0.2m radius."),
        Err(e) => println!("Error inflating obstacles: {:?}", e),
    }

    println!("\nCostmap after inflation:");
    println!("{}", costmap);

    // Demonstrate world coordinate conversion
    let world_pos_tuple = (-0.5, -0.5); // A point within the map
    let world_pos = WorldPoint::new(world_pos_tuple.0, world_pos_tuple.1);
    if let Some(grid_p) = costmap.world_to_grid(world_pos) {
        println!("\nWorld position {:?} maps to grid position ({}, {})", world_pos_tuple, grid_p.x, grid_p.y);
        
        // Convert back to world coordinates (center of the cell)
        if let Some(back_p) = costmap.grid_to_world(grid_p) {
            println!("Grid position ({}, {}) maps back to world position ({:.2}, {:.2})", 
                    grid_p.x, grid_p.y, back_p.x, back_p.y);
        }
    } else {
        println!("\nWorld position {:?} is outside map bounds.", world_pos_tuple);
    }

    // Get world bounds
    let (min_p, max_p) = costmap.get_world_bounds();
    println!("\nMap bounds:");
    println!("Min world coordinates: ({:.2}, {:.2})", min_p.x, min_p.y);
    println!("Max world coordinates: ({:.2}, {:.2})", max_p.x, max_p.y);

    // Demonstrate setting and getting costs using world coordinates
    // Note: Obstacle positions are random, so the exact cost at these points will vary.
    let example_world_points = [
        WorldPoint::new(-0.9, -0.9), // Near origin
        WorldPoint::new(0.0, 0.0),   // Center of the map area (if origin is -1, -1)
        WorldPoint::new(0.8, 0.8),   // Towards the other end
    ];

    println!("\nCosts at specific world positions (will vary due to random obstacles):");
    for (i, &p) in example_world_points.iter().enumerate() {
        match costmap.get_cost_at_world(p) {
            Ok(cost) => {
                if let Some(gp) = costmap.world_to_grid(p) {
                    println!("Point {}: {:?} (grid: {},{}) -> Cost: {}", i, (p.x, p.y), gp.x, gp.y, cost);
                } else {
                    println!("Point {}: {:?} (out of bounds) -> Cost: {}", i, (p.x, p.y), cost);
                }
            }
            Err(e) => println!("Point {}: {:?} -> Error: {:?}", i, (p.x, p.y), e),
        }
    }
}
