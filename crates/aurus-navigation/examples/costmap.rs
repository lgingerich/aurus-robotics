use aurus_navigation::map::costmap::{Costmap2D, CellCost};
use aurus_navigation::map::{GridPoint, WorldPoint};

fn main() {
    let mut costmap = Costmap2D::new(20, 20, 0.1, WorldPoint::new(-1.0, -1.0)).unwrap();

    // Create a more complex obstacle pattern
    // Main obstacle in the center
    for x in 8..12 {
        for y in 8..12 {
            costmap.set_cost(GridPoint::new(x, y), CellCost::Lethal).unwrap();
        }
    }

    // Add some inscribed areas (guaranteed collision)
    for x in 7..13 {
        for y in 7..13 {
            if costmap.get_cost(GridPoint::new(x, y)).unwrap() == &CellCost::Free {
                costmap.set_cost(GridPoint::new(x, y), CellCost::Inscribed).unwrap();
            }
        }
    }

    // Inflate obstacles with a 0.3m radius
    costmap.inflate_obstacles(0.3).unwrap();

    // Demonstrate world coordinate conversion
    let world_pos_tuple = (0.0, 0.0);
    let world_pos = WorldPoint::new(world_pos_tuple.0, world_pos_tuple.1);
    if let Some(grid_p) = costmap.world_to_grid(world_pos) {
        println!("World position {:?} maps to grid position ({}, {})", world_pos_tuple, grid_p.x, grid_p.y);
        
        // Convert back to world coordinates
        if let Some(back_p) = costmap.grid_to_world(grid_p) {
            println!("Grid position ({}, {}) maps back to world position ({:.2}, {:.2})", 
                    grid_p.x, grid_p.y, back_p.x, back_p.y);
        }
    }

    // Get world bounds
    let (min_p, max_p) = costmap.get_world_bounds();
    println!("\nMap bounds:");
    println!("Min: ({:.2}, {:.2})", min_p.x, min_p.y);
    println!("Max: ({:.2}, {:.2})", max_p.x, max_p.y);

    // Print the costmap with a legend
    println!("\nCostmap visualization (0=Free, 1-252=Inflated, 253=Inscribed, 254=Lethal, 255=Unknown):");
    for y in 0..20 {
        for x in 0..20 {
            let cost = costmap.get_cost(GridPoint::new(x, y)).unwrap();
            print!("{:3} ", cost.as_u8());
        }
        println!();
    }

    // Demonstrate setting and getting costs using world coordinates
    // Let's check a position we know has a lethal cost (center of the map)
    let test_world_pos_tuple = (0.0, 0.0);  // This should be in the center where we set lethal costs
    let test_world_p = WorldPoint::new(test_world_pos_tuple.0, test_world_pos_tuple.1);
    if let Some(grid_p) = costmap.world_to_grid(test_world_p) {
        println!("\nCost at world position {:?} (grid: {}, {}): {}", 
                test_world_pos_tuple, 
                grid_p.x, grid_p.y,
                costmap.get_cost_at_world(test_world_p).unwrap());
    }

    // Let's also check a position we know is free
    let free_world_pos_tuple = (-0.8, -0.8);  // This should be in a free area
    let free_world_p = WorldPoint::new(free_world_pos_tuple.0, free_world_pos_tuple.1);
    if let Some(grid_p) = costmap.world_to_grid(free_world_p) {
        println!("Cost at world position {:?} (grid: {}, {}): {}", 
                free_world_pos_tuple, 
                grid_p.x, grid_p.y,
                costmap.get_cost_at_world(free_world_p).unwrap());
    }

    // Let's check a position in the inflated area
    let inflated_world_pos_tuple = (0.2, 0.1);  // This should be in the inflated area
    let inflated_world_p = WorldPoint::new(inflated_world_pos_tuple.0, inflated_world_pos_tuple.1);
    if let Some(grid_p) = costmap.world_to_grid(inflated_world_p) {
        println!("Cost at world position {:?} (grid: {}, {}): {}", 
                inflated_world_pos_tuple, 
                grid_p.x, grid_p.y,
                costmap.get_cost_at_world(inflated_world_p).unwrap());
    }
}
