use aurus_navigation::map::costmap::{Costmap2D, CellCost};

fn main() {
    // Create a 20x20 costmap with 0.1m resolution, centered at (0,0)
    let mut costmap = Costmap2D::new(20, 20, 0.1, -1.0, -1.0).unwrap();

    // Create a more complex obstacle pattern
    // Main obstacle in the center
    for x in 8..12 {
        for y in 8..12 {
            costmap.set_cost(x, y, CellCost::Lethal).unwrap();
        }
    }

    // Add some inscribed areas (guaranteed collision)
    for x in 7..13 {
        for y in 7..13 {
            if costmap.get_cost(x, y).unwrap() == &CellCost::Free {
                costmap.set_cost(x, y, CellCost::Inscribed).unwrap();
            }
        }
    }

    // Inflate obstacles with a 0.3m radius
    costmap.inflate_obstacles(0.3).unwrap();

    // Demonstrate world coordinate conversion
    let world_pos = (0.0, 0.0);
    if let Some((grid_x, grid_y)) = costmap.world_to_grid(world_pos.0, world_pos.1) {
        println!("World position {:?} maps to grid position ({}, {})", world_pos, grid_x, grid_y);
        
        // Convert back to world coordinates
        if let Some((back_x, back_y)) = costmap.grid_to_world(grid_x, grid_y) {
            println!("Grid position ({}, {}) maps back to world position ({:.2}, {:.2})", 
                    grid_x, grid_y, back_x, back_y);
        }
    }

    // Get world bounds
    let ((min_x, min_y), (max_x, max_y)) = costmap.get_world_bounds();
    println!("\nMap bounds:");
    println!("Min: ({:.2}, {:.2})", min_x, min_y);
    println!("Max: ({:.2}, {:.2})", max_x, max_y);

    // Print the costmap with a legend
    println!("\nCostmap visualization (0=Free, 1-252=Inflated, 253=Inscribed, 254=Lethal, 255=Unknown):");
    for y in 0..20 {
        for x in 0..20 {
            let cost = costmap.get_cost(x, y).unwrap();
            print!("{:3} ", cost.as_u8());
        }
        println!();
    }

    // Demonstrate setting and getting costs using world coordinates
    // Let's check a position we know has a lethal cost (center of the map)
    let test_world_pos = (0.0, 0.0);  // This should be in the center where we set lethal costs
    if let Some((grid_x, grid_y)) = costmap.world_to_grid(test_world_pos.0, test_world_pos.1) {
        println!("\nCost at world position {:?} (grid: {}, {}): {}", 
                test_world_pos, 
                grid_x, grid_y,
                costmap.get_cost_at_world(test_world_pos.0, test_world_pos.1).unwrap());
    }

    // Let's also check a position we know is free
    let free_world_pos = (-0.8, -0.8);  // This should be in a free area
    if let Some((grid_x, grid_y)) = costmap.world_to_grid(free_world_pos.0, free_world_pos.1) {
        println!("Cost at world position {:?} (grid: {}, {}): {}", 
                free_world_pos, 
                grid_x, grid_y,
                costmap.get_cost_at_world(free_world_pos.0, free_world_pos.1).unwrap());
    }

    // Let's check a position in the inflated area
    let inflated_world_pos = (0.2, 0.1);  // This should be in the inflated area
    if let Some((grid_x, grid_y)) = costmap.world_to_grid(inflated_world_pos.0, inflated_world_pos.1) {
        println!("Cost at world position {:?} (grid: {}, {}): {}", 
                inflated_world_pos, 
                grid_x, grid_y,
                costmap.get_cost_at_world(inflated_world_pos.0, inflated_world_pos.1).unwrap());
    }
}
