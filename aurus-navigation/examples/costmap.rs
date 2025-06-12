use aurus_navigation::map::costmap::{CellCost, CostMap};
use aurus_common::{GridPoint, WorldPoint};
use nalgebra::SVector;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create a 20x20 costmap with 0.1m resolution
    let dims = SVector::<usize, 2>::new(20, 20);
    let resolution = SVector::<f32, 2>::new(0.1, 0.1);
    let origin = SVector::<f32, 2>::new(-1.0, -1.0);
    let mut costmap = CostMap::<2>::new(dims, resolution, origin)?;

    // Create a more complex obstacle pattern
    // Main obstacle in the center
    for x in 8..12 {
        for y in 8..12 {
            let coords = SVector::<usize, 2>::new(x, y);
            costmap.set_cost(&coords, CellCost::Lethal)?;
        }
    }

    // Add some inscribed areas (guaranteed collision)
    for x in 7..13 {
        for y in 7..13 {
            let coords = SVector::<usize, 2>::new(x, y);
            if matches!(costmap.get_cost(&coords)?, CellCost::Free) {
                costmap.set_cost(&coords, CellCost::Inscribed)?;
            }
        }
    }

    // Inflate obstacles with a 0.3m radius
    costmap.inflate_obstacles(0.3)?;

    // Demonstrate world coordinate conversion
    let world_pos_tuple = (0.0, 0.0);
    let world_pos = SVector::<f32, 2>::new(world_pos_tuple.0, world_pos_tuple.1);
    if let Some(grid_p) = costmap.world_to_grid(&world_pos) {
        println!(
            "World position {:?} maps to grid position ({}, {})",
            world_pos_tuple, grid_p[0], grid_p[1]
        );

        // Convert back to world coordinates
        if let Some(back_p) = costmap.grid_to_world(&grid_p) {
            println!(
                "Grid position ({}, {}) maps back to world position ({:.2}, {:.2})",
                grid_p[0], grid_p[1], back_p[0], back_p[1]
            );
        }
    }

    // Calculate world bounds manually
    let dims = costmap.get_dims();
    let origin = costmap.get_origin();
    let resolution = costmap.get_resolution();
    
    let min_p = *origin;
    let mut max_p = SVector::<f32, 2>::zeros();
    for i in 0..2 {
        max_p[i] = origin[i] + (dims[i] as f32) * resolution[i];
    }
    
    println!("\nMap bounds:");
    println!("Min: ({:.2}, {:.2})", min_p[0], min_p[1]);
    println!("Max: ({:.2}, {:.2})", max_p[0], max_p[1]);

    // Print the costmap with a legend
    println!(
        "\nCostmap visualization (. = Free, I = Inflated, # = Inscribed, X = Lethal, ? = Unknown):"
    );
    let dims = costmap.get_dims();
    for y in (0..dims[1]).rev() {
        print!("{:2} ", y);
        for x in 0..dims[0] {
            let coords = SVector::<usize, 2>::new(x, y);
            let cost = costmap.get_cost(&coords)?;
            match cost {
                CellCost::Free => print!(". "),
                CellCost::Inflated(_) => print!("I "),
                CellCost::Inscribed => print!("# "),
                CellCost::Lethal => print!("X "),
                CellCost::Unknown => print!("? "),
            }
        }
        println!();
    }
    print!("   ");
    for x in 0..dims[0] {
        print!("{} ", x % 10);
    }
    println!();

    // Demonstrate setting and getting costs using world coordinates
    // Let's check a position we know has a lethal cost (center of the map)
    let test_world_pos_tuple = (0.0, 0.0); // This should be in the center where we set lethal costs
    let test_world_p = SVector::<f32, 2>::new(test_world_pos_tuple.0, test_world_pos_tuple.1);
    if let Some(grid_p) = costmap.world_to_grid(&test_world_p) {
        println!(
            "\nCost at world position {:?} (grid: {}, {}): {:?}",
            test_world_pos_tuple,
            grid_p[0],
            grid_p[1],
            costmap.get_cost(&grid_p)?
        );
    }

    // Let's also check a position we know is free
    let free_world_pos_tuple = (-0.8, -0.8); // This should be in a free area
    let free_world_p = SVector::<f32, 2>::new(free_world_pos_tuple.0, free_world_pos_tuple.1);
    if let Some(grid_p) = costmap.world_to_grid(&free_world_p) {
        println!(
            "Cost at world position {:?} (grid: {}, {}): {:?}",
            free_world_pos_tuple,
            grid_p[0],
            grid_p[1],
            costmap.get_cost(&grid_p)?
        );
    }

    // Let's check a position in the inflated area
    let inflated_world_pos_tuple = (0.2, 0.1); // This should be in the inflated area
    let inflated_world_p = SVector::<f32, 2>::new(inflated_world_pos_tuple.0, inflated_world_pos_tuple.1);
    if let Some(grid_p) = costmap.world_to_grid(&inflated_world_p) {
        println!(
            "Cost at world position {:?} (grid: {}, {}): {:?}",
            inflated_world_pos_tuple,
            grid_p[0],
            grid_p[1],
            costmap.get_cost(&grid_p)?
        );
    }

    Ok(())
}
