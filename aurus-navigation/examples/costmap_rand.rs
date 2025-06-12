use aurus_navigation::map::costmap::{CellCost, CostMap};
use nalgebra::SVector;
use rand::Rng;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create a 20x20 costmap with 0.1m resolution
    let dims = SVector::<usize, 2>::new(20, 20);
    let resolution = SVector::<f32, 2>::new(0.1, 0.1);
    let origin = SVector::<f32, 2>::new(-1.0, -1.0);
    let mut costmap = CostMap::<2>::new(dims, resolution, origin)?;

    // Create random obstacles manually
    let mut rng = rand::rng();
    let num_obstacles = 15;
    let max_obstacle_size = 2;

    println!("Generating {} random obstacles...", num_obstacles);
    for _ in 0..num_obstacles {
        // Random starting position
        let start_x = rng.random_range(0..dims[0]);
        let start_y = rng.random_range(0..dims[1]);

        // Random obstacle size
        let width = rng.random_range(1..=max_obstacle_size);
        let height = rng.random_range(1..=max_obstacle_size);

        // Create obstacle
        for x in start_x..std::cmp::min(start_x + width, dims[0]) {
            for y in start_y..std::cmp::min(start_y + height, dims[1]) {
                let coords = SVector::<usize, 2>::new(x, y);
                costmap.set_cost(&coords, CellCost::Lethal)?;
            }
        }
    }

    println!("Costmap after random generation:");
    print_costmap(&costmap);

    // Inflate obstacles with a 0.2m radius
    match costmap.inflate_obstacles(0.2) {
        Ok(_) => println!("\nObstacles inflated with 0.2m radius."),
        Err(e) => println!("Error inflating obstacles: {:?}", e),
    }

    println!("\nCostmap after inflation:");
    print_costmap(&costmap);

    // Demonstrate world coordinate conversion
    let world_pos_tuple = (-0.5, -0.5); // A point within the map
    let world_pos = SVector::<f32, 2>::new(world_pos_tuple.0, world_pos_tuple.1);
    if let Some(grid_p) = costmap.world_to_grid(&world_pos) {
        println!(
            "\nWorld position {:?} maps to grid position ({}, {})",
            world_pos_tuple, grid_p[0], grid_p[1]
        );

        // Convert back to world coordinates (center of the cell)
        if let Some(back_p) = costmap.grid_to_world(&grid_p) {
            println!(
                "Grid position ({}, {}) maps back to world position ({:.2}, {:.2})",
                grid_p[0], grid_p[1], back_p[0], back_p[1]
            );
        }
    } else {
        println!(
            "\nWorld position {:?} is outside map bounds.",
            world_pos_tuple
        );
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
    println!("Min world coordinates: ({:.2}, {:.2})", min_p[0], min_p[1]);
    println!("Max world coordinates: ({:.2}, {:.2})", max_p[0], max_p[1]);

    // Demonstrate setting and getting costs using world coordinates
    // Note: Obstacle positions are random, so the exact cost at these points will vary.
    let example_world_points = [
        SVector::<f32, 2>::new(-0.9, -0.9), // Near origin
        SVector::<f32, 2>::new(0.0, 0.0),   // Center of the map area (if origin is -1, -1)
        SVector::<f32, 2>::new(0.8, 0.8),   // Towards the other end
    ];

    println!("\nCosts at specific world positions (will vary due to random obstacles):");
    for (i, &p) in example_world_points.iter().enumerate() {
        match costmap.get_cost_at_world(&p) {
            Ok(cost) => {
                if let Some(gp) = costmap.world_to_grid(&p) {
                    println!(
                        "Point {}: {:?} (grid: {},{}) -> Cost: {:?}",
                        i,
                        (p[0], p[1]),
                        gp[0],
                        gp[1],
                        cost
                    );
                } else {
                    println!(
                        "Point {}: {:?} (out of bounds) -> Cost: {:?}",
                        i,
                        (p[0], p[1]),
                        cost
                    );
                }
            }
            Err(e) => println!("Point {}: {:?} -> Error: {:?}", i, (p[0], p[1]), e),
        }
    }

    Ok(())
}

fn print_costmap(costmap: &CostMap<2>) {
    let dims = costmap.get_dims();

    // Print from top to bottom (reverse y order for visual clarity)
    for y in (0..dims[1]).rev() {
        print!("{:2} ", y);
        for x in 0..dims[0] {
            let coords = SVector::<usize, 2>::new(x, y);
            match costmap.get_cost(&coords) {
                Ok(CellCost::Free) => print!(". "),
                Ok(CellCost::Lethal) => print!("X "),
                Ok(CellCost::Inflated(_)) => print!("I "),
                Ok(CellCost::Inscribed) => print!("# "),
                Ok(CellCost::Unknown) => print!("? "),
                Err(_) => print!("E "),
            }
        }
        println!();
    }

    // Print x-axis labels
    print!("   ");
    for x in 0..dims[0] {
        print!("{} ", x % 10);
    }
    println!();
}
