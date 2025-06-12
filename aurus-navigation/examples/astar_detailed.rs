use aurus_common::GridPoint;
#[cfg(feature = "serde")]
use aurus_navigation::astar::PathResult;
use aurus_navigation::astar::astar_search_grid_detailed;
use aurus_navigation::map::costmap::{CellCost, CostMap};
use nalgebra::SVector;

#[cfg(feature = "serde")]
use serde_json;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("A* Pathfinding with Detailed Results Example");
    println!("============================================");

    // Create a 2D costmap
    let dims = SVector::<usize, 2>::new(8, 8);
    let resolution = SVector::<f32, 2>::new(0.1, 0.1);
    let origin = SVector::<f32, 2>::new(0.0, 0.0);
    let mut costmap = CostMap::<2>::new(dims, resolution, origin)?;

    // Add some obstacles to make pathfinding interesting
    println!("\nSetting up obstacles...");

    // Create a wall
    for y in 2..6 {
        let coords = SVector::<usize, 2>::new(3, y);
        costmap.set_cost(&coords, CellCost::Lethal)?;
    }

    // Add some inflated cost areas
    for x in 1..3 {
        for y in 1..3 {
            let coords = SVector::<usize, 2>::new(x, y);
            costmap.set_cost(&coords, CellCost::Inflated(50))?;
        }
    }

    // Display the costmap
    println!("\nCostmap layout:");
    print_costmap(&costmap);

    // Define start and goal
    let start = GridPoint::<2>::from([0, 0]);
    let goal = GridPoint::<2>::from([7, 7]);

    println!("\nStart: {:?}", start);
    println!("Goal: {:?}", goal);

    // Perform pathfinding with detailed results
    println!("\nPerforming A* pathfinding...");
    let result = astar_search_grid_detailed(&costmap, start, goal);

    // Display the result
    println!("\nPathfinding Result:");
    println!("{}", result);

    if result.is_success() {
        if let Some(path) = &result.path {
            println!("\nPath waypoints:");
            for (i, point) in path.iter().enumerate() {
                println!("  {}: {:?}", i, point);
            }

            println!("\nPath visualization:");
            print_path_on_costmap(&costmap, path);
        }

        // Demonstrate serialization if serde feature is enabled
        #[cfg(feature = "serde")]
        {
            println!("\nSerialization example:");
            match serde_json::to_string_pretty(&result) {
                Ok(json) => {
                    println!("PathResult as JSON:");
                    println!("{}", json);

                    // Test deserialization
                    match serde_json::from_str::<PathResult<GridPoint<2>, 2>>(&json) {
                        Ok(deserialized) => {
                            println!("\nDeserialization successful!");
                            println!("Deserialized result: {}", deserialized);
                        }
                        Err(e) => println!("Deserialization failed: {}", e),
                    }
                }
                Err(e) => println!("Serialization failed: {}", e),
            }
        }

        #[cfg(not(feature = "serde"))]
        {
            println!("\nNote: Enable 'serde' feature to see serialization example");
            println!("Run with: cargo run --example astar_detailed --features serde");
        }
    } else {
        println!("\nNo path found!");
    }

    // Test a blocked scenario
    println!("\n{}", "=".repeat(50));
    println!("Testing blocked scenario...");

    let mut blocked_costmap = CostMap::<2>::new(dims, resolution, origin)?;

    // Block entire middle column
    for y in 0..8 {
        let coords = SVector::<usize, 2>::new(3, y);
        blocked_costmap.set_cost(&coords, CellCost::Lethal)?;
    }

    let blocked_result = astar_search_grid_detailed(&blocked_costmap, start, goal);
    println!("Blocked scenario result: {}", blocked_result);

    Ok(())
}

fn print_costmap(costmap: &CostMap<2>) {
    println!("Legend: . = Free, X = Lethal, I = Inflated");
    let dims = costmap.get_dims();
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
    print!("   ");
    for x in 0..dims[0] {
        print!("{} ", x);
    }
    println!();
}

fn print_path_on_costmap(costmap: &CostMap<2>, path: &[GridPoint<2>]) {
    use std::collections::HashSet;

    let path_set: HashSet<(usize, usize)> =
        path.iter().map(|p| (p.coords[0], p.coords[1])).collect();

    println!("Legend: . = Free, X = Lethal, I = Inflated, * = Path, S = Start, G = Goal");
    let dims = costmap.get_dims();
    for y in (0..dims[1]).rev() {
        print!("{:2} ", y);
        for x in 0..dims[0] {
            let coords = SVector::<usize, 2>::new(x, y);
            let pos = (x, y);

            if pos == (path[0].coords[0], path[0].coords[1]) {
                print!("S ");
            } else if pos
                == (
                    path.last().unwrap().coords[0],
                    path.last().unwrap().coords[1],
                )
            {
                print!("G ");
            } else if path_set.contains(&pos) {
                print!("* ");
            } else {
                match costmap.get_cost(&coords) {
                    Ok(CellCost::Free) => print!(". "),
                    Ok(CellCost::Lethal) => print!("X "),
                    Ok(CellCost::Inflated(_)) => print!("I "),
                    Ok(CellCost::Inscribed) => print!("# "),
                    Ok(CellCost::Unknown) => print!("? "),
                    Err(_) => print!("E "),
                }
            }
        }
        println!();
    }
    print!("   ");
    for x in 0..dims[0] {
        print!("{} ", x);
    }
    println!();
}
