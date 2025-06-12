use aurus_navigation::astar::astar_search_grid;
use aurus_navigation::map::costmap::{CellCost, CostMap};
use aurus_common::GridPoint;
use nalgebra::SVector;
use std::collections::HashSet;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create a 10x10 costmap
    let dims = SVector::<usize, 2>::new(10, 10);
    let resolution = SVector::<f32, 2>::new(0.1, 0.1);
    let origin = SVector::<f32, 2>::new(0.0, 0.0);
    let mut costmap = CostMap::<2>::new(dims, resolution, origin)?;

    // Create the same obstacle pattern as the original grid
    let obstacles = vec![
        (1, 1), (2, 1), (7, 1), (8, 1),
        (4, 2),
        (2, 3), (3, 3), (4, 3), (5, 3), (7, 3),
        (5, 4), (7, 4),
        (1, 5), (2, 5), (3, 5), (5, 5), (7, 5), (8, 5),
        (3, 6),
        (1, 7), (3, 7), (5, 7), (6, 7), (7, 7),
        (1, 8), (8, 8),
        (3, 9), (4, 9), (5, 9),
    ];

    // Set obstacles in the costmap
    for (x, y) in obstacles {
        let coords = SVector::<usize, 2>::new(x, y);
        costmap.set_cost(&coords, CellCost::Lethal)?;
    }

    let start = GridPoint::<2>::from([0, 0]);
    let goal = GridPoint::<2>::from([9, 9]);

    println!("Grid:");
    print_costmap(&costmap, Some(&start), Some(&goal), None);
    println!("\nStart: {:?}", start);
    println!("Goal: {:?}", goal);

    if let Some(path) = astar_search_grid(&costmap, start, goal) {
        println!("\nPath found with {} waypoints!", path.len());

        let path_set: HashSet<(usize, usize)> = path
            .iter()
            .map(|p| (p.coords[0], p.coords[1]))
            .collect();

        println!("\nGrid with path:");
        print_costmap(&costmap, Some(&start), Some(&goal), Some(&path_set));
    } else {
        println!("\nNo path found.");
    }

    Ok(())
}

fn print_costmap(
    costmap: &CostMap<2>,
    start: Option<&GridPoint<2>>,
    goal: Option<&GridPoint<2>>,
    path: Option<&HashSet<(usize, usize)>>,
) {
    let dims = costmap.get_dims();
    
    // Print from top to bottom (reverse y order for visual clarity)
    for y in (0..dims[1]).rev() {
        print!("{} ", y);
        for x in 0..dims[0] {
            let coords = SVector::<usize, 2>::new(x, y);
            let pos = (x, y);
            
            // Check if this is start or goal position
            if let Some(s) = start {
                if s.coords[0] == x && s.coords[1] == y {
                    print!("S ");
                    continue;
                }
            }
            if let Some(g) = goal {
                if g.coords[0] == x && g.coords[1] == y {
                    print!("G ");
                    continue;
                }
            }
            
            // Check if this is part of the path
            if let Some(path_set) = path {
                if path_set.contains(&pos) {
                    print!("* ");
                    continue;
                }
            }
            
            // Print based on cost
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
    print!("  ");
    for x in 0..dims[0] {
        print!("{} ", x);
    }
    println!();
}
