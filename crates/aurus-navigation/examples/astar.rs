use aurus_navigation::astar::astar_search;
use std::collections::HashSet;

fn main() {
    // Create a grid
    // 0 = walkable
    // 1 = blocked
    let grid = vec![
        vec![0, 0, 0, 0, 0, 0, 0, 0, 0, 0], // Row 0
        vec![0, 1, 1, 0, 0, 0, 0, 1, 1, 0], // Row 1
        vec![0, 0, 0, 0, 1, 0, 0, 0, 0, 0], // Row 2
        vec![0, 0, 1, 1, 1, 1, 0, 1, 0, 0], // Row 3
        vec![0, 0, 0, 0, 0, 1, 0, 1, 0, 0], // Row 4
        vec![0, 1, 1, 1, 0, 1, 0, 1, 1, 0], // Row 5
        vec![0, 0, 0, 1, 0, 0, 0, 0, 0, 0], // Row 6
        vec![0, 1, 0, 1, 0, 1, 1, 1, 0, 0], // Row 7
        vec![0, 1, 0, 0, 0, 0, 0, 0, 1, 0], // Row 8
        vec![0, 0, 0, 1, 1, 1, 0, 0, 0, 0], // Row 9
    ];

    let start = (0, 0);
    let goal = (9, 9);

    println!("Grid:");
    for r in 0..grid.len() {
        for c in 0..grid[0].len() {
            if (r as i32, c as i32) == start {
                print!("S ");
            } else if (r as i32, c as i32) == goal {
                print!("G ");
            } else if grid[r][c] == 1 {
                print!("X ");
            } else {
                print!(". ");
            }
        }
        println!();
    }
    println!("\nStart: {:?}, Goal: {:?}", start, goal);

    if let Some(path) = astar_search(&grid, start, goal) {
        println!("\nPath found: {:?}", path);

        let path_set: HashSet<(i32, i32)> = path.iter().cloned().collect();

        println!("\nGrid with path:");
        for r in 0..grid.len() {
            for c in 0..grid[0].len() {
                let current_pos = (r as i32, c as i32);
                if current_pos == start {
                    print!("S ");
                } else if current_pos == goal {
                    print!("G ");
                } else if path_set.contains(&current_pos) {
                    print!("* ");
                } else if grid[r][c] == 1 {
                    print!("X ");
                } else {
                    print!(". ");
                }
            }
            println!();
        }
    } else {
        println!("\nNo path found.");
    }
}
