use opencv::{Result, calib3d, core, features2d, imgcodecs, imgproc, prelude::*};
use std::collections::HashMap;
use std::fs;
use std::path::Path;

// Minimal SLAM data structures
#[derive(Debug, Clone)]
pub struct MapPoint {
    pub position: core::Point3f,
    pub descriptor: core::Mat,
    pub observations: u32,
}

#[derive(Debug)]
pub struct KeyFrame {
    pub id: u32,
    pub pose: core::Mat, // 4x4 transformation matrix
    pub keypoints: core::Vector<core::KeyPoint>,
    pub descriptors: core::Mat,
    pub map_points: Vec<Option<u32>>, // indices into global map
}

#[derive(Debug)]
pub struct SimpleMap {
    pub keyframes: HashMap<u32, KeyFrame>,
    pub map_points: HashMap<u32, MapPoint>,
    pub next_kf_id: u32,
    pub next_mp_id: u32,
}

impl SimpleMap {
    pub fn new() -> Self {
        Self {
            keyframes: HashMap::new(),
            map_points: HashMap::new(),
            next_kf_id: 0,
            next_mp_id: 0,
        }
    }

    pub fn add_keyframe(
        &mut self,
        keypoints: core::Vector<core::KeyPoint>,
        descriptors: core::Mat,
        pose: core::Mat,
    ) -> u32 {
        let id = self.next_kf_id;
        self.next_kf_id += 1;

        let map_points_len = keypoints.len();
        let keyframe = KeyFrame {
            id,
            pose,
            keypoints,
            descriptors,
            map_points: vec![None; map_points_len],
        };

        self.keyframes.insert(id, keyframe);
        id
    }

    pub fn add_map_point(&mut self, position: core::Point3f, descriptor: core::Mat) -> u32 {
        let id = self.next_mp_id;
        self.next_mp_id += 1;

        let map_point = MapPoint {
            position,
            descriptor,
            observations: 1,
        };

        self.map_points.insert(id, map_point);
        id
    }
}

pub fn run() -> Result<()> {
    let mut slam_map = SimpleMap::new();

    // Camera intrinsic matrix (TUM freiburg1 camera parameters)
    let camera_matrix =
        core::Mat::from_slice_2d(&[&[517.3, 0.0, 318.6], &[0.0, 516.5, 255.3], &[0.0, 0.0, 1.0]])?;

    // Get all available RGB and depth images
    let rgb_dir = "src/slam/data/tum/rgbd_dataset_freiburg1_xyz/rgb";
    let depth_dir = "src/slam/data/tum/rgbd_dataset_freiburg1_xyz/depth";

    let mut rgb_files = get_image_files(rgb_dir)?;
    let mut depth_files = get_image_files(depth_dir)?;

    // Sort files by timestamp (filename)
    rgb_files.sort();
    depth_files.sort();

    // Take first 20 images for demo (adjust as needed)
    let num_frames = std::cmp::min(100, rgb_files.len());
    rgb_files.truncate(num_frames);
    depth_files.truncate(num_frames);

    println!("=== Starting SLAM with {} frames ===", num_frames);
    println!("RGB images: {} available", rgb_files.len());
    println!("Depth images: {} available", depth_files.len());

    let mut current_pose = core::Mat::eye(4, 4, core::CV_64F)?.to_mat()?; // Identity matrix for first frame
    let mut trajectory = Vec::new(); // Store camera trajectory
    let mut total_map_points = 0;
    let mut total_matches = 0;
    let mut successful_tracks = 0;

    for (i, (rgb_file, depth_file)) in rgb_files.iter().zip(depth_files.iter()).enumerate() {
        println!("\n=== Processing Frame {}/{} ===", i + 1, num_frames);
        println!("RGB: {}", rgb_file.split('/').last().unwrap_or("unknown"));
        println!(
            "Depth: {}",
            depth_file.split('/').last().unwrap_or("unknown")
        );

        // Load RGB and depth images
        let color_image = imgcodecs::imread(rgb_file, imgcodecs::IMREAD_COLOR)?;
        let depth_image = imgcodecs::imread(depth_file, imgcodecs::IMREAD_ANYDEPTH)?;

        if color_image.empty() || depth_image.empty() {
            println!("⚠️  Skipping frame {} - failed to load images", i);
            continue;
        }

        let gray_image = rgbd_to_gray(&color_image)?;

        // Extract features
        let (keypoints, descriptors) = extract_orb_features(&gray_image)?;
        println!("📍 Extracted {} features", keypoints.len());

        if i == 0 {
            // First frame - initialize map
            let kf_id =
                slam_map.add_keyframe(keypoints.clone(), descriptors.clone(), current_pose.clone());

            // Create initial map points from depth
            let map_point_ids = create_map_points_from_depth(
                &mut slam_map,
                &keypoints,
                &descriptors,
                &depth_image,
                &camera_matrix,
                &current_pose,
            )?;

            // Associate map points with keyframe
            if let Some(keyframe) = slam_map.keyframes.get_mut(&kf_id) {
                keyframe.map_points = map_point_ids;
            }

            total_map_points = slam_map.map_points.len();
            println!("🗺️  Created {} initial map points", total_map_points);

            // Add initial pose to trajectory
            trajectory.push(extract_translation(&current_pose)?);
        } else {
            // Track against previous keyframe
            let prev_kf_id = slam_map.keyframes.len() as u32 - 1;
            if let Some(prev_keyframe) = slam_map.keyframes.get(&prev_kf_id) {
                // Match features with previous frame
                let matches = match_features(&prev_keyframe.descriptors, &descriptors)?;
                total_matches += matches.len();
                println!("🔗 Found {} matches", matches.len());

                if matches.len() >= 8 {
                    // Estimate pose using PnP with existing map points
                    match estimate_pose_pnp(
                        &prev_keyframe,
                        &keypoints,
                        &matches,
                        &slam_map,
                        &camera_matrix,
                    ) {
                        Ok(new_pose) => {
                            current_pose = new_pose;
                            successful_tracks += 1;
                            println!("✅ Pose estimated successfully");

                            // Add pose to trajectory
                            trajectory.push(extract_translation(&current_pose)?);

                            // Add new keyframe if motion is significant
                            if should_add_keyframe(&current_pose, &prev_keyframe.pose)? {
                                let kf_id = slam_map.add_keyframe(
                                    keypoints.clone(),
                                    descriptors.clone(),
                                    current_pose.clone(),
                                );

                                // Create new map points
                                let new_map_points = create_map_points_from_depth(
                                    &mut slam_map,
                                    &keypoints,
                                    &descriptors,
                                    &depth_image,
                                    &camera_matrix,
                                    &current_pose,
                                )?;

                                if let Some(keyframe) = slam_map.keyframes.get_mut(&kf_id) {
                                    keyframe.map_points = new_map_points;
                                }

                                let new_points = slam_map.map_points.len() - total_map_points;
                                total_map_points = slam_map.map_points.len();
                                println!(
                                    "🆕 Added keyframe {}, created {} new map points",
                                    kf_id, new_points
                                );
                            }

                            // Print current position
                            let pos = extract_translation(&current_pose)?;
                            println!("📍 Position: [{:.3}, {:.3}, {:.3}]", pos.0, pos.1, pos.2);
                        }
                        Err(_) => {
                            println!("❌ Pose estimation failed - using previous pose");
                            trajectory.push(extract_translation(&current_pose)?);
                        }
                    }
                } else {
                    println!("⚠️  Insufficient matches for pose estimation");
                    trajectory.push(extract_translation(&current_pose)?);
                }
            }
        }

        // Print running statistics
        if i > 0 {
            let track_success_rate = (successful_tracks as f64 / i as f64) * 100.0;
            let avg_matches = if i > 0 { total_matches / i } else { 0 };
            println!(
                "📊 Stats: {:.1}% track success, {} avg matches, {} total map points",
                track_success_rate, avg_matches, total_map_points
            );
        }

        // Save visualization every 5 frames
        if i % 5 == 0 || i == num_frames - 1 {
            let vis_image = create_frame_visualization(&color_image, &keypoints, i, &slam_map)?;
            let filename = format!("slam_frame_{:03}.jpg", i);
            imgcodecs::imwrite(&filename, &vis_image, &core::Vector::new())?;
            println!("💾 Saved visualization: {}", filename);
        }
    }

    // Print final SLAM statistics
    println!("\n=== SLAM COMPLETE ===");
    println!("🎯 Processed {} frames", num_frames);
    println!("🗺️  Total keyframes: {}", slam_map.keyframes.len());
    println!("📍 Total map points: {}", slam_map.map_points.len());
    println!(
        "✅ Successful tracks: {}/{} ({:.1}%)",
        successful_tracks,
        num_frames - 1,
        (successful_tracks as f64 / (num_frames - 1) as f64) * 100.0
    );
    println!(
        "🔗 Average matches per frame: {}",
        if num_frames > 1 {
            total_matches / (num_frames - 1)
        } else {
            0
        }
    );

    // Create trajectory visualization
    let trajectory_image = create_trajectory_visualization(&trajectory)?;
    imgcodecs::imwrite(
        "slam_trajectory.jpg",
        &trajectory_image,
        &core::Vector::new(),
    )?;
    println!("🛤️  Saved trajectory visualization: slam_trajectory.jpg");

    // Create final map visualization
    if let Some(last_kf) = slam_map.keyframes.values().last() {
        let final_vis = create_final_visualization(&slam_map, &last_kf.keypoints)?;
        imgcodecs::imwrite("slam_final_map.jpg", &final_vis, &core::Vector::new())?;
        println!("🗺️  Saved final map visualization: slam_final_map.jpg");
    }

    Ok(())
}

fn get_image_files(dir_path: &str) -> Result<Vec<String>> {
    let mut files = Vec::new();

    if let Ok(entries) = fs::read_dir(dir_path) {
        for entry in entries {
            if let Ok(entry) = entry {
                let path = entry.path();
                if let Some(extension) = path.extension() {
                    if extension == "png" || extension == "jpg" {
                        if let Some(path_str) = path.to_str() {
                            files.push(path_str.to_string());
                        }
                    }
                }
            }
        }
    }

    Ok(files)
}

fn extract_translation(pose: &core::Mat) -> Result<(f64, f64, f64)> {
    let x = *pose.at_2d::<f64>(0, 3)?;
    let y = *pose.at_2d::<f64>(1, 3)?;
    let z = *pose.at_2d::<f64>(2, 3)?;
    Ok((x, y, z))
}

fn create_frame_visualization(
    image: &core::Mat,
    keypoints: &core::Vector<core::KeyPoint>,
    frame_num: usize,
    slam_map: &SimpleMap,
) -> Result<core::Mat> {
    let mut output = image.clone();

    // Draw keypoints
    for i in 0..keypoints.len() {
        let kp = keypoints.get(i)?;
        let center = core::Point::new(kp.pt().x as i32, kp.pt().y as i32);

        imgproc::circle(
            &mut output,
            center,
            3,
            core::Scalar::new(0.0, 255.0, 0.0, 0.0), // Green circles
            2,
            imgproc::LINE_8,
            0,
        )?;
    }

    // Add text overlay with statistics
    let text = format!(
        "Frame: {} | Features: {} | Map Points: {} | Keyframes: {}",
        frame_num,
        keypoints.len(),
        slam_map.map_points.len(),
        slam_map.keyframes.len()
    );

    imgproc::put_text(
        &mut output,
        &text,
        core::Point::new(10, 30),
        imgproc::FONT_HERSHEY_SIMPLEX,
        0.7,
        core::Scalar::new(255.0, 255.0, 255.0, 0.0), // White text
        2,
        imgproc::LINE_8,
        false,
    )?;

    Ok(output)
}

fn create_trajectory_visualization(trajectory: &[(f64, f64, f64)]) -> Result<core::Mat> {
    let mut canvas = core::Mat::zeros(600, 800, core::CV_8UC3)?.to_mat()?;

    if trajectory.len() < 2 {
        return Ok(canvas);
    }

    // Find trajectory bounds
    let mut min_x = trajectory[0].0;
    let mut max_x = trajectory[0].0;
    let mut min_z = trajectory[0].2;
    let mut max_z = trajectory[0].2;

    for &(x, _, z) in trajectory {
        min_x = min_x.min(x);
        max_x = max_x.max(x);
        min_z = min_z.min(z);
        max_z = max_z.max(z);
    }

    // Add padding
    let padding = 0.1;
    let range_x = max_x - min_x;
    let range_z = max_z - min_z;
    min_x -= range_x * padding;
    max_x += range_x * padding;
    min_z -= range_z * padding;
    max_z += range_z * padding;

    // Convert trajectory to image coordinates
    let scale_x = 700.0 / (max_x - min_x);
    let scale_z = 500.0 / (max_z - min_z);
    let scale = scale_x.min(scale_z);

    let mut prev_point = None;

    for (i, &(x, _, z)) in trajectory.iter().enumerate() {
        let img_x = ((x - min_x) * scale + 50.0) as i32;
        let img_y = ((z - min_z) * scale + 50.0) as i32;
        let current_point = core::Point::new(img_x, img_y);

        // Draw trajectory line
        if let Some(prev) = prev_point {
            imgproc::line(
                &mut canvas,
                prev,
                current_point,
                core::Scalar::new(0.0, 255.0, 255.0, 0.0), // Yellow line
                2,
                imgproc::LINE_8,
                0,
            )?;
        }

        // Draw position markers
        let color = if i == 0 {
            core::Scalar::new(0.0, 255.0, 0.0, 0.0) // Green for start
        } else if i == trajectory.len() - 1 {
            core::Scalar::new(0.0, 0.0, 255.0, 0.0) // Red for end
        } else {
            core::Scalar::new(255.0, 255.0, 255.0, 0.0) // White for intermediate
        };

        imgproc::circle(
            &mut canvas,
            current_point,
            4,
            color,
            -1, // Filled circle
            imgproc::LINE_8,
            0,
        )?;

        prev_point = Some(current_point);
    }

    // Add title and legend
    imgproc::put_text(
        &mut canvas,
        "Camera Trajectory (Top View)",
        core::Point::new(10, 30),
        imgproc::FONT_HERSHEY_SIMPLEX,
        0.8,
        core::Scalar::new(255.0, 255.0, 255.0, 0.0),
        2,
        imgproc::LINE_8,
        false,
    )?;

    imgproc::put_text(
        &mut canvas,
        &format!(
            "Frames: {} | Distance: {:.2}m",
            trajectory.len(),
            calculate_trajectory_distance(trajectory)
        ),
        core::Point::new(10, 570),
        imgproc::FONT_HERSHEY_SIMPLEX,
        0.6,
        core::Scalar::new(255.0, 255.0, 255.0, 0.0),
        1,
        imgproc::LINE_8,
        false,
    )?;

    Ok(canvas)
}

fn create_final_visualization(
    slam_map: &SimpleMap,
    keypoints: &core::Vector<core::KeyPoint>,
) -> Result<core::Mat> {
    let mut canvas = core::Mat::zeros(600, 800, core::CV_8UC3)?.to_mat()?;

    // Draw map points (top-down view)
    for map_point in slam_map.map_points.values() {
        let x = ((map_point.position.x + 5.0) * 50.0 + 100.0) as i32; // Scale and offset
        let z = ((map_point.position.z + 5.0) * 50.0 + 100.0) as i32;

        if x >= 0 && x < 800 && z >= 0 && z < 600 {
            imgproc::circle(
                &mut canvas,
                core::Point::new(x, z),
                1,
                core::Scalar::new(255.0, 255.0, 255.0, 0.0), // White points
                -1,
                imgproc::LINE_8,
                0,
            )?;
        }
    }

    // Add title
    imgproc::put_text(
        &mut canvas,
        "Final SLAM Map (Top View)",
        core::Point::new(10, 30),
        imgproc::FONT_HERSHEY_SIMPLEX,
        0.8,
        core::Scalar::new(255.0, 255.0, 255.0, 0.0),
        2,
        imgproc::LINE_8,
        false,
    )?;

    Ok(canvas)
}

fn calculate_trajectory_distance(trajectory: &[(f64, f64, f64)]) -> f64 {
    let mut total_distance = 0.0;

    for i in 1..trajectory.len() {
        let (x1, y1, z1) = trajectory[i - 1];
        let (x2, y2, z2) = trajectory[i];

        let dx = x2 - x1;
        let dy = y2 - y1;
        let dz = z2 - z1;

        total_distance += (dx * dx + dy * dy + dz * dz).sqrt();
    }

    total_distance
}

fn create_map_points_from_depth(
    slam_map: &mut SimpleMap,
    keypoints: &core::Vector<core::KeyPoint>,
    descriptors: &core::Mat,
    depth_image: &core::Mat,
    camera_matrix: &core::Mat,
    _pose: &core::Mat, // Simplified: ignore pose transformation for now
) -> Result<Vec<Option<u32>>> {
    let mut map_point_ids = vec![None; keypoints.len()];

    // Camera parameters
    let fx = camera_matrix.at_2d::<f64>(0, 0)?;
    let fy = camera_matrix.at_2d::<f64>(1, 1)?;
    let cx = camera_matrix.at_2d::<f64>(0, 2)?;
    let cy = camera_matrix.at_2d::<f64>(1, 2)?;

    for i in 0..keypoints.len() {
        let kp = keypoints.get(i)?;
        let u = kp.pt().x as i32;
        let v = kp.pt().y as i32;

        // Check bounds
        if u >= 0 && v >= 0 && u < depth_image.cols() && v < depth_image.rows() {
            let depth_value = *depth_image.at_2d::<u16>(v, u)? as f32 / 5000.0; // TUM depth scale

            if depth_value > 0.1 && depth_value < 10.0 {
                // Valid depth range
                // Back-project to 3D in camera coordinates (simplified - no world transform)
                let x_cam = (u as f64 - cx) * depth_value as f64 / fx;
                let y_cam = (v as f64 - cy) * depth_value as f64 / fy;
                let z_cam = depth_value as f64;

                let world_pos = core::Point3f::new(x_cam as f32, y_cam as f32, z_cam as f32);

                // Extract descriptor for this keypoint - simplified approach
                let mut descriptor = core::Mat::default();
                descriptors.row(i as i32)?.copy_to(&mut descriptor)?;

                let mp_id = slam_map.add_map_point(world_pos, descriptor);
                map_point_ids[i] = Some(mp_id);
            }
        }
    }

    Ok(map_point_ids)
}

fn estimate_pose_pnp(
    prev_keyframe: &KeyFrame,
    current_keypoints: &core::Vector<core::KeyPoint>,
    matches: &core::Vector<core::DMatch>,
    slam_map: &SimpleMap,
    camera_matrix: &core::Mat,
) -> Result<core::Mat> {
    let mut object_points = core::Vector::<core::Point3f>::new();
    let mut image_points = core::Vector::<core::Point2f>::new();

    // Collect 3D-2D correspondences
    for i in 0..matches.len() {
        let m = matches.get(i)?;
        let prev_idx = m.query_idx as usize;
        let curr_idx = m.train_idx as usize;

        // Check if previous keypoint has associated map point
        if prev_idx < prev_keyframe.map_points.len() {
            if let Some(mp_id) = prev_keyframe.map_points[prev_idx] {
                if let Some(map_point) = slam_map.map_points.get(&mp_id) {
                    object_points.push(map_point.position);
                    image_points.push(current_keypoints.get(curr_idx)?.pt());
                }
            }
        }
    }

    if object_points.len() < 4 {
        return Err(opencv::Error::new(
            core::StsError,
            "Not enough 3D-2D correspondences",
        ));
    }

    // Solve PnP
    let mut rvec = core::Mat::default();
    let mut tvec = core::Mat::default();
    let mut inliers = core::Mat::default();

    let success = calib3d::solve_pnp_ransac(
        &object_points,
        &image_points,
        camera_matrix,
        &core::Mat::default(), // no distortion
        &mut rvec,
        &mut tvec,
        false, // use_extrinsic_guess
        100,   // iterations_count
        8.0,   // reprojection_error
        0.99,  // confidence
        &mut inliers,
        calib3d::SOLVEPNP_ITERATIVE,
    )?;

    if !success {
        return Err(opencv::Error::new(core::StsError, "PnP solving failed"));
    }

    // Convert rotation vector to rotation matrix
    let mut rotation_matrix = core::Mat::default();
    calib3d::rodrigues(&rvec, &mut rotation_matrix, &mut core::Mat::default())?;

    // Create 4x4 transformation matrix
    let mut pose = core::Mat::eye(4, 4, core::CV_64F)?.to_mat()?;

    // Copy rotation (3x3)
    for i in 0..3 {
        for j in 0..3 {
            *pose.at_2d_mut::<f64>(i, j)? = *rotation_matrix.at_2d::<f64>(i, j)?;
        }
    }

    // Copy translation (3x1)
    for i in 0..3 {
        *pose.at_2d_mut::<f64>(i, 3)? = *tvec.at::<f64>(i)?;
    }

    println!(
        "PnP solved with {} inliers out of {} correspondences",
        inliers.rows(),
        object_points.len()
    );

    Ok(pose)
}

fn should_add_keyframe(current_pose: &core::Mat, last_pose: &core::Mat) -> Result<bool> {
    // Simple keyframe selection: add if translation > 0.1m or rotation > 10 degrees

    // Extract translation difference
    let tx_diff = current_pose.at_2d::<f64>(0, 3)? - last_pose.at_2d::<f64>(0, 3)?;
    let ty_diff = current_pose.at_2d::<f64>(1, 3)? - last_pose.at_2d::<f64>(1, 3)?;
    let tz_diff = current_pose.at_2d::<f64>(2, 3)? - last_pose.at_2d::<f64>(2, 3)?;

    let translation_distance = (tx_diff * tx_diff + ty_diff * ty_diff + tz_diff * tz_diff).sqrt();

    // Simple threshold-based decision
    Ok(translation_distance > 0.1) // 10cm threshold
}

fn visualize_features(
    image: &core::Mat,
    keypoints: &core::Vector<core::KeyPoint>,
) -> Result<core::Mat> {
    let mut output = image.clone();

    for i in 0..keypoints.len() {
        let kp = keypoints.get(i)?;
        let center = core::Point::new(kp.pt().x as i32, kp.pt().y as i32);

        imgproc::circle(
            &mut output,
            center,
            3,
            core::Scalar::new(0.0, 255.0, 0.0, 0.0), // Green circles
            2,
            imgproc::LINE_8,
            0,
        )?;
    }

    Ok(output)
}

fn rgbd_to_gray(color_image: &core::Mat) -> Result<core::Mat> {
    let mut gray_image = core::Mat::default();
    imgproc::cvt_color(
        &color_image,
        &mut gray_image,
        imgproc::COLOR_BGR2GRAY,
        0, // dcn = 0 lets OpenCV figure it out
        core::AlgorithmHint::ALGO_HINT_DEFAULT,
    )?;
    Ok(gray_image)
}

fn extract_orb_features(image: &core::Mat) -> Result<(core::Vector<core::KeyPoint>, core::Mat)> {
    // Create ORB detector with default parameters
    let mut orb = features2d::ORB::create_def()?;

    let mut keypoints = core::Vector::<core::KeyPoint>::new();
    let mut descriptors = core::Mat::default();

    // ORB does both detection and description in one call
    orb.detect_and_compute(
        image,
        &core::Mat::default(), // no mask
        &mut keypoints,
        &mut descriptors,
        false, // use_provided_keypoints = false
    )?;

    Ok((keypoints, descriptors))
}

fn match_features(desc1: &core::Mat, desc2: &core::Mat) -> Result<core::Vector<core::DMatch>> {
    // Create brute force matcher with Hamming distance (for binary descriptors like ORB)
    let bf_matcher = features2d::BFMatcher::new(core::NORM_HAMMING, false)?;

    let mut matches = core::Vector::<core::DMatch>::new();

    // Simple nearest neighbor matching
    bf_matcher.train_match(desc1, desc2, &mut matches, &core::Mat::default())?;

    // Filter matches by distance (simple threshold)
    let mut good_matches = core::Vector::<core::DMatch>::new();
    let distance_threshold = 50.0; // Adjust this value as needed

    for i in 0..matches.len() {
        let m = matches.get(i)?;
        if m.distance < distance_threshold {
            good_matches.push(m);
        }
    }

    Ok(good_matches)
}
