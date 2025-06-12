//! Core geometric types and operations for SLAM systems.
//!
//! This module provides fundamental geometric primitives using a body/world coordinate
//! system convention with generalizable dimensions. It wraps nalgebra types to provide
//! domain-specific operations while maintaining type safety between different coordinate frames.
//!
//! # Coordinate Systems
//!
//! - **World Frame**: Global coordinate system, typically fixed to the environment
//! - **Body Frame**: Local coordinate system attached to the sensor/robot
//! - **Sensor Frame**: 2D coordinate system of sensor observations (e.g., pixels)

use nalgebra::{Isometry3, Matrix3, Matrix4, Point, Point2, Point3, UnitQuaternion, Vector3};
use std::fmt;

// Re-export nalgebra types for direct use where appropriate
pub use nalgebra::{Matrix2, Matrix6, Vector2, Vector4};

// Generalizable dimension type aliases
/// A point in world/global coordinates with D dimensions.
///
/// Represents a point in the global coordinate system, typically fixed to the
/// environment. This is the reference frame for mapping and localization.
pub type WorldPoint<const D: usize> = Point<f32, D>;

/// A point in grid coordinates with D dimensions using integer indices.
///
/// Represents discrete grid coordinates, useful for occupancy grids,
/// voxel grids, and other discretized spatial representations.
pub type GridPoint<const D: usize> = Point<usize, D>;

// Specialized 2D and 3D type aliases for common use cases
/// A 2D point in sensor/observation coordinates.
///
/// Represents a point in the sensor's 2D coordinate system, such as pixel coordinates
/// in an image or angular coordinates from a scanning sensor.
pub type Point2D = Point2<f64>;

/// A 3D point in world/global coordinates (specialized for f64).
///
/// Represents a point in the global coordinate system with double precision.
pub type WorldPoint3D = Point3<f64>;

/// A 3D point in body/local coordinates.
///
/// Represents a point in the sensor's local coordinate system. This frame is
/// typically attached to the robot or sensor and moves with it.
pub type BodyPoint = Point3<f64>;

// Generic dimension-aware coordinate conversion utilities
/// Converts between different coordinate representations while preserving dimensionality.
pub trait CoordinateConversion<T, const D: usize> {
    /// Converts to nalgebra SVector for use with navigation algorithms.
    fn to_svector(&self) -> nalgebra::SVector<T, D>;
    /// Creates from nalgebra SVector.
    fn from_svector(svector: &nalgebra::SVector<T, D>) -> Self;
}

/// Pose of body frame relative to world frame (SE3 transformation).
///
/// Represents the position and orientation of the body frame (sensor/robot)
/// in the world coordinate system. Uses SE3 Lie group representation for
/// efficient composition and optimization.
#[derive(Debug, Clone, PartialEq)]
pub struct Pose {
    /// SE3 transformation (rotation + translation) from world to body frame
    pub isometry: Isometry3<f64>,
}

/// Sensor projection model for transforming between 3D and 2D coordinates.
///
/// Encapsulates the intrinsic parameters of a sensor (e.g., camera) including
/// focal lengths, principal point, and depth sensing parameters. Supports
/// projection from 3D body coordinates to 2D sensor coordinates and vice versa.
#[derive(Debug, Clone, PartialEq)]
pub struct ProjectionModel {
    /// Focal length in x direction (sensor units)
    pub fx: f64,
    /// Focal length in y direction (sensor units)
    pub fy: f64,
    /// Principal point x coordinate (sensor units)
    pub cx: f64,
    /// Principal point y coordinate (sensor units)
    pub cy: f64,
    /// Sensor width (sensor units)
    pub width: u32,
    /// Sensor height (sensor units)
    pub height: u32,
    /// Depth scale factor: depth_meters = depth_raw * depth_scale
    pub depth_scale: f32,
    /// Maximum valid depth (meters)
    pub depth_max: f32,
    /// Minimum valid depth (meters)
    pub depth_min: f32,
}

/// Extension trait for 2D points in sensor coordinates.
pub trait Point2DExt {
    /// Checks if the point is within the given sensor bounds.
    ///
    /// # Arguments
    ///
    /// * `width` - Sensor width in sensor units
    /// * `height` - Sensor height in sensor units
    ///
    /// # Returns
    ///
    /// `true` if the point is within bounds, `false` otherwise.
    ///
    /// # Examples
    ///
    /// ```
    /// use nalgebra::Point2;
    /// use aurus_common::{Point2D, Point2DExt};
    ///
    /// let point: Point2D = Point2::new(100.0, 200.0);
    /// assert!(point.is_in_bounds(640, 480));
    /// assert!(!point.is_in_bounds(50, 50));
    /// ```
    fn is_in_bounds(&self, width: u32, height: u32) -> bool;
}

impl Point2DExt for Point2D {
    fn is_in_bounds(&self, width: u32, height: u32) -> bool {
        self.x >= 0.0 && self.x < width as f64 && self.y >= 0.0 && self.y < height as f64
    }
}

/// Extension trait for 3D points in world coordinates.
pub trait WorldPointExt {
    /// Transforms this world point to body coordinates using the given pose.
    ///
    /// # Arguments
    ///
    /// * `pose` - The pose transformation from world to body frame
    ///
    /// # Returns
    ///
    /// A BodyPoint representing the same 3D location in body coordinates.
    ///
    /// # Examples
    ///
    /// ```
    /// use nalgebra::{Point3, Vector3};
    /// use aurus_common::{WorldPoint3D, WorldPointExt, Pose};
    ///
    /// let world_point: WorldPoint3D = Point3::new(1.0, 2.0, 3.0);
    /// let pose = Pose::from_translation(Vector3::new(0.5, 0.5, 0.5));
    /// let body_point = world_point.to_body(&pose);
    /// ```
    fn to_body(&self, pose: &Pose) -> BodyPoint;
}

impl WorldPointExt for WorldPoint3D {
    fn to_body(&self, pose: &Pose) -> BodyPoint {
        pose.isometry.inverse() * self
    }
}

/// Generic extension trait for dimension-aware world points.
pub trait WorldPointExtGeneric<const D: usize> {
    /// Converts to SVector for use with navigation algorithms.
    fn to_navigation_coords(&self) -> nalgebra::SVector<f32, D>;
    /// Creates from SVector navigation coordinates.
    fn from_navigation_coords(coords: &nalgebra::SVector<f32, D>) -> Self;
}

/// Extension trait for 3D points in body coordinates.
pub trait BodyPointExt {
    /// Projects this 3D body point to 2D sensor coordinates.
    ///
    /// Uses the pinhole camera model to project the 3D point onto the sensor plane.
    /// Returns `None` if the point is behind the sensor or outside sensor bounds.
    ///
    /// # Arguments
    ///
    /// * `projection` - The projection model containing sensor parameters
    ///
    /// # Returns
    ///
    /// `Some(Point2D)` if projection is successful, `None` otherwise.
    ///
    /// # Examples
    ///
    /// ```
    /// use nalgebra::Point3;
    /// use aurus_common::{BodyPoint, BodyPointExt, ProjectionModel, Point2DExt};
    ///
    /// let body_point: BodyPoint = Point3::new(0.1, 0.2, 1.0);
    /// let projection = ProjectionModel::new(500.0, 500.0, 320.0, 240.0, 640, 480);
    /// let sensor_point = body_point.project(&projection);
    /// ```
    fn project(&self, projection: &ProjectionModel) -> Option<Point2D>;

    /// Transforms this body point to world coordinates using the given pose.
    ///
    /// # Arguments
    ///
    /// * `pose` - The pose transformation from world to body frame
    ///
    /// # Returns
    ///
    /// A WorldPoint3D representing the same 3D location in world coordinates.
    ///
    /// # Examples
    ///
    /// ```
    /// use nalgebra::{Point3, Vector3};
    /// use aurus_common::{BodyPoint, BodyPointExt, Pose};
    ///
    /// let body_point: BodyPoint = Point3::new(0.1, 0.2, 1.0);
    /// let pose = Pose::from_translation(Vector3::new(0.5, 0.5, 0.5));
    /// let world_point = body_point.to_world(&pose);
    /// ```
    fn to_world(&self, pose: &Pose) -> WorldPoint3D;
}

impl BodyPointExt for BodyPoint {
    fn project(&self, projection: &ProjectionModel) -> Option<Point2D> {
        if self.z <= 0.0 {
            return None; // Point behind sensor
        }

        let x = projection.fx * self.x / self.z + projection.cx;
        let y = projection.fy * self.y / self.z + projection.cy;

        let sensor_point = Point2::new(x, y);
        if sensor_point.is_in_bounds(projection.width, projection.height) {
            Some(sensor_point)
        } else {
            None
        }
    }

    fn to_world(&self, pose: &Pose) -> WorldPoint3D {
        pose.isometry * self
    }
}

impl Pose {
    /// Creates an identity pose (no rotation, no translation).
    ///
    /// The identity pose represents no transformation - the body frame
    /// coincides with the world frame.
    ///
    /// # Returns
    ///
    /// A pose representing the identity transformation.
    ///
    /// # Examples
    ///
    /// ```
    /// use aurus_common::Pose;
    /// use nalgebra::Vector3;
    ///
    /// let identity = Pose::identity();
    /// let translation = identity.translation();
    /// assert_eq!(translation, Vector3::new(0.0, 0.0, 0.0));
    /// ```
    pub fn identity() -> Self {
        Self {
            isometry: Isometry3::identity(),
        }
    }

    /// Creates a pose from a rotation matrix and translation vector.
    ///
    /// # Arguments
    ///
    /// * `rotation` - 3x3 rotation matrix
    /// * `translation` - 3D translation vector
    ///
    /// # Returns
    ///
    /// A new pose representing the given transformation.
    ///
    /// # Examples
    ///
    /// ```
    /// use aurus_common::Pose;
    /// use nalgebra::{Matrix3, Vector3};
    ///
    /// let rotation = Matrix3::identity();
    /// let translation = Vector3::new(1.0, 2.0, 3.0);
    /// let pose = Pose::from_matrix_translation(rotation, translation);
    /// ```
    pub fn from_matrix_translation(rotation: Matrix3<f64>, translation: Vector3<f64>) -> Self {
        Self {
            isometry: Isometry3::from_parts(
                translation.into(),
                UnitQuaternion::from_matrix(&rotation),
            ),
        }
    }

    /// Creates a pose from translation only (no rotation).
    ///
    /// # Arguments
    ///
    /// * `translation` - 3D translation vector
    ///
    /// # Returns
    ///
    /// A new pose with the given translation and identity rotation.
    ///
    /// # Examples
    ///
    /// ```
    /// use aurus_common::Pose;
    /// use nalgebra::Vector3;
    ///
    /// let translation = Vector3::new(1.0, 2.0, 3.0);
    /// let pose = Pose::from_translation(translation);
    /// assert_eq!(pose.translation(), translation);
    /// ```
    pub fn from_translation(translation: Vector3<f64>) -> Self {
        Self {
            isometry: Isometry3::translation(translation.x, translation.y, translation.z),
        }
    }

    /// Returns the rotation component as a 3x3 matrix.
    ///
    /// # Returns
    ///
    /// The rotation matrix representing the orientation of the body frame
    /// relative to the world frame.
    pub fn rotation(&self) -> Matrix3<f64> {
        *self.isometry.rotation.to_rotation_matrix().matrix()
    }

    /// Returns the translation component as a 3D vector.
    ///
    /// # Returns
    ///
    /// The translation vector representing the position of the body frame
    /// origin in world coordinates.
    pub fn translation(&self) -> Vector3<f64> {
        self.isometry.translation.vector
    }

    /// Composes this pose with another pose.
    ///
    /// Performs the composition `self * other`, which applies `other` first,
    /// then `self`. This is equivalent to transforming by `other`, then by `self`.
    ///
    /// # Arguments
    ///
    /// * `other` - The pose to compose with
    ///
    /// # Returns
    ///
    /// The composed pose.
    ///
    /// # Examples
    ///
    /// ```
    /// use aurus_common::Pose;
    /// use nalgebra::Vector3;
    ///
    /// let pose1 = Pose::from_translation(Vector3::new(1.0, 0.0, 0.0));
    /// let pose2 = Pose::from_translation(Vector3::new(0.0, 1.0, 0.0));
    /// let composed = pose1.compose(&pose2);
    /// ```
    pub fn compose(&self, other: &Pose) -> Pose {
        Self {
            isometry: self.isometry * other.isometry,
        }
    }

    /// Returns the inverse of this pose.
    ///
    /// The inverse pose transforms from body frame back to world frame.
    ///
    /// # Returns
    ///
    /// The inverse pose.
    ///
    /// # Examples
    ///
    /// ```
    /// use aurus_common::Pose;
    /// use nalgebra::Vector3;
    ///
    /// let pose = Pose::from_translation(Vector3::new(1.0, 2.0, 3.0));
    /// let inverse = pose.inverse();
    /// let identity = pose.compose(&inverse);
    /// ```
    pub fn inverse(&self) -> Pose {
        Self {
            isometry: self.isometry.inverse(),
        }
    }

    /// Converts the pose to a 4x4 homogeneous transformation matrix.
    ///
    /// # Returns
    ///
    /// A 4x4 matrix representing the SE3 transformation.
    ///
    /// # Examples
    ///
    /// ```
    /// use aurus_common::Pose;
    /// use nalgebra::Vector3;
    ///
    /// let pose = Pose::from_translation(Vector3::new(1.0, 2.0, 3.0));
    /// let matrix = pose.to_matrix();
    /// assert_eq!(matrix[(0, 3)], 1.0);
    /// assert_eq!(matrix[(1, 3)], 2.0);
    /// assert_eq!(matrix[(2, 3)], 3.0);
    /// ```
    pub fn to_matrix(&self) -> Matrix4<f64> {
        self.isometry.to_homogeneous()
    }

    /// Creates a pose from a 4x4 homogeneous transformation matrix.
    ///
    /// # Arguments
    ///
    /// * `matrix` - A 4x4 transformation matrix
    ///
    /// # Returns
    ///
    /// `Some(Pose)` if the matrix represents a valid SE3 transformation,
    /// `None` otherwise.
    ///
    /// # Examples
    ///
    /// ```
    /// use aurus_common::Pose;
    /// use nalgebra::Matrix4;
    ///
    /// let matrix = Matrix4::identity();
    /// let pose = Pose::from_matrix(&matrix);
    /// assert!(pose.is_some());
    /// ```
    pub fn from_matrix(matrix: &Matrix4<f64>) -> Option<Self> {
        // Check if the last row is [0, 0, 0, 1] for a valid homogeneous matrix
        if (matrix[(3, 0)] - 0.0).abs() > 1e-10
            || (matrix[(3, 1)] - 0.0).abs() > 1e-10
            || (matrix[(3, 2)] - 0.0).abs() > 1e-10
            || (matrix[(3, 3)] - 1.0).abs() > 1e-10
        {
            return None;
        }

        // Extract the 3x3 rotation part
        let rotation_matrix = Matrix3::new(
            matrix[(0, 0)],
            matrix[(0, 1)],
            matrix[(0, 2)],
            matrix[(1, 0)],
            matrix[(1, 1)],
            matrix[(1, 2)],
            matrix[(2, 0)],
            matrix[(2, 1)],
            matrix[(2, 2)],
        );

        // Extract the translation vector
        let translation = Vector3::new(matrix[(0, 3)], matrix[(1, 3)], matrix[(2, 3)]);

        // Convert rotation matrix to unit quaternion
        // Check if rotation matrix is valid (orthogonal with determinant 1)
        let det = rotation_matrix.determinant();
        if (det - 1.0).abs() > 1e-6 {
            return None;
        }

        let unit_quaternion = UnitQuaternion::from_matrix(&rotation_matrix);
        let isometry = Isometry3::from_parts(translation.into(), unit_quaternion);

        Some(Self { isometry })
    }

    /// Computes the relative pose from this pose to another pose.
    ///
    /// Returns the transformation that, when applied to this pose,
    /// results in the other pose.
    ///
    /// # Arguments
    ///
    /// * `other` - The target pose
    ///
    /// # Returns
    ///
    /// The relative pose from `self` to `other`.
    ///
    /// # Examples
    ///
    /// ```
    /// use aurus_common::Pose;
    /// use nalgebra::Vector3;
    ///
    /// let pose1 = Pose::from_translation(Vector3::new(1.0, 0.0, 0.0));
    /// let pose2 = Pose::from_translation(Vector3::new(2.0, 0.0, 0.0));
    /// let relative = pose1.relative_to(&pose2);
    /// ```
    pub fn relative_to(&self, other: &Pose) -> Pose {
        Self {
            isometry: self.isometry.inverse() * other.isometry,
        }
    }

    /// Converts the pose to its Lie algebra representation (se3).
    ///
    /// Returns the 6-dimensional tangent space vector [ω, v] where:
    /// - ω (first 3 elements): rotation axis-angle representation
    /// - v (last 3 elements): translation component
    ///
    /// This is a simplified logarithmic map from SE3 to se3, useful for optimization.
    /// Note: This is a simplified version that concatenates rotation and translation.
    ///
    /// # Returns
    ///
    /// A 6D vector representing the pose in the Lie algebra.
    ///
    /// # Examples
    ///
    /// ```
    /// use aurus_common::Pose;
    /// use nalgebra::Vector3;
    ///
    /// let pose = Pose::from_translation(Vector3::new(1.0, 2.0, 3.0));
    /// let tangent = pose.log();
    /// assert_eq!(tangent.len(), 6);
    /// ```
    pub fn log(&self) -> nalgebra::Vector6<f64> {
        // Extract rotation and translation
        let rotation = self.isometry.rotation;
        let translation = self.isometry.translation.vector;

        // Compute rotation axis-angle (logarithmic map for SO3)
        let axis_angle = rotation.scaled_axis();

        // Simplified approach: just concatenate rotation and translation
        // This avoids the complex V matrix computation but still provides
        // a useful representation for interpolation and optimization
        nalgebra::Vector6::new(
            axis_angle.x,
            axis_angle.y,
            axis_angle.z,
            translation.x,
            translation.y,
            translation.z,
        )
    }

    /// Creates a pose from its Lie algebra representation (se3).
    ///
    /// Takes a 6-dimensional tangent space vector [ω, v] and converts it
    /// to a pose using the exponential map from se3 to SE3.
    ///
    /// # Arguments
    ///
    /// * `tangent` - 6D vector [ω, v] in the Lie algebra
    ///
    /// # Returns
    ///
    /// A pose representing the exponential map of the tangent vector.
    ///
    /// # Examples
    ///
    /// ```
    /// use aurus_common::Pose;
    /// use nalgebra::Vector6;
    ///
    /// let tangent = Vector6::new(0.1, 0.2, 0.3, 1.0, 2.0, 3.0);
    /// let pose = Pose::exp(&tangent);
    ///
    /// // Verify round-trip consistency
    /// let recovered = pose.log();
    /// assert!((tangent - recovered).norm() < 1e-10);
    /// ```
    pub fn exp(tangent: &nalgebra::Vector6<f64>) -> Self {
        // Extract rotation and translation parts
        let omega = Vector3::new(tangent[0], tangent[1], tangent[2]);
        let translation = Vector3::new(tangent[3], tangent[4], tangent[5]);

        // Compute rotation using exponential map for SO3
        let angle = omega.norm();
        let rotation = if angle < 1e-6 {
            // Small angle approximation
            UnitQuaternion::from_scaled_axis(omega)
        } else {
            UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(omega), angle)
        };

        // Simplified approach: use translation directly
        // This matches our simplified log implementation
        Self {
            isometry: Isometry3::from_parts(translation.into(), rotation),
        }
    }

    /// Interpolates between this pose and another pose using SLERP.
    ///
    /// Performs spherical linear interpolation for the rotation component
    /// and linear interpolation for the translation component.
    ///
    /// # Arguments
    ///
    /// * `other` - The target pose to interpolate towards
    /// * `t` - Interpolation parameter in [0, 1], where 0 returns `self` and 1 returns `other`
    ///
    /// # Returns
    ///
    /// The interpolated pose.
    ///
    /// # Examples
    ///
    /// ```
    /// use aurus_common::Pose;
    /// use nalgebra::Vector3;
    ///
    /// let pose1 = Pose::from_translation(Vector3::new(0.0, 0.0, 0.0));
    /// let pose2 = Pose::from_translation(Vector3::new(2.0, 0.0, 0.0));
    /// let midpoint = pose1.interpolate(&pose2, 0.5);
    ///
    /// let expected_translation = Vector3::new(1.0, 0.0, 0.0);
    /// assert!((midpoint.translation() - expected_translation).norm() < 1e-10);
    /// ```
    pub fn interpolate(&self, other: &Pose, t: f64) -> Pose {
        // Clamp t to [0, 1]
        let t = t.clamp(0.0, 1.0);

        // SLERP for rotation
        let rotation = self.isometry.rotation.slerp(&other.isometry.rotation, t);

        // Linear interpolation for translation
        let translation =
            self.isometry.translation.vector * (1.0 - t) + other.isometry.translation.vector * t;

        Self {
            isometry: Isometry3::from_parts(translation.into(), rotation),
        }
    }

    /// Interpolates between poses using the Lie group structure.
    ///
    /// This method uses the exponential/logarithmic maps to perform
    /// interpolation in the Lie algebra (se3), which can provide
    /// better behavior for large rotations compared to naive SLERP.
    ///
    /// # Arguments
    ///
    /// * `other` - The target pose to interpolate towards
    /// * `t` - Interpolation parameter in [0, 1]
    ///
    /// # Returns
    ///
    /// The interpolated pose using Lie group interpolation.
    ///
    /// # Examples
    ///
    /// ```
    /// use aurus_common::Pose;
    /// use nalgebra::Vector3;
    ///
    /// let pose1 = Pose::identity();
    /// let pose2 = Pose::from_translation(Vector3::new(1.0, 1.0, 1.0));
    /// let interpolated = pose1.interpolate_lie(&pose2, 0.3);
    /// ```
    pub fn interpolate_lie(&self, other: &Pose, t: f64) -> Pose {
        // Clamp t to [0, 1]
        let t = t.clamp(0.0, 1.0);

        // Compute relative transformation
        let relative = self.relative_to(other);

        // Get Lie algebra representation
        let tangent = relative.log();

        // Scale by interpolation parameter
        let scaled_tangent = tangent * t;

        // Exponential map back to SE3 and compose with starting pose
        let interpolated_relative = Self::exp(&scaled_tangent);
        self.compose(&interpolated_relative)
    }
}

impl Default for Pose {
    fn default() -> Self {
        Self::identity()
    }
}

impl ProjectionModel {
    /// Creates a new projection model with the given parameters.
    ///
    /// # Arguments
    ///
    /// * `fx` - Focal length in x direction (sensor units)
    /// * `fy` - Focal length in y direction (sensor units)
    /// * `cx` - Principal point x coordinate (sensor units)
    /// * `cy` - Principal point y coordinate (sensor units)
    /// * `width` - Sensor width (sensor units)
    /// * `height` - Sensor height (sensor units)
    ///
    /// # Returns
    ///
    /// A new ProjectionModel with default depth parameters.
    ///
    /// # Examples
    ///
    /// ```
    /// use aurus_common::ProjectionModel;
    ///
    /// let projection = ProjectionModel::new(525.0, 525.0, 320.0, 240.0, 640, 480);
    /// assert_eq!(projection.fx, 525.0);
    /// assert_eq!(projection.width, 640);
    /// ```
    pub fn new(fx: f64, fy: f64, cx: f64, cy: f64, width: u32, height: u32) -> Self {
        Self {
            fx,
            fy,
            cx,
            cy,
            width,
            height,
            depth_scale: 1.0 / 1000.0, // Default: 1mm per unit
            depth_max: 10.0,           // Default: 10 meters
            depth_min: 0.1,            // Default: 10 cm
        }
    }

    /// Unprojects a 2D sensor point with depth to 3D body coordinates.
    ///
    /// Uses the inverse pinhole camera model to transform from 2D sensor
    /// coordinates plus depth to 3D coordinates in the body frame.
    ///
    /// # Arguments
    ///
    /// * `sensor_point` - 2D point in sensor coordinates
    /// * `depth` - Depth value (in raw sensor units, will be scaled)
    ///
    /// # Returns
    ///
    /// `Some(BodyPoint)` if unprojection is successful, `None` if depth is invalid.
    ///
    /// # Examples
    ///
    /// ```
    /// use nalgebra::Point2;
    /// use aurus_common::{ProjectionModel, Point2D};
    ///
    /// let projection = ProjectionModel::new(525.0, 525.0, 320.0, 240.0, 640, 480);
    /// let sensor_point: Point2D = Point2::new(320.0, 240.0);
    /// let depth = 1000.0; // 1 meter in mm
    /// let body_point = projection.unproject(&sensor_point, depth);
    /// ```
    pub fn unproject(&self, sensor_point: &Point2D, depth: f32) -> Option<BodyPoint> {
        if depth < self.depth_min || depth > self.depth_max || depth <= 0.0 {
            return None;
        }

        let depth_meters = depth * self.depth_scale;
        let x = (sensor_point.x - self.cx) * depth_meters as f64 / self.fx;
        let y = (sensor_point.y - self.cy) * depth_meters as f64 / self.fy;
        let z = depth_meters as f64;

        Some(Point3::new(x, y, z))
    }

    /// Returns the 3x3 projection matrix (K matrix).
    ///
    /// The projection matrix encodes the intrinsic parameters in matrix form:
    /// ```text
    /// K = [fx  0  cx]
    ///     [ 0 fy  cy]
    ///     [ 0  0   1]
    /// ```
    ///
    /// # Returns
    ///
    /// A 3x3 matrix representing the projection parameters.
    ///
    /// # Examples
    ///
    /// ```
    /// use aurus_common::ProjectionModel;
    ///
    /// let projection = ProjectionModel::new(525.0, 525.0, 320.0, 240.0, 640, 480);
    /// let k_matrix = projection.projection_matrix();
    /// assert_eq!(k_matrix[(0, 0)], 525.0);
    /// assert_eq!(k_matrix[(1, 1)], 525.0);
    /// ```
    pub fn projection_matrix(&self) -> Matrix3<f64> {
        Matrix3::new(self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0)
    }

    /// Checks if a depth value is valid according to the sensor parameters.
    ///
    /// # Arguments
    ///
    /// * `depth` - Depth value to validate (in raw sensor units)
    ///
    /// # Returns
    ///
    /// `true` if the depth is within valid range and finite, `false` otherwise.
    ///
    /// # Examples
    ///
    /// ```
    /// use aurus_common::ProjectionModel;
    ///
    /// let projection = ProjectionModel::new(525.0, 525.0, 320.0, 240.0, 640, 480);
    /// assert!(projection.is_valid_depth(1.0)); // 1 meter in scaled units
    /// assert!(!projection.is_valid_depth(0.0));   // Invalid
    /// assert!(!projection.is_valid_depth(f32::NAN)); // Invalid
    /// ```
    pub fn is_valid_depth(&self, depth: f32) -> bool {
        depth > self.depth_min && depth < self.depth_max && depth.is_finite()
    }
}

impl fmt::Display for Pose {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let t = self.translation();
        let _r = self.rotation();
        write!(f, "Pose(t=[{:.3}, {:.3}, {:.3}], R=...)", t.x, t.y, t.z)
    }
}

// Implement coordinate conversion traits for WorldPoint and GridPoint
impl<const D: usize> CoordinateConversion<f32, D> for WorldPoint<D> {
    fn to_svector(&self) -> nalgebra::SVector<f32, D> {
        let mut svector = nalgebra::SVector::<f32, D>::zeros();
        for i in 0..D {
            svector[i] = self.coords[i];
        }
        svector
    }

    fn from_svector(svector: &nalgebra::SVector<f32, D>) -> Self {
        let mut coords = [0.0f32; D];
        for i in 0..D {
            coords[i] = svector[i];
        }
        WorldPoint::from(coords)
    }
}

impl<const D: usize> CoordinateConversion<usize, D> for GridPoint<D> {
    fn to_svector(&self) -> nalgebra::SVector<usize, D> {
        let mut svector = nalgebra::SVector::<usize, D>::zeros();
        for i in 0..D {
            svector[i] = self.coords[i];
        }
        svector
    }

    fn from_svector(svector: &nalgebra::SVector<usize, D>) -> Self {
        let mut coords = [0usize; D];
        for i in 0..D {
            coords[i] = svector[i];
        }
        GridPoint::from(coords)
    }
}

impl<const D: usize> WorldPointExtGeneric<D> for WorldPoint<D> {
    fn to_navigation_coords(&self) -> nalgebra::SVector<f32, D> {
        self.to_svector()
    }

    fn from_navigation_coords(coords: &nalgebra::SVector<f32, D>) -> Self {
        Self::from_svector(coords)
    }
}

/// Convenience functions for seamless integration with navigation algorithms.
pub mod navigation {
    use super::*;

    /// Converts a path of SVectors to WorldPoints.
    pub fn svector_path_to_world_points<const D: usize>(
        path: &[nalgebra::SVector<f32, D>],
    ) -> Vec<WorldPoint<D>> {
        path.iter().map(WorldPoint::from_svector).collect()
    }

    /// Converts a path of WorldPoints to SVectors.
    pub fn world_points_to_svector_path<const D: usize>(
        path: &[WorldPoint<D>],
    ) -> Vec<nalgebra::SVector<f32, D>> {
        path.iter().map(WorldPoint::to_svector).collect()
    }

    /// Converts a path of SVectors to GridPoints.
    pub fn svector_path_to_grid_points<const D: usize>(
        path: &[nalgebra::SVector<usize, D>],
    ) -> Vec<GridPoint<D>> {
        path.iter().map(GridPoint::from_svector).collect()
    }

    /// Converts a path of GridPoints to SVectors.
    pub fn grid_points_to_svector_path<const D: usize>(
        path: &[GridPoint<D>],
    ) -> Vec<nalgebra::SVector<usize, D>> {
        path.iter().map(GridPoint::to_svector).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;

    #[test]
    fn test_point2d_bounds() {
        let point: Point2D = Point2::new(100.0, 200.0);
        assert!(point.is_in_bounds(640, 480));
        assert!(!point.is_in_bounds(50, 50));
    }

    #[test]
    fn test_body_projection() {
        let projection = ProjectionModel::new(500.0, 500.0, 320.0, 240.0, 640, 480);
        let body_point: BodyPoint = Point3::new(0.1, 0.2, 1.0);

        let projected = body_point.project(&projection).unwrap();
        assert!((projected.x - 370.0).abs() < 1e-10);
        assert!((projected.y - 340.0).abs() < 1e-10);
    }

    #[test]
    fn test_pose_composition() {
        let pose1 = Pose::from_translation(Vector3::new(1.0, 0.0, 0.0));
        let pose2 = Pose::from_translation(Vector3::new(0.0, 1.0, 0.0));

        let composed = pose1.compose(&pose2);
        let expected_translation = Vector3::new(1.0, 1.0, 0.0);

        assert!((composed.translation() - expected_translation).norm() < 1e-10);
    }

    #[test]
    fn test_world_to_body_transform() {
        let world_point: WorldPoint3D = Point3::new(1.0, 2.0, 3.0);
        let pose = Pose::from_translation(Vector3::new(0.5, 0.5, 0.5));

        let body_point = world_point.to_body(&pose);
        let back_to_world = body_point.to_world(&pose);

        assert!((world_point - back_to_world).norm() < 1e-10);
    }

    #[test]
    fn test_pose_exp_log_roundtrip() {
        use nalgebra::Vector6;

        // Test with a non-trivial tangent vector
        let tangent = Vector6::new(0.1, 0.2, 0.3, 1.0, 2.0, 3.0);
        let pose = Pose::exp(&tangent);
        let recovered = pose.log();

        assert!(
            (tangent - recovered).norm() < 1e-10,
            "Exp-log roundtrip failed: original={:?}, recovered={:?}",
            tangent,
            recovered
        );
    }

    #[test]
    fn test_pose_log_exp_roundtrip() {
        // Test with a non-trivial pose
        let rotation = Matrix3::new(0.866, -0.5, 0.0, 0.5, 0.866, 0.0, 0.0, 0.0, 1.0);
        let translation = Vector3::new(1.0, 2.0, 3.0);
        let pose = Pose::from_matrix_translation(rotation, translation);

        let tangent = pose.log();
        let recovered = Pose::exp(&tangent);

        assert!((pose.translation() - recovered.translation()).norm() < 1e-10);
        assert!((pose.rotation() - recovered.rotation()).norm() < 1e-10);
    }

    #[test]
    fn test_pose_interpolation() {
        let pose1 = Pose::from_translation(Vector3::new(0.0, 0.0, 0.0));
        let pose2 = Pose::from_translation(Vector3::new(2.0, 4.0, 6.0));

        // Test endpoints
        let start = pose1.interpolate(&pose2, 0.0);
        let end = pose1.interpolate(&pose2, 1.0);

        assert!((start.translation() - pose1.translation()).norm() < 1e-10);
        assert!((end.translation() - pose2.translation()).norm() < 1e-10);

        // Test midpoint
        let midpoint = pose1.interpolate(&pose2, 0.5);
        let expected_translation = Vector3::new(1.0, 2.0, 3.0);
        assert!((midpoint.translation() - expected_translation).norm() < 1e-10);
    }

    #[test]
    fn test_pose_lie_interpolation() {
        let pose1 = Pose::identity();
        let pose2 = Pose::from_translation(Vector3::new(1.0, 1.0, 1.0));

        // Test endpoints
        let start = pose1.interpolate_lie(&pose2, 0.0);
        let end = pose1.interpolate_lie(&pose2, 1.0);

        assert!((start.translation() - pose1.translation()).norm() < 1e-10);
        assert!((end.translation() - pose2.translation()).norm() < 1e-10);

        // Test that interpolation produces reasonable intermediate values
        let quarter = pose1.interpolate_lie(&pose2, 0.25);
        let expected_quarter = Vector3::new(0.25, 0.25, 0.25);
        assert!((quarter.translation() - expected_quarter).norm() < 1e-10);
    }

    #[test]
    fn test_small_angle_approximation() {
        use nalgebra::Vector6;

        // Test with very small rotation
        let small_tangent = Vector6::new(1e-8, 2e-8, 3e-8, 0.1, 0.2, 0.3);
        let pose = Pose::exp(&small_tangent);
        let recovered = pose.log();

        assert!(
            (small_tangent - recovered).norm() < 1e-12,
            "Small angle approximation failed"
        );
    }

    #[test]
    fn test_generalizable_dimensions() {
        // Test 2D world points
        let world_2d: WorldPoint<2> = Point2::new(1.0, 2.0);
        assert_eq!(world_2d.x, 1.0);
        assert_eq!(world_2d.y, 2.0);

        // Test 3D world points
        let world_3d: WorldPoint<3> = Point3::new(1.0, 2.0, 3.0);
        assert_eq!(world_3d.x, 1.0);
        assert_eq!(world_3d.y, 2.0);
        assert_eq!(world_3d.z, 3.0);

        // Test grid points
        let grid_2d: GridPoint<2> = Point2::new(10, 20);
        assert_eq!(grid_2d.x, 10);
        assert_eq!(grid_2d.y, 20);

        let grid_3d: GridPoint<3> = Point3::new(10, 20, 30);
        assert_eq!(grid_3d.x, 10);
        assert_eq!(grid_3d.y, 20);
        assert_eq!(grid_3d.z, 30);
    }

    #[test]
    fn test_coordinate_conversion() {
        use nalgebra::SVector;

        // Test WorldPoint conversion
        let world_2d: WorldPoint<2> = Point2::new(1.5, 2.5);
        let svector_2d = world_2d.to_svector();
        assert_eq!(svector_2d, SVector::<f32, 2>::new(1.5, 2.5));

        let world_back = WorldPoint::<2>::from_svector(&svector_2d);
        assert_eq!(world_back.x, 1.5);
        assert_eq!(world_back.y, 2.5);

        // Test GridPoint conversion
        let grid_3d: GridPoint<3> = Point3::new(10, 20, 30);
        let svector_3d = grid_3d.to_svector();
        assert_eq!(svector_3d, SVector::<usize, 3>::new(10, 20, 30));

        let grid_back = GridPoint::<3>::from_svector(&svector_3d);
        assert_eq!(grid_back.x, 10);
        assert_eq!(grid_back.y, 20);
        assert_eq!(grid_back.z, 30);
    }

    #[test]
    fn test_navigation_helpers() {
        use crate::navigation::*;
        use nalgebra::SVector;

        // Test world point path conversion
        let world_path = vec![
            WorldPoint::<2>::from([1.0, 2.0]),
            WorldPoint::<2>::from([3.0, 4.0]),
        ];

        let svector_path = world_points_to_svector_path(&world_path);
        assert_eq!(svector_path.len(), 2);
        assert_eq!(svector_path[0], SVector::<f32, 2>::new(1.0, 2.0));
        assert_eq!(svector_path[1], SVector::<f32, 2>::new(3.0, 4.0));

        let world_path_back = svector_path_to_world_points(&svector_path);
        assert_eq!(world_path_back.len(), 2);
        assert_eq!(world_path_back[0].x, 1.0);
        assert_eq!(world_path_back[0].y, 2.0);
        assert_eq!(world_path_back[1].x, 3.0);
        assert_eq!(world_path_back[1].y, 4.0);
    }
}
