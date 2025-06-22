//! Unified search interface for different search algorithms
//!
//! This module provides a unified interface that can switch between
//! different search algorithms at runtime.

use super::KdTree;
use super::traits::{NearestNeighborSearch, SearchConfiguration, SearchInputCloud, SearchMethod};
use crate::common::{PointCloud, PointXYZ, PointXYZRGB};
use crate::error::{PclError, PclResult};

/// Unified search interface for PointXYZ
pub enum SearchXYZ {
    /// KD-tree search
    KdTree(KdTree<PointXYZ>),
    // Future: Octree, FLANN, etc.
}

impl SearchXYZ {
    /// Create a new search instance with the specified method
    pub fn new(method: SearchMethod) -> PclResult<Self> {
        match method {
            SearchMethod::KdTree => Ok(SearchXYZ::KdTree(KdTree::<PointXYZ>::new()?)),
            _ => Err(PclError::not_implemented(
                format!("{} search for PointXYZ", method.name()),
                Some("Use KdTree method for now".to_string()),
            )),
        }
    }

    /// Get the current search method
    pub fn method(&self) -> SearchMethod {
        match self {
            SearchXYZ::KdTree(_) => SearchMethod::KdTree,
        }
    }
}

impl NearestNeighborSearch<PointXYZ> for SearchXYZ {
    fn nearest_k_search(&self, point: &PointXYZ, k: i32) -> PclResult<Vec<i32>> {
        match self {
            SearchXYZ::KdTree(kdtree) => kdtree.nearest_k_search(point, k),
        }
    }

    fn nearest_k_search_with_distances(
        &self,
        point: &PointXYZ,
        k: i32,
    ) -> PclResult<(Vec<i32>, Vec<f32>)> {
        match self {
            SearchXYZ::KdTree(kdtree) => kdtree.nearest_k_search_with_distances(point, k),
        }
    }

    fn radius_search(&self, point: &PointXYZ, radius: f64) -> PclResult<Vec<i32>> {
        match self {
            SearchXYZ::KdTree(kdtree) => kdtree.radius_search(point, radius),
        }
    }

    fn radius_search_with_distances(
        &self,
        point: &PointXYZ,
        radius: f64,
    ) -> PclResult<(Vec<i32>, Vec<f32>)> {
        match self {
            SearchXYZ::KdTree(kdtree) => kdtree.radius_search_with_distances(point, radius),
        }
    }
}

impl SearchConfiguration for SearchXYZ {
    fn epsilon(&self) -> f32 {
        match self {
            SearchXYZ::KdTree(kdtree) => kdtree.epsilon(),
        }
    }

    fn set_epsilon(&mut self, epsilon: f32) -> PclResult<()> {
        match self {
            SearchXYZ::KdTree(kdtree) => kdtree.set_epsilon(epsilon),
        }
    }
}

impl SearchInputCloud<PointCloud<PointXYZ>> for SearchXYZ {
    fn set_input_cloud(&mut self, cloud: &PointCloud<PointXYZ>) -> PclResult<()> {
        match self {
            SearchXYZ::KdTree(kdtree) => kdtree.set_input_cloud(cloud),
        }
    }

    fn has_input_cloud(&self) -> bool {
        match self {
            SearchXYZ::KdTree(kdtree) => kdtree.has_input_cloud(),
        }
    }
}

/// Unified search interface for PointXYZRGB
pub enum SearchXYZRGB {
    /// KD-tree search
    KdTree(KdTree<PointXYZRGB>),
    // Future: Octree, FLANN, etc.
}

impl SearchXYZRGB {
    /// Create a new search instance with the specified method
    pub fn new(method: SearchMethod) -> PclResult<Self> {
        match method {
            SearchMethod::KdTree => Ok(SearchXYZRGB::KdTree(KdTree::<PointXYZRGB>::new()?)),
            _ => Err(PclError::not_implemented(
                format!("{} search for PointXYZRGB", method.name()),
                Some("Use KdTree method for now".to_string()),
            )),
        }
    }

    /// Get the current search method
    pub fn method(&self) -> SearchMethod {
        match self {
            SearchXYZRGB::KdTree(_) => SearchMethod::KdTree,
        }
    }
}

impl NearestNeighborSearch<PointXYZRGB> for SearchXYZRGB {
    fn nearest_k_search(&self, point: &PointXYZRGB, k: i32) -> PclResult<Vec<i32>> {
        match self {
            SearchXYZRGB::KdTree(kdtree) => kdtree.nearest_k_search(point, k),
        }
    }

    fn nearest_k_search_with_distances(
        &self,
        point: &PointXYZRGB,
        k: i32,
    ) -> PclResult<(Vec<i32>, Vec<f32>)> {
        match self {
            SearchXYZRGB::KdTree(kdtree) => kdtree.nearest_k_search_with_distances(point, k),
        }
    }

    fn radius_search(&self, point: &PointXYZRGB, radius: f64) -> PclResult<Vec<i32>> {
        match self {
            SearchXYZRGB::KdTree(kdtree) => kdtree.radius_search(point, radius),
        }
    }

    fn radius_search_with_distances(
        &self,
        point: &PointXYZRGB,
        radius: f64,
    ) -> PclResult<(Vec<i32>, Vec<f32>)> {
        match self {
            SearchXYZRGB::KdTree(kdtree) => kdtree.radius_search_with_distances(point, radius),
        }
    }
}

impl SearchConfiguration for SearchXYZRGB {
    fn epsilon(&self) -> f32 {
        match self {
            SearchXYZRGB::KdTree(kdtree) => kdtree.epsilon(),
        }
    }

    fn set_epsilon(&mut self, epsilon: f32) -> PclResult<()> {
        match self {
            SearchXYZRGB::KdTree(kdtree) => kdtree.set_epsilon(epsilon),
        }
    }
}

impl SearchInputCloud<PointCloud<PointXYZRGB>> for SearchXYZRGB {
    fn set_input_cloud(&mut self, cloud: &PointCloud<PointXYZRGB>) -> PclResult<()> {
        match self {
            SearchXYZRGB::KdTree(kdtree) => kdtree.set_input_cloud(cloud),
        }
    }

    fn has_input_cloud(&self) -> bool {
        match self {
            SearchXYZRGB::KdTree(kdtree) => kdtree.has_input_cloud(),
        }
    }
}
