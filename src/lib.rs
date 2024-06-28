#![allow(dead_code)]
#![allow(unused_imports)]

use ahash::AHashMap;
use croaring::Bitmap;
use rstar::primitives::{GeomWithData, Line};
use rstar::*;
use s2::{
    cap::Cap,
    cellid::CellID,
    latlng::LatLng,
    point::Point,
    region::RegionCoverer,
    s1::{
        angle::{Angle, Deg},
        chordangle::ChordAngle,
    },
};

pub struct World {
    pub cellid_map: AHashMap<u64, Bitmap>,
    pub min_level: u64,
    pub max_level: u64,
    pub tree: Option<RTree<TreePoint>>,
    pub shape_tree: RTree<GeomWithData<Line<(f64, f64)>, u32>>,
}
pub struct TreePoint(pub f64, pub f64, pub u32);
impl RTreeObject for TreePoint {
    type Envelope = AABB<[f64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        AABB::from_point([self.0, self.1])
    }
}
impl PointDistance for TreePoint {
    #[inline(always)]
    fn distance_2(&self, point: &[f64; 2]) -> f64 {
        let dx = self.0 - point[0];
        let dy = self.1 - point[1];
        dx * dx + dy * dy
    }
    #[inline(always)]
    fn contains_point(&self, point: &[f64; 2]) -> bool {
        [self.1, self.0].contains_point(point)
    }
}
impl World {
    pub fn new(min_level: u64, max_level: u64) -> Self {
        World {
            cellid_map: AHashMap::<u64, Bitmap>::default(),
            min_level,
            max_level,
            tree: None,
            shape_tree: RTree::new(),
        }
    }
    ///Build the reverse geocode index from a list of points
    pub fn build_reverse_geocode_index(&mut self, points: Vec<TreePoint>) {
        println!(
            "Building reverse geocode index with {} points",
            points.len()
        );
        let rstar = RTree::bulk_load(points);
        self.tree = Some(rstar);
    }
    ///insert item into world by latlng and id
    pub fn insert(&mut self, coords: (f64, f64), item_id: u32) {
        let latlng = LatLng::new(Angle::from(Deg(coords.0)), Angle::from(Deg(coords.1)));
        let mut id = CellID::from(latlng);
        for i in self.min_level..=self.max_level {
            id = id.parent(i as u64);
            // println!("cellid level {}", id.level());
            let bitmap = self.cellid_map.entry(id.0).or_insert_with(Bitmap::default);
            bitmap.add(item_id);
        }
    }
    ///get bitmaps of item ids within a circle defined by a center(coords) and a radius in km. The input variable coords is defined as (latitude: f64, longitude: f64)
    ///the output is a bitmap of item ids
    pub fn within_radius(&self, coords: (f64, f64), radius: f64, max_cells: usize) -> Bitmap {
        let latlng = LatLng::new(Angle::from(Deg(coords.0)), Angle::from(Deg(coords.1)));
        let radius_in_deg = radius / 111.0;
        let cap = Cap::from_center_chordangle(
            &Point::from(latlng),
            &ChordAngle::from(Deg(radius_in_deg)),
        );
        let coverer = RegionCoverer {
            min_level: self.min_level as u8,
            max_level: self.max_level as u8,
            level_mod: 0,
            max_cells,
        };
        let cell_union = coverer.covering(&cap);
        let empty_bitmap = Bitmap::default();
        let bitmaps = cell_union
            .0
            .iter()
            .map(|i| self.cellid_map.get(&i.0).unwrap_or(&empty_bitmap))
            .collect::<Vec<_>>();
        Bitmap::fast_or_heap(&bitmaps)
    }
    ///get nearest items id to a given latlng
    pub fn nearest_vec(&self, (lat, lon): (f64, f64), limit: usize) -> std::io::Result<Vec<u32>> {
        match self.tree {
            Some(ref tree) => Ok(tree
                .nearest_neighbor_iter(&[lat, lon])
                .map(|point| point.2)
                .take(limit)
                .collect::<Vec<u32>>()),
            None => Err(std::io::Error::new(
                std::io::ErrorKind::Other,
                "No tree found",
            )),
        }
    }

    pub fn nearest(&self, (lat, lon): (f64, f64), limit: usize) -> std::io::Result<u32> {
        match self.tree {
            Some(ref tree) => match tree.nearest_neighbor(&[lat, lon]).map(|point| point.2) {
                Some(id) => Ok(id),
                None => Err(std::io::Error::new(std::io::ErrorKind::Other, "No result")),
            },
            None => Err(std::io::Error::new(
                std::io::ErrorKind::Other,
                "No tree found",
            )),
        }
    }

    ///insert line into world by Vec<latlng> and id
    pub fn insert_shape(&mut self, coords: Vec<(f64, f64)>, item_id: u32) {
        if coords.len() < 2 {
            return;
        }
        for line in coords.windows(2) {
            self.shape_tree
                .insert(GeomWithData::new(Line::new(line[0], line[1]), item_id))
        }
    }
    
    ///insert line into world by Vec<<Veclatlng>> and id
    pub fn insert_shapes(&mut self, shapes: Vec<Vec<(f64, f64)>>, item_id: u32) {
        for coords in shapes {
            if coords.len() < 2 {
                return;
            }
            for line in coords.windows(2) {
                self.shape_tree
                    .insert(GeomWithData::new(Line::new(line[0], line[1]), item_id))
            }
        }
        
    }

    ///get nearest geometric shape id from point
    pub fn nearest_shape(&self, (lat, lon): (f64, f64)) -> Option<u32> {
        self.shape_tree
            .nearest_neighbor(&(lat, lon))
            .map(|line| {
                return line.data;
            })
    }
    ///get nearest geometric shape ids from point
    pub fn nearest_shapes(&self, (lat, lon): (f64, f64)) -> Option<Vec<u32>> {
        let shapes = self.shape_tree
            .nearest_neighbors(&(lat, lon));

        if shapes.is_empty() {
            return None;
        }

        let mut ids = Vec::new();
        for shape in shapes {
            ids.push(shape.data);
        }
        Some(ids)
    }
    ///get geometric shape ids within a given radius
    pub fn shapes_within_radius(&self, (lat, lon): (f64, f64), radius: f64) -> Vec<u32> {
        self.shape_tree
            .locate_within_distance((lat, lon), radius * radius)
            .map(|line| line.data)
            .collect::<Vec<_>>()
    }
    ///calculate the squared euclidean distance between two points
    #[inline(always)]
    pub fn squared_euclidean_distance(p1: (f64, f64), p2: (f64, f64)) -> f64 {
        let dx = p1.0 - p2.0;
        let dy = p1.1 - p2.1;
        dx * dx + dy * dy
    }
}

impl Default for World {
    fn default() -> Self {
        Self::new(9, 12)
    }
}

pub struct KdWorld {}
