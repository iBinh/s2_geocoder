use s2::{region::RegionCoverer, cap::Cap, cellid::CellID, latlng::LatLng, point::Point, s1::{chordangle::ChordAngle, angle::{Angle, Deg}}};
use ahash::AHashMap;
use croaring::Bitmap;

#[derive(Debug)]
pub struct World{
    pub cellid_map: AHashMap<u64, Bitmap>,
    pub min_level: u64,
    pub max_level: u64
}

impl World {
    pub fn new(min_level: u64, max_level: u64) -> Self {
        World { 
            cellid_map: AHashMap::<u64, Bitmap>::default(), 
            min_level,
            max_level
        }
    }
    ///insert item into world by latlng and id
    pub fn insert(&mut self, coords: (f64, f64), item_id: u32) {
        let latlng = LatLng::new(Angle::from(Deg(coords.0)), Angle::from(Deg(coords.1)));
        let mut id = CellID::from(latlng);        
        for i in self.min_level..=self.max_level {
            id = id.parent(i as u64);
            // println!("cellid level {}", id.level());
            let bitmap = self.cellid_map.entry(id.0).or_insert(Bitmap::default());
            bitmap.add(item_id);
        }
        
    }
    ///get bitmaps of item ids within a circle defined by a center(coords) and a radius in km. The input variable coords is defined as (latitude: f64, longitude: f64)
    pub fn within_radius(&self, coords: (f64, f64), radius: f64, max_cells: usize) -> Bitmap {
        let latlng = LatLng::new(Angle::from(Deg(coords.0)), Angle::from(Deg(coords.1)));
        let radius_in_deg = radius/111.0;
        let cap = Cap::from_center_chordangle(&Point::from(latlng), &ChordAngle::from(Deg(radius_in_deg)));
        let coverer = RegionCoverer {
            min_level: self.min_level as u8,
            max_level: self.max_level as u8,
            level_mod: 0,
            max_cells
        };
        let cell_union = coverer.covering(&cap);
        let empty_bitmap = Bitmap::default();
        let bitmaps = cell_union.0.iter().map(|i| self.cellid_map.get(&i.0).unwrap_or(&empty_bitmap)).collect::<Vec<_>>();
        let result = Bitmap::fast_or_heap(&bitmaps);
        result
    }
}

impl Default for World {
    fn default() -> Self {
        Self::new(9, 12)
    }
}

