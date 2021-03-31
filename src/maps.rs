use core::convert::From;
use core::option::Option;
use core::option::Option::{None, Some};
use osm4routing::models::{Edge, Node};
use petgraph::graph::NodeIndex;

pub struct Map {
    pub nodes: Vec<Node>,
    pub edges: Vec<Edge>,
}

impl From<&String> for Map {
    fn from(file_name: &String) -> Map {
        let (nodes, edges) = osm4routing::reader::read(file_name).expect("Read OSM file");
        Self { nodes, edges }
    }
}

impl Map {
    pub fn reverse_geocode_node(&self, coordinate: Coordinate) -> Node {
        let mut min_distance: Option<f64> = None;
        let mut node: Option<Node> = None;
        for current_node in self.nodes.clone() {
            let calc_distance: f64 = self.haversine_distance(
                coordinate.lat,
                coordinate.lon,
                current_node.coord.lat,
                current_node.coord.lon,
            );
            if min_distance.is_none() || calc_distance < min_distance.unwrap() {
                min_distance = Some(calc_distance);
                node = Some(current_node);
            }
        }
        node.unwrap()
    }

    fn haversine_distance(&self, origin_lat: f64, origin_lon: f64, dest_lat: f64, dest_lon: f64) -> f64 {
        static R: f64 = 6372.8;
        let diff = origin_lon - dest_lon;
        let ori_lon = diff.to_radians();
        let ori_lat = origin_lat.to_radians();
        let dest_lat = dest_lat.to_radians();
        let dz: f64 = ori_lat.sin() - dest_lat.sin();
        let dx: f64 = ori_lon.cos() * ori_lat.cos() - dest_lat.cos();
        let dy: f64 = ori_lon.sin() * ori_lat.cos();
        let dist: f64 = ((dx * dx + dy * dy + dz * dz).sqrt() / 2.0).asin() * 2.0 * R;
        dist
    }
}

#[derive(Clone, Copy)]
pub struct Coordinate {
    lat: f64,
    lon: f64,
    pub map_node: Option<Node>,
    pub graph_node: Option<NodeIndex>,
}

impl From<(f64, f64)> for Coordinate {
    fn from(coordinate: (f64, f64)) -> Coordinate {
        Self { lat: coordinate.0, lon: coordinate.1, map_node: None, graph_node: None }
    }
}
