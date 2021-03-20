use petgraph::Graph;
use petgraph::algo::{astar};
use petgraph::prelude::*;
use std::collections::{BTreeMap};
use std::time::SystemTime;
use osm4routing::models::{Node, Edge};

fn main() {
    let now = SystemTime::now();
    let file_name: String = String::from("mad1.osm.pbf");

    let map = Map::from(&file_name);
    let mut graph: Graph<(), f64, Directed> = Graph::new();
    let mut nodes_on_graph = BTreeMap::new();
    let mut index_on_graph = BTreeMap::new();

    let coordinate_input: Vec<(f64, f64)> = vec![
        (40.424725, -3.690438),
        (40.421096, -3.688578),
        (40.421834, -3.693954)
    ];
    let mut coordinates: Vec<Coordinate> = Vec::new();
    for coordinate in coordinate_input {
        let mut coord = Coordinate::from(coordinate);
        let node = map.reverse_geocode_node(coord);
        coord.node = Some(node);
        coordinates.push(coord);
    }

    for node in map.nodes {
        let graph_node = graph.add_node(());
        nodes_on_graph.insert(node.id, graph_node);
        index_on_graph.insert(graph_node, node);
    }

    for edge in map.edges.iter().filter(|&e| {
        e.properties.car_forward >= 1
    }) {
        let start = nodes_on_graph.get(&edge.source).unwrap().clone();
        let end = nodes_on_graph.get(&edge.target).unwrap().clone();
        let weight = edge.length();
        graph.add_edge(start, end, weight);
    }
    println!("Nodes on graph, {}", graph.node_count());
    println!("Edges on graph, {}", graph.edge_count());

    let mut result: Vec<Vec<f64>> = Vec::new();

    for origin_coordinate in &coordinates {
        let start_node = nodes_on_graph.get(&origin_coordinate.node.unwrap().id).unwrap().clone();
        let mut distances: Vec<f64> = Vec::new();

        for destination_coordinate in &coordinates {
            let end_node = nodes_on_graph.get(&destination_coordinate.node.unwrap().id).unwrap().clone();
            let res = astar(&graph, start_node, |finish| finish == end_node, |e| *e.weight(), |_| 0.);
            let (cost, _path) = res.unwrap();
            distances.push(cost);
        }

        result.push(distances);
    }

    println!(" ✓ duration: {}s\n", now.elapsed().unwrap().as_secs());
    println!("Costs: {:?}s\n", result);

}

struct Map {
    nodes: Vec<Node>,
    edges: Vec<Edge>,
}

impl From<&String> for Map {
    fn from(file_name: &String) -> Map {
        let (nodes, edges) = osm4routing::reader::read(file_name).expect("Read OSM file");
        Self { nodes, edges }
    }
}

impl Map {
    fn reverse_geocode_node(&self, coordinate: Coordinate) -> Node {
        let mut min_distance: Option<f64> = None;
        let mut node: Option<Node> = None;
        for current_node in self.nodes.clone() {
            let calc_distance: Option<f64> = Some(
                self.haversine_distance(
                    coordinate.lat,
                    coordinate.lon,
                    current_node.coord.lat,
                    current_node.coord.lon
                )
            );
            if min_distance.is_none() {
                min_distance = calc_distance;
                node = Some(current_node);
            } else if calc_distance.unwrap() < min_distance.unwrap() {
                min_distance = calc_distance;
                node = Some(current_node);
            }
        }
        return node.unwrap();
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
        return dist;
    }
}

#[derive(Clone, Copy)]
struct Coordinate {
    lat: f64,
    lon: f64,
    node: Option<Node>,
}

impl From<(f64, f64)> for Coordinate {
    fn from(coordinate: (f64, f64)) -> Coordinate {
        Self { lat: coordinate.0, lon: coordinate.1, node: None }
    }
}



