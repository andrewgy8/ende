use petgraph::Graph;
use petgraph::algo::{astar};
use petgraph::prelude::*;
use std::collections::{BTreeMap};
use std::time::SystemTime;
use std::process;
use osm4routing::models::{Node, Edge};
use haversiner::{haversine, Point, Measure};
use std::fs::OpenOptions;
use std::cmp::min;

fn main() {
    let file_name = "mad1.osm.pbf";
    let now = SystemTime::now();

    let (nodes, edges) = parse_graph(file_name);

    let mut graph: Graph<(), f64, Directed> = Graph::new();
    let mut nodes_on_graph = BTreeMap::new();
    let mut index_on_graph = BTreeMap::new();

    let start_coordinates = (40.424725, -3.690438);
    let end_coordinates = (40.421096, -3.688578);
    let inferred_start_node = get_node_for_coordinates(start_coordinates.0, start_coordinates.1, nodes.clone());
    let inferred_end_node = get_node_for_coordinates(end_coordinates.0, end_coordinates.1, nodes.clone());

    let START_NODE_ID: i64 = 25903298;
    let END_NODE_ID: i64 = 942080448;
    assert_eq!(inferred_start_node.id, START_NODE_ID);
    assert_eq!(inferred_end_node.id, END_NODE_ID);
    let mut end_node = None;

    for node in nodes {
        let new_node = graph.add_node(());
        nodes_on_graph.insert(node.id, new_node);
        index_on_graph.insert(new_node, node);
        if node.id == END_NODE_ID {
            end_node = Some(new_node);
        }
    }
    let start_node = nodes_on_graph.get(&START_NODE_ID).unwrap().clone();
    if end_node == None {
        println!("The node {} is unknown. Are you sure it is the intersection of two roads?",
                 END_NODE_ID);
        process::exit(1);
    }

    for edge in edges.iter().filter(|&e| {
        e.properties.car_forward >= 1
    }) {
        let start = nodes_on_graph.get(&edge.source).unwrap().clone();
        let end = nodes_on_graph.get(&edge.target).unwrap().clone();
        let weight = edge.length();
        graph.add_edge(start, end, weight);
    }

    println!("Nodes on graph, {}", graph.node_count());
    println!("Edges on graph, {}", graph.edge_count());
    println!(" ✓ duration: {}s\n", now.elapsed().unwrap().as_secs());

    let res = astar(&graph, start_node, |finish| finish == end_node.unwrap(), |e| *e.weight(),  |_| 0.);
    let (cost, path) = res.unwrap();
    println!(" ✓ duration: {}s\n", now.elapsed().unwrap().as_secs());
    println!("Path count: {}\n", path.len());
    println!("Result cost: {}\n", cost);


    for value in path {
        let node = index_on_graph.get(&value).unwrap().clone();

        println!("{:?}, {:?}", node.coord.lat, node.coord.lon);
    }


// z is not inside res because there is not path from b to z.
}


fn parse_graph(file_name: &str) -> (Vec<Node>, Vec<Edge>) {
    let (nodes, edges) = osm4routing::reader::read(file_name).expect("Read OSM file");
    return (nodes, edges);
}

fn get_node_for_coordinates(lat: f64, lon: f64, nodes: Vec<Node>) -> Node {
    let mut min_distance: Option<f64> = None;
    let mut node: Option<Node> = None;
    for current_node in nodes {
        let calc_distance: Option<f64> = Some(haversine_distance(lat, lon, current_node.coord.lat, current_node.coord.lon));
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

fn haversine_distance(origin_lat: f64, origin_lon: f64, dest_lat: f64, dest_lon: f64) -> f64 {
    static R: f64 = 6372.8;
    let diff = origin_lon - dest_lon;
    let ori_lon = diff.to_radians();
    let ori_lat = origin_lat.to_radians();
    let dest_lat = dest_lat.to_radians();
    let dz: f64 = ori_lat.sin() - dest_lat.sin();
    let dx: f64 = ori_lon.cos() * ori_lat.cos() - dest_lat.cos();
    let dy: f64 = ori_lon.sin() * ori_lat.cos();
    let dist: f64 = ((dx * dx + dy * dy + dz * dz).sqrt() / 2.0).asin() * 2.0 * R;
    return dist
}