use std::collections::{HashMap};
use std::time::SystemTime;

use petgraph::Graph;
use petgraph::prelude::*;
use crate::types::{Coordinate, Map};
use docopt::Docopt;

mod algorithms;
mod types;

fn main() {
    const USAGE: &'static str = "Ende Matrix Generator

    Usage:
    ende [options] <source.osm.pbf> <locations.json>
    ende -h | --help

    Options:
    -h --help      Show this screen";

    let args = Docopt::new(USAGE).unwrap().parse().unwrap_or_else(|e| e.exit());

    let now = SystemTime::now();

    let coordinate_input: Vec<(f64, f64)> = vec![(40.424725, -3.690438), (40.421096, -3.688578)];
    println!("Generating a matrix for {} locations", coordinate_input.len());

    println!("Parse map file started.");

    let file_name: String = String::from(args.get_str("<source.osm.pbf>"));
    let map = Map::from(&file_name);
    println!(" ✓ duration: {}s\n", now.elapsed().unwrap().as_secs());

    let mut graph: Graph<(), f64, Directed> = Graph::new();
    let mut nodes_on_graph = HashMap::new();
    let mut index_on_graph = HashMap::new();
    for node in &map.nodes {
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
    println!(" ✓ duration: {}s\n", now.elapsed().unwrap().as_secs());

    let mut coordinates: Vec<Coordinate> = Vec::new();
    println!("Reverse geocoding coordinates...");
    for coordinate in coordinate_input {
        let mut coord = Coordinate::from(coordinate);
        let node = map.reverse_geocode_node(coord);
        coord.map_node = Some(node);
        coord.graph_node = Some(*nodes_on_graph.get(&node.id).unwrap());
        coordinates.push(coord);
    }
    println!("Reverse geocoding complete.");
    println!(" ✓ duration: {}s\n", now.elapsed().unwrap().as_secs());

    let mut distances: Vec<Vec<f64>> = Vec::new();
    let mut durations: Vec<Vec<f64>> = Vec::new();

    println!("Generating matrix...");
    &coordinates.iter().for_each(|origin_coordinate| {
        let start_node = nodes_on_graph.get(&origin_coordinate.map_node.unwrap().id).unwrap().clone();
        let (distance, duration) = find_distances(start_node, (*coordinates).to_owned(), nodes_on_graph.clone(), graph.clone());

        distances.push(distance);
        durations.push(duration);
    });

    println!("Distances: {:?}", distances);
    println!("Durations: {:?}\n", durations);
    println!(" ✓ Total duration: {}s", now.elapsed().unwrap().as_secs());
}

fn find_distances(
    origin_node: NodeIndex,
    coordinates: Vec<Coordinate>,
    nodes_on_graph: HashMap<i64, NodeIndex>,
    graph: Graph<(), f64, Directed>,
) -> (Vec<f64>, Vec<f64>) {
    let mut distance: Vec<f64> = Vec::new();
    let mut duration: Vec<f64> = Vec::new();

    let res: HashMap<NodeIndex, f64> = algorithms::astar_multiple_goals(
        &graph,
        origin_node,
        |finish| coordinates.iter().map(|coordinate| nodes_on_graph.get(&coordinate.map_node.unwrap().id)).any(|node_index| node_index == Some(&finish)),
        |e| *e.weight(),
        |_| 0.
    );

    // let res = algorithms::dijkstra(
    //     &graph,
    //     origin_node,
    //     |finish| coordinates.iter().map(|coordinate| nodes_on_graph.get(&coordinate.node.unwrap().id)).any(|node_index| node_index == Some(&finish)),
    //     |e| *e.weight()
    // );
    // for (node_id, result) in &res {
    //     let cost = result.unwrap();
    //     println!("Found cost: {:?}", cost);
    // }
    // println!("Found costs: {:?}", res.len());
    const AVG_VEHICLE_SPEED: f64 = 25.00;

    for coordinate in &coordinates {
        match res.get(&coordinate.graph_node.unwrap()) {
            Some(cost) => {
                distance.push(*cost);
                duration.push(f64::trunc(cost / AVG_VEHICLE_SPEED));
            }
            None => {
                distance.push(0.);
                duration.push(0.);
            }
        }
    }
    return (distance, duration)
}