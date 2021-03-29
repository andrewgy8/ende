use crate::types::{Coordinate, Map};
use petgraph::{Graph, Directed};
use petgraph::prelude::*;
use std::collections::HashMap;
use crate::{algorithms, AVG_VEHICLE_SPEED};
use std::time::SystemTime;

pub struct MatrixResult {
    pub(crate) distances: Vec<Vec<f64>>,
    pub(crate) durations: Vec<Vec<f64>>
}

pub struct Matrix;

impl Matrix {

    pub fn build(file_name: String, locations: Vec<(f64, f64)>) -> MatrixResult {
        let now = SystemTime::now();
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
        for coordinate in locations {
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
            let (distance, duration) = calculate_costs(
                start_node,
                (*coordinates).to_owned(),
                nodes_on_graph.clone(),
                graph.clone()
            );

            distances.push(distance);
            durations.push(duration);
        });
        let result = MatrixResult{distances, durations};
        println!(" ✓ Total duration: {}s", now.elapsed().unwrap().as_secs());
        return result;
    }

}


fn calculate_costs(
    origin_node: NodeIndex,
    coordinates: Vec<Coordinate>,
    nodes_on_graph: HashMap<i64, NodeIndex>,
    graph: Graph<(), f64, Directed>,
) -> (Vec<f64>, Vec<f64>) {
    let mut distance: Vec<f64> = Vec::new();
    let mut duration: Vec<f64> = Vec::new();
    let is_goal = |finish| {
        coordinates.iter().map(|coordinate| nodes_on_graph.get(&coordinate.map_node?.id)).any(|node_index| node_index == Some(&finish))
    };
    let res: HashMap<NodeIndex, f64> = algorithms::astar_multiple_goals(
        &graph,
        origin_node,
        is_goal,
        |e| *e.weight(),
        |_| 0.
    );

    // let res = algorithms::dijkstra(
    //     &graph,
    //     origin_node,
    //     is_goal,
    //     |e| *e.weight()
    // );

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