use crate::maps::{Coordinate, Map};
use petgraph::{Graph, Directed};
use petgraph::prelude::*;
use std::collections::HashMap;
use crate::{algorithms, AVG_VEHICLE_SPEED};
use std::time::SystemTime;

pub struct MatrixResult {
    pub distances: Vec<Vec<f64>>,
    pub durations: Vec<Vec<f64>>
}

pub struct OpenRouteGraph {
    map: Map,
    graph: Graph<(), f64, Directed>,
    nodes_on_graph: HashMap<i64, NodeIndex>,
}

impl OpenRouteGraph {
    pub fn new_from_map(map: Map) -> Self {
        let mut graph: Graph<(), f64, Directed> = Graph::new();
        let mut nodes_on_graph = HashMap::new();
        for node in &map.nodes {
            let graph_node = graph.add_node(());
            nodes_on_graph.insert(node.id, graph_node);
        }

        for edge in map.edges.iter().filter(|&e| {
            e.properties.car_forward >= 1
        }) {
            let start = *nodes_on_graph.get(&edge.source).unwrap();
            let end = *nodes_on_graph.get(&edge.target).unwrap();
            let weight = edge.length();
            graph.add_edge(start, end, weight);
        }
        Self {map, graph, nodes_on_graph}
    }
}

pub struct Matrix;

impl Matrix {

    pub fn build(map: Map, locations: Vec<(f64, f64)>) -> MatrixResult {
        let now = SystemTime::now();

        println!(" ✓ duration: {}s\n", now.elapsed().unwrap().as_secs());

        let graph = OpenRouteGraph::new_from_map(map);
        println!("Nodes on graph, {}",graph.graph.node_count());
        println!("Edges on graph, {}",graph.graph.edge_count());
        println!(" ✓ duration: {}s\n", now.elapsed().unwrap().as_secs());

        let mut coordinates: Vec<Coordinate> = Vec::new();
        println!("Reverse geocoding coordinates...");
        for coordinate in locations {
            let mut coord = Coordinate::from(coordinate);
            let node = graph.map.reverse_geocode_node(coord);
            coord.map_node = Some(node);
            coord.graph_node = Some(*graph.nodes_on_graph.get(&node.id).unwrap());
            coordinates.push(coord);
        }
        println!("Reverse geocoding complete.");
        println!(" ✓ duration: {}s\n", now.elapsed().unwrap().as_secs());

        let mut distances: Vec<Vec<f64>> = Vec::new();
        let mut durations: Vec<Vec<f64>> = Vec::new();

        println!("Generating matrix...");
        coordinates.iter().for_each(|origin_coordinate| {
            let start_node = *graph.nodes_on_graph.get(&origin_coordinate.map_node.unwrap().id).unwrap();
            let (distance, duration) = calculate_costs(
                start_node, (*coordinates).to_owned(), graph.nodes_on_graph.clone(), graph.graph.clone());

            distances.push(distance);
            durations.push(duration);
        });
        let result = MatrixResult{distances, durations};
        println!(" ✓ Total duration: {}s", now.elapsed().unwrap().as_secs());
        result
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
    (distance, duration)
}


#[test]
fn test_results() {
    let file_name = &String::from("benchmarks/small/mad1.osm.pbf");
    let coordinate_input = vec![(40.424725, -3.690438), (40.421096, -3.688578)];
    let map = Map::from(file_name);
    let matrix_result = Matrix::build(map, coordinate_input);
    let durations_result = [[0.0, 20.0], [32.0, 0.0]];
    let distances_result = [[0.0, 523.5572339611438], [808.9896429889042, 0.0]];

    assert_eq!(matrix_result.distances.len(), 2);
    assert_eq!(&matrix_result.durations, &durations_result);
    assert_eq!(&matrix_result.distances, &distances_result);
}