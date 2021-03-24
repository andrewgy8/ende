mod algorithms;

use petgraph::Graph;
use petgraph::algo::{astar};
use petgraph::prelude::*;
use std::collections::{BTreeMap};
use std::time::SystemTime;
use osm4routing::models::{Node, Edge};
use rayon::prelude::*;

fn main() {
    let now = SystemTime::now();
    println!("Parse map file started.");
    let file_name: String = String::from("Madrid.osm.pbf");

    let map = Map::from(&file_name);
    println!(" ✓ duration: {}s\n", now.elapsed().unwrap().as_secs());

    let mut graph: Graph<(), f64, Directed> = Graph::new();
    let mut nodes_on_graph = BTreeMap::new();
    let mut index_on_graph = BTreeMap::new();

    let coordinate_input: Vec<(f64, f64)> = vec![(40.44296164614941, -3.6554814874879336), (40.43657544164316, -3.6810450713776803)
                                                 // , (40.43678125091761, -3.7127972838801626), (40.47437815625357, -3.6720881994873142), (40.44093236781111, -3.6486800457949955), (40.43655857777823, -3.7324250229132856), (40.45934656112994, -3.667711297410533), (40.45369625162689, -3.7047368004817987), (40.44750054404099, -3.664309051901257), (40.45890400235908, -3.6461039894650553), (40.466442329845485, -3.712673310392273), (40.47159852109352, -3.687173719968859), (40.45984855330422, -3.6381376172657793), (40.40294172937246, -3.653150039295446), (40.46293214530529, -3.710764632188618), (40.434608768974876, -3.7148919081473037), (40.4656598904786, -3.6741083901443616), (40.41645342873255, -3.6642917017493803), (40.44439792790308, -3.696821644629975), (40.438757400038824, -3.7281947632912127), (40.45662882542248, -3.647857186715173), (40.40702552957198, -3.721561069648268), (40.40998585047522, -3.6509505851024784), (40.45551348806151, -3.649310933176554), (40.44518297130619, -3.690431200080588), (40.452126724581, -3.728948356072997), (40.39794200840548, -3.662867901333583), (40.409334289849085, -3.693855371402283), (40.44760712511513, -3.721232673793561), (40.39896376140905, -3.673907977550483), (40.4544670836956, -3.6430798035359464), (40.42205880081204, -3.68094250674519), (40.451711941211336, -3.644835082618932), (40.41652101823193, -3.640240232238668), (40.446028751809465, -3.6843838579559485), (40.445180047297185, -3.683094600967046), (40.4365706023023, -3.65410796790603), (40.42205731266119, -3.73516228876485), (40.40290685963453, -3.6489625692890972), (40.44626852378054, -3.6690872441858726), (40.44352330827034, -3.7321946746577144), (40.39755237245202, -3.651806173590089), (40.43976684796896, -3.7243477077628855), (40.44472370643311, -3.6757169534617384), (40.42443897676289, -3.7243558140593294), (40.40098296670438, -3.705289504377196), (40.4190228814084, -3.672063270181507), (40.4242116537873, -3.6582712522452328), (40.40395663619917, -3.6521284218051644), (40.4697485962488, -3.6560121589236028)
    ];
    println!("Generating a matrix for {} locations", coordinate_input.len());
    let mut coordinates: Vec<Coordinate> = Vec::new();

    println!("Retrieving nodes for coordinates");
    for coordinate in coordinate_input {
        let mut coord = Coordinate::from(coordinate);
        let node = map.reverse_geocode_node(coord);
        coord.node = Some(node);
        coordinates.push(coord);
    }
    println!(" ✓ duration: {}s\n", now.elapsed().unwrap().as_secs());

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
    println!(" ✓ duration: {}s\n", now.elapsed().unwrap().as_secs());

    let mut distances: Vec<Vec<Option<f64>>> = Vec::new();
    let mut durations: Vec<Vec<Option<f64>>> = Vec::new();

    println!("Generating matrix");
    &coordinates.iter().for_each(|origin_coordinate| {
        let start_node = nodes_on_graph.get(&origin_coordinate.node.unwrap().id).unwrap().clone();

        let (distance, duration) = find_distances(start_node, (*coordinates).to_owned(), nodes_on_graph.clone(), graph.clone());

        distances.push(distance);
        durations.push(duration);
    });

    println!("Distances: {:?}\n", distances);
    println!("Durations: {:?}\n", durations);
    println!(" ✓ Total duration: {}s\n", now.elapsed().unwrap().as_secs());
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
                    current_node.coord.lon,
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

fn find_distances(
    origin_node: NodeIndex,
    coordinates: Vec<Coordinate>,
    nodes_on_graph: BTreeMap<i64, NodeIndex>,
    graph: Graph<(), f64, Directed>
) -> (Vec<Option<f64>>, Vec<Option<f64>>){
    let mut distance: Vec<Option<f64>> = Vec::new();
    let mut duration: Vec<Option<f64>> = Vec::new();

    &coordinates.iter().for_each(|destination_coordinate| {
        let end_node = nodes_on_graph.get(&destination_coordinate.node.unwrap().id).unwrap().clone();
        let res = algorithms::astar_multiple_goals(&graph, origin_node, |finish| finish == end_node, |e| *e.weight(), |_| 0.);

        match res {
            Some(value) => {
                let (cost, _path) = value;
                distance.push(Option::from(cost));
                duration.push(Option::from(f64::trunc(cost / 25.00)));
            }
            None => {
                distance.push(None);
                duration.push(None);
            }
        }
    });
    return (distance, duration)
}