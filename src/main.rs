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
    // let file_name: String = String::from("mad1.osm.pbf");
    let file_name: String = String::from("Madrid1.osm.pbf");

    let map = Map::from(&file_name);
    println!(" ✓ duration: {}s\n", now.elapsed().unwrap().as_secs());

    let mut graph: Graph<(), f64, Directed> = Graph::new();
    let mut nodes_on_graph = BTreeMap::new();
    let mut index_on_graph = BTreeMap::new();
    // let coordinate_input: Vec<(f64, f64)> = vec![(40.424725, -3.690438), (40.421096, -3.688578)];
    let coordinate_input: Vec<(f64, f64)> = vec![(40.45406979085049, -3.7047928477640073), (40.4302915521185, -3.7248617249939104), (40.446375186971444, -3.6832047437438784), (40.44374892804033, -3.675195887568214), (40.44736010339184, -3.6759121064509297), (40.45480122634401, -3.6516687650782456), (40.431367070065924, -3.6969488283345155), (40.40918560168279, -3.6518124770351963), (40.44814075586496, -3.6986181592036305), (40.41389557263346, -3.6506295137106726), (40.41003589948459, -3.6366436700097173), (40.429209911429695, -3.725462294401394), (40.40264508314366, -3.725002028754057), (40.45206481482673, -3.714821768684163), (40.44159685703408, -3.7113811243913544), (40.397685104527305, -3.6956634556821975), (40.41267843735821, -3.690090524967307), (40.468829197311244, -3.658081078389943), (40.402109502177986, -3.7165459933006044), (40.427421868640344, -3.699920343484687), (40.44461337471709, -3.7181518893281926), (40.452403991810826, -3.6817085614116625), (40.44479930180164, -3.7304202133748543), (40.439266546400944, -3.6687708241996453), (40.41870538269331, -3.670768232415561), (40.45755827594756, -3.6463879917692705), (40.42709601001018, -3.7391483621405874), (40.452273626282825, -3.67898037364266), (40.44314552014561, -3.638980322941117), (40.44465940082432, -3.711816770933302), (40.41834903366685, -3.6897121649914064), (40.441241089723, -3.733184642874021), (40.40281249963626, -3.649489546338016), (40.45685900502627, -3.7150573796312254), (40.434728442752466, -3.712584268132944), (40.406135325053704, -3.6754371720474412), (40.43136249830612, -3.66470033993203), (40.43391853763063, -3.716343779533534), (40.39831103389811, -3.666528011557476), (40.42449034539011, -3.6486988695777995), (40.424104754592534, -3.6298171448600978), (40.46286374600103, -3.7001715756150446), (40.42653375479285, -3.6988670511931554), (40.40044951579655, -3.690679325528065), (40.400825342107154, -3.659616518210784), (40.440983719609555, -3.648919628724564), (40.40341852011385, -3.688757855192186), (40.43638913731272, -3.66880752668552), (40.41717655652202, -3.7232881943005767), (40.445550629542296, -3.732254283179336), (40.46546155538044, -3.7020761622165685), (40.429641952699974, -3.6806886306524795), (40.4431355629009, -3.713734947912662), (40.402953225910025, -3.7094071001482507), (40.44884377719322, -3.6867901677083927), (40.44377413443607, -3.631418769666054), (40.41219136693914, -3.658574390161358), (40.46101587361047, -3.662729684921255), (40.45043441447493, -3.727732875448789), (40.44952616023382, -3.67174981667876), (40.46015302406952, -3.6502393036227168), (40.444981572082234, -3.6362389161950093), (40.45225226126489, -3.677877071527354), (40.46861203293293, -3.704015652675816), (40.43261353257363, -3.7116590921526846), (40.440210041221754, -3.717653015453257), (40.421342651348276, -3.6558359738884305), (40.41424821096271, -3.695893166023976), (40.42759479073607, -3.7371502908564285), (40.39720113742787, -3.7014403581889703), (40.44143673406973, -3.6395433827055776), (40.418951996157176, -3.668096093746995), (40.436004789837995, -3.629180119723698), (40.45016774365834, -3.6942506818710856), (40.39807350135697, -3.680253351505399), (40.40666140201655, -3.6632360548990968), (40.441315424951995, -3.702820283749397), (40.41928776964927, -3.641216330536595), (40.42015654300728, -3.6716438121858106), (40.42699279700211, -3.6584334789451427), (40.434058881551486, -3.636435541886991), (40.44376200424207, -3.728007007381975), (40.43134947670697, -3.6633114587518665), (40.405465412271965, -3.640522345572245), (40.42497466765259, -3.6798292982927516), (40.46040236883411, -3.697022947507531), (40.410602580982804, -3.639370848034582), (40.465811511304764, -3.6606637284825725), (40.466988368620896, -3.653401716944593), (40.427352937209506, -3.735416288603102), (40.43762382110809, -3.653121797427444), (40.40620423187186, -3.6748212778998783), (40.450820941532086, -3.644758290156213), (40.42091248814598, -3.6494178349847646), (40.408194387826214, -3.704600839719716), (40.43922922944042, -3.686726296745252), (40.470695619308366, -3.692415480428139), (40.44178513145114, -3.6588804378125546), (40.41802177333361, -3.683336624162041), (40.40168644348842, -3.672749864350078)
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
    &coordinates.par_iter().for_each(|origin_coordinate| {
        let start_node = nodes_on_graph.get(&origin_coordinate.node.unwrap().id).unwrap().clone();

        let (distance, duration) = find_distances(start_node, (*coordinates).to_owned(), nodes_on_graph.clone(), graph.clone());

        // distances.push(distance);
        // durations.push(duration);
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
    graph: Graph<(), f64, Directed>,
) -> (Vec<Option<f64>>, Vec<Option<f64>>) {
    let mut distance: Vec<Option<f64>> = Vec::new();
    let mut duration: Vec<Option<f64>> = Vec::new();

    let res = algorithms::astar_multiple_goals(
        &graph,
        origin_node,
        |finish| coordinates.iter().map(|coordinate| nodes_on_graph.get(&coordinate.node.unwrap().id)).any(|node_index| node_index == Some(&finish)),
        |e| *e.weight(),
        |_| 0.);
    for (node_id, result) in res {
        let cost = result.unwrap();
        println!("Node id: {:?}, Cost: {:?}", node_id, cost);
    }
    // match res {
    //     Some(value) => {
    //         let (cost, _path) = value;
    //         distance.push(Option::from(cost));
    //         duration.push(Option::from(f64::trunc(cost / 25.00)));
    //     }
    //     None => {
    //         distance.push(None);
    //         duration.push(None);
    //     }
    // }
    return (distance, duration)
}