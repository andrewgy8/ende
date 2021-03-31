use docopt::Docopt;
use crate::matrix::Matrix;
use crate::maps::Map;
use std::fs;

mod algorithms;
mod maps;
mod matrix;

const AVG_VEHICLE_SPEED: f64 = 25.00;

fn main() {
    const USAGE: & str = "Ende Matrix Generator

    Usage:
    ende [options] <source.osm.pbf> <locations.json>
    ende -h | --help

    Options:
    -h --help      Show this screen";

    let args = Docopt::new(USAGE).unwrap().parse().unwrap_or_else(|e| e.exit());

    let locations_file = fs::File::open(args.get_str("<locations.json>")).expect("Locations json not found.");
    let coordinate_input: Vec<(f64, f64)> = serde_json::from_reader(locations_file).expect("Locations json is invalid.");

    println!("Generating a matrix for {} locations", coordinate_input.len());

    let file_name: String = String::from(args.get_str("<source.osm.pbf>"));
    println!("Reading  {}", file_name);
    let map = Map::from(&file_name);
    let matrix_result = Matrix::build(map, coordinate_input);
    println!("Distances: {:?}", matrix_result.distances);
    println!("Durations: {:?}\n", matrix_result.durations);
}
