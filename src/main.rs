use docopt::Docopt;
use crate::matrix::Matrix;
use std::fs;

mod algorithms;
mod maps;
mod matrix;

const AVG_VEHICLE_SPEED: f64 = 25.00;

fn main() {
    const USAGE: &'static str = "Ende Matrix Generator

    Usage:
    ende [options] <source.osm.pbf> <locations.json>
    ende -h | --help

    Options:
    -h --help      Show this screen";

    let args = Docopt::new(USAGE).unwrap().parse().unwrap_or_else(|e| e.exit());

    let locations_file = fs::File::open(args.get_str("<locations.json>")).expect("file should open read only");
    let coordinate_input: Vec<(f64, f64)> = serde_json::from_reader(locations_file).expect("file should be proper JSON");

    println!("Generating a matrix for {} locations", coordinate_input.len());

    let file_name: String = String::from(args.get_str("<source.osm.pbf>"));
    println!("Reading  {}", file_name);
    let matrix_result = Matrix::build(file_name, coordinate_input);
    println!("Distances: {:?}", matrix_result.distances);
    println!("Durations: {:?}\n", matrix_result.durations);
}
