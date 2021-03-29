use docopt::Docopt;
use crate::matrix::Matrix;

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

    let coordinate_input: Vec<(f64, f64)> = vec![(40.424725, -3.690438), (40.421096, -3.688578)];
    println!("Generating a matrix for {} locations", coordinate_input.len());

    let file_name: String = String::from(args.get_str("<source.osm.pbf>"));
    println!("Reading  {}", file_name);
    let matrix_result = Matrix::build(file_name, coordinate_input);
    println!("Distances: {:?}", matrix_result.distances);
    println!("Durations: {:?}\n", matrix_result.durations);
}
