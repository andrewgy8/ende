cargo build --

echo "Running small sized benchmark..."
./target/release/ende benchmarks/small/mad1.osm.pbf benchmarks/small/locations.json


echo "Running medium sized benchmark..."
./target/debug/ende benchmarks/medium/Madrid.osm.pbf benchmarks/medium/locations.json
