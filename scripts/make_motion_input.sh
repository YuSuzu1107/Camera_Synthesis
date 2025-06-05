python3 scripts/my_utils/bvh2json.py \
--bvh_dir input \
--output_dir input 

python3 scripts/my_utils/joint_extraction.py \
--json_path input/raw.json \
--output_path input/raw.json \

cp "input/raw.json" "input/motion"

python3 scripts/my_utils/hip.py \
--json_path input/raw.json \
--output_dir input/motion \

python3 scripts/my_utils/standardization.py \
--json_dir input \
--output_dir input/motion \

./scripts/json2msgpack input/motion input/motion/