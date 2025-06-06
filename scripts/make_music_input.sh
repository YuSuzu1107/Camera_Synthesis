python3 scripts/my_utils/beat.py \
# ここに入力してください
--page  "https://songle.jp/songs/songle.jp%2Fuploads%2Fct8m4peiqnulo43nehig.mp3" \
--output_dir input/music

python3 scripts/my_utils/sabi.py \
# ここに入力してください
--page  "https://songle.jp/songs/songle.jp%2Fuploads%2Fct8m4peiqnulo43nehig.mp3" \
--output_dir input

python3 scripts/my_utils/music.py \
--audio input/music.wav \
--motion input/raw.json \
--output input/music/music.json

python3 scripts/my_utils/frameinterval.py \
--beat input/music/beat.json \
--motion input/raw.json \
--output input/frame.json

python3 scripts/my_utils/sabi+frameinterval.py \
--chorus input/sabi.json \
--frame input/frame.json \
--output input/music/sabi_frame.json

./scripts/json2msgpack input/music input/music/
