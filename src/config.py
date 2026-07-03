from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent.parent
INPUT_DIR = BASE_DIR / "data" / "input"
OUTPUT_DIR = BASE_DIR / "data" / "output"

NODES_CSV = INPUT_DIR / "mm_nodes_dense_27_33.csv"
EDGES_CSV = INPUT_DIR / "mm_edges_dense_27_33.csv"
GPS_CSV = INPUT_DIR / "충남70자1265_0430_filtered.csv"

OUT_LOG = OUTPUT_DIR / "realtime_dense_27_33_1265_improved_log.csv"
OUT_COMM = OUTPUT_DIR / "realtime_dense_27_33_1265_improved_committed_nodes.csv"
OUT_GEO = OUTPUT_DIR / "realtime_dense_27_33_1265_improved_route.geojson"
OUT_HTML = OUTPUT_DIR / "realtime_dense_27_33_1265_improved_map.html"
OUT_ZIP = OUTPUT_DIR / "realtime_dense_27_33_1265_improved_bundle.zip"
OUT_SUMM = OUTPUT_DIR / "realtime_dense_27_33_1265_improved_summary.json"

WINDOW_SIZE = 24
COMMIT_SIZE = 1

SPEED_THRESH = 45.0
MAD_WINDOW = 9
MAD_MULT = 8.0
MIN_MAD = 1.0
HEADING_DIST_THRESH = 2.0
HEADING_JUMP_THRESH = 145.0
