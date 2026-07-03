
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Pseudo Realtime Map Matching (Improved for complex intersections / curves)

개선 포인트
1) heading-aware candidate re-ranking
   - GPS heading과 후보 노드 주변 도로 방향이 비슷한 후보를 우선
2) complex / curve zone에서 후보 반경 확대
3) transition 제약 adaptive 완화
   - 커브/복잡구간에서는 rd/gd, heading penalty를 완화
4) continuity(stickiness) 보정
   - 직전 확정 노드와 너무 멀리 튀는 후보를 약하게 감점
5) commit_size=1 기본
   - 잘못 확정되는 양을 줄여 복구성 향상

입력 파일은 아래 기본값으로 맞춰져 있음:
- /mnt/data/mm_nodes_dense_27_33.csv
- /mnt/data/mm_edges_dense_27_33.csv
- /mnt/data/충남70자1265_0430_filtered.csv
"""

import os
import json
import math
import zipfile
from collections import deque, OrderedDict, defaultdict
from dataclasses import dataclass
from typing import Optional, List, Dict

import numpy as np
import pandas as pd
import networkx as nx
import folium
from geopy.distance import geodesic
from shapely import wkt
from shapely.geometry import LineString, mapping
from sklearn.neighbors import BallTree


# =========================
# 경로 설정
# =========================
NODES_CSV = "/mnt/data/mm_nodes_dense_27_33.csv"
EDGES_CSV = "/mnt/data/mm_edges_dense_27_33.csv"
GPS_CSV   = "/mnt/data/충남70자1265_0430_filtered.csv"

OUT_LOG   = "/mnt/data/realtime_dense_27_33_1265_improved_log.csv"
OUT_COMM  = "/mnt/data/realtime_dense_27_33_1265_improved_committed_nodes.csv"
OUT_GEO   = "/mnt/data/realtime_dense_27_33_1265_improved_route.geojson"
OUT_HTML  = "/mnt/data/realtime_dense_27_33_1265_improved_map.html"
OUT_ZIP   = "/mnt/data/realtime_dense_27_33_1265_improved_bundle.zip"
OUT_SUMM  = "/mnt/data/realtime_dense_27_33_1265_improved_summary.json"


# =========================
# 유틸
# =========================
def bearing_deg(p1, p2):
    lat1, lon1 = np.radians(p1[0]), np.radians(p1[1])
    lat2, lon2 = np.radians(p2[0]), np.radians(p2[1])
    dlon = lon2 - lon1
    y = np.sin(dlon) * np.cos(lat2)
    x = np.cos(lat1) * np.sin(lat2) - np.sin(lat1) * np.cos(lat2) * np.cos(dlon)
    brng = np.degrees(np.arctan2(y, x))
    return (brng + 360) % 360


def angle_diff(a, b):
    return (a - b + 180) % 360 - 180


def emit_prob(distance_m, sigma=35.0):
    return math.exp(-distance_m / sigma)


def is_curve(prev, curr, nxt, thr=28):
    a1 = bearing_deg(prev, curr)
    a2 = bearing_deg(curr, nxt)
    diff = (a2 - a1 + 180) % 360 - 180
    return abs(diff) > thr


class LRUCache:
    def __init__(self, cap=100000):
        self.cap = cap
        self.od = OrderedDict()

    def get(self, k):
        if k not in self.od:
            return None
        self.od.move_to_end(k)
        return self.od[k]

    def set(self, k, v):
        self.od[k] = v
        self.od.move_to_end(k)
        if len(self.od) > self.cap:
            self.od.popitem(last=False)


@dataclass
class GPSPoint:
    lat: float
    lon: float
    t: Optional[pd.Timestamp]
    raw_heading: float = np.nan


@dataclass
class CleanPoint:
    lat: float
    lon: float
    t: Optional[pd.Timestamp]
    dt: float
    dist: float
    speed: float
    heading: float


# =========================
# 완화 전처리
# =========================
class OnlinePreprocessor:
    def __init__(
        self,
        speed_thresh=45.0,
        mad_window=9,
        mad_mult=8.0,
        min_mad=1.0,
        heading_dist_thresh=2.0,
        heading_jump_thresh=145.0,
    ):
        self.speed_thresh = speed_thresh
        self.mad_window = mad_window
        self.mad_mult = mad_mult
        self.min_mad = min_mad
        self.heading_dist_thresh = heading_dist_thresh
        self.heading_jump_thresh = heading_jump_thresh
        self.buf = deque(maxlen=300)
        self.prev: Optional[GPSPoint] = None

    def update(self, p: GPSPoint):
        if self.prev is None:
            cp = CleanPoint(
                lat=p.lat, lon=p.lon, t=p.t, dt=1.0, dist=0.0, speed=0.0, heading=np.nan
            )
            self.prev = p
            self.buf.append(cp)
            return cp

        dt = 1.0
        if p.t is not None and self.prev.t is not None:
            secs = (p.t - self.prev.t).total_seconds()
            dt = secs if (secs and secs > 0) else 1.0

        dist = geodesic((self.prev.lat, self.prev.lon), (p.lat, p.lon)).meters
        speed = dist / dt
        heading = bearing_deg((self.prev.lat, self.prev.lon), (p.lat, p.lon)) if dist > 0 else np.nan

        self.prev = p

        # 1) speed spike
        if speed > self.speed_thresh:
            return None

        # 2) near-stationary heading jump
        if len(self.buf) >= 1 and dist < self.heading_dist_thresh:
            prev_h = self.buf[-1].heading
            if not np.isnan(prev_h) and not np.isnan(heading):
                if abs(angle_diff(heading, prev_h)) > self.heading_jump_thresh:
                    return None

        # 3) MAD 완화
        recent = list(self.buf)[-self.mad_window + 1:]
        speeds = [c.speed for c in recent] + [speed]
        med = float(np.median(speeds))
        mad = float(np.median(np.abs(np.array(speeds) - med)))
        mad = max(mad, self.min_mad)

        if abs(speed - med) > self.mad_mult * mad:
            return None

        cp = CleanPoint(
            lat=p.lat, lon=p.lon, t=p.t, dt=dt, dist=dist, speed=speed, heading=heading
        )
        self.buf.append(cp)
        return cp


# =========================
# 도로 네트워크
# =========================
class RoadNetwork:
    def __init__(self, nodes_csv, edges_csv):
        nd = pd.read_csv(nodes_csv, encoding="utf-8-sig")
        self.nodes = nd.rename(
            columns={"id": "Node_ID", "lat": "Latitude", "lon": "Longitude"}
        )[["Node_ID", "Latitude", "Longitude"]].copy()

        self.nodes["Node_ID"] = self.nodes["Node_ID"].astype(int)
        self.nodes["Latitude"] = pd.to_numeric(self.nodes["Latitude"], errors="coerce")
        self.nodes["Longitude"] = pd.to_numeric(self.nodes["Longitude"], errors="coerce")
        self.nodes = self.nodes.dropna().reset_index(drop=True)

        self.edges = pd.read_csv(edges_csv, encoding="utf-8-sig")

        self.G = nx.DiGraph()
        for _, r in self.nodes.iterrows():
            self.G.add_node(
                int(r["Node_ID"]), lat=float(r["Latitude"]), lon=float(r["Longitude"])
            )

        self.edge_geom = {}
        self.node_headings = defaultdict(list)

        geom_col = "geometry" if "geometry" in self.edges.columns else "geometry_wkt"
        len_col = "length" if "length" in self.edges.columns else ("length_m" if "length_m" in self.edges.columns else None)

        for _, r in self.edges.iterrows():
            u = int(r["from"])
            v = int(r["to"])
            w = float(r[len_col]) if len_col and pd.notna(r[len_col]) else None

            geom = None
            if isinstance(r.get(geom_col), str):
                try:
                    geom = wkt.loads(str(r[geom_col]))
                    self.edge_geom[(u, v)] = geom
                except Exception:
                    geom = None

            if w is None:
                if geom is not None:
                    coords = list(geom.coords)
                    total = 0.0
                    for i in range(len(coords) - 1):
                        x1, y1 = coords[i]
                        x2, y2 = coords[i + 1]
                        total += geodesic((y1, x1), (y2, x2)).meters
                    w = total
                elif u in self.G.nodes and v in self.G.nodes:
                    w = geodesic(
                        (self.G.nodes[u]["lat"], self.G.nodes[u]["lon"]),
                        (self.G.nodes[v]["lat"], self.G.nodes[v]["lon"]),
                    ).meters
                else:
                    w = 1.0

            if u in self.G.nodes and v in self.G.nodes:
                self.G.add_edge(u, v, weight=w)
                self.G.add_edge(v, u, weight=w)

                # 후보 정렬용: 노드 주변 대표 방향 저장
                h_uv = bearing_deg(
                    (self.G.nodes[u]["lat"], self.G.nodes[u]["lon"]),
                    (self.G.nodes[v]["lat"], self.G.nodes[v]["lon"]),
                )
                h_vu = bearing_deg(
                    (self.G.nodes[v]["lat"], self.G.nodes[v]["lon"]),
                    (self.G.nodes[u]["lat"], self.G.nodes[u]["lon"]),
                )
                self.node_headings[u].append(h_uv)
                self.node_headings[v].append(h_vu)

        self.tree = BallTree(
            np.radians(self.nodes[["Latitude", "Longitude"]].values),
            metric="haversine"
        )
        self.sp_cache = LRUCache()
        self.path_cache = LRUCache()

    def sp_len(self, u, v):
        key = (u, v) if u <= v else (v, u)
        c = self.sp_cache.get(key)
        if c is not None:
            return c

        if not nx.has_path(self.G, u, v):
            self.sp_cache.set(key, None)
            return None

        try:
            rd = nx.shortest_path_length(self.G, u, v, weight="weight")
        except Exception:
            rd = None

        self.sp_cache.set(key, rd)
        return rd

    def shortest_path(self, u, v):
        key = (u, v)
        c = self.path_cache.get(key)
        if c is not None:
            return c
        try:
            sp = nx.shortest_path(self.G, u, v, weight="weight")
        except Exception:
            sp = None
        self.path_cache.set(key, sp)
        return sp

    def edge_heading(self, u, v):
        return bearing_deg(
            (self.G.nodes[u]["lat"], self.G.nodes[u]["lon"]),
            (self.G.nodes[v]["lat"], self.G.nodes[v]["lon"])
        )

    def node_complexity(self, node_id):
        return self.G.degree(int(node_id))

    def representative_heading_penalty(self, node_id, obs_heading):
        hs = self.node_headings.get(int(node_id), [])
        if not hs or np.isnan(obs_heading):
            return 1.0
        diffs = [abs(angle_diff(h, obs_heading)) for h in hs]
        d = min(diffs)
        if d <= 25:
            return 1.15
        if d <= 45:
            return 1.0
        if d <= 90:
            return math.exp(-(d - 45) / 30.0)
        return 0.35

    def candidates(self, W, i):
        p = W[i]
        prev = W[i - 1] if i > 0 else None
        nxt = W[i + 1] if i < len(W) - 1 else None

        curved = False
        if prev is not None and nxt is not None:
            curved = is_curve((prev.lat, prev.lon), (p.lat, p.lon), (nxt.lat, nxt.lon), thr=28)

        # 복잡구간 / 저속 / 커브일수록 후보 범위 확대
        if curved:
            radius = 24
            k = 10
        elif p.speed < 5.0:
            radius = 20
            k = 8
        else:
            radius = 14
            k = 6

        q = np.radians([[p.lat, p.lon]])
        inds = self.tree.query_radius(q, r=radius / 6371000.0, return_distance=True)

        idxs = inds[0][0] if len(inds[0]) > 0 else np.array([])
        dists = inds[1][0] * 6371000.0 if len(inds[1]) > 0 else np.array([])

        if len(idxs) == 0:
            d, idx = self.tree.query(q, k=min(k, len(self.nodes)))
            idxs = idx[0]
            dists = d[0] * 6371000.0

        raw = []
        for dist, idx in zip(dists, idxs):
            row = self.nodes.iloc[int(idx)]
            nid = int(row["Node_ID"])
            head_bonus = self.representative_heading_penalty(nid, p.heading)
            complexity = self.node_complexity(nid)
            score = float(dist) / head_bonus
            # 매우 단순 노드에서 heading 불일치가 크면 살짝 감점
            if complexity <= 2 and head_bonus < 0.6:
                score *= 1.2
            raw.append({
                "Node_ID": nid,
                "Latitude": float(row["Latitude"]),
                "Longitude": float(row["Longitude"]),
                "Distance": float(dist),
                "_cand_score": score,
                "_head_bonus": head_bonus,
                "_complexity": complexity,
            })

        raw = sorted(raw, key=lambda x: x["_cand_score"])[:k]
        return raw


# =========================
# Adaptive transition
# =========================
def trans_prob_adaptive(rn, n1, n2, g1, g2, dt, obs_heading, curved=False, complex_zone=False):
    u = int(n1["Node_ID"])
    v = int(n2["Node_ID"])

    rd = rn.sp_len(u, v)
    if rd is None:
        return 1e-12

    gd = geodesic(g1, g2).meters

    ratio_limit = 4.8 if (curved or complex_zone) else 3.2
    diff_limit = 65.0 if (curved or complex_zone) else 35.0
    near_stop_rd_limit = 65.0 if (curved or complex_zone) else 40.0

    # (1) 거리 비율 제약
    if gd > 5.0:
        if (rd / max(gd, 1.0)) > ratio_limit and (rd - gd) > diff_limit:
            return 1e-12
    else:
        if rd > near_stop_rd_limit:
            return 1e-12

    # (2) 속도 일치성
    if dt and dt > 0:
        rs = rd / dt
        os_ = gd / dt
        rs_limit = 52.0 if (curved or complex_zone) else 45.0
        if rs > rs_limit:
            return 1e-12
        if os_ < 15.0:
            limit = max(28.0 if (curved or complex_zone) else 25.0, 2.8 * max(os_, 1.0))
            if rs > limit:
                return 1e-12

    # (3) 헤딩 제약
    head_term = 1.0
    try:
        rh = rn.edge_heading(u, v)
    except Exception:
        rh = np.nan

    if not np.isnan(obs_heading) and not np.isnan(rh):
        diff = abs(angle_diff(rh, obs_heading))
        hard_block = 170 if (curved or complex_zone) else 150
        if diff >= hard_block:
            return 1e-12
        elif diff >= 110:
            head_term = math.exp(-((diff - 110) / 18.0)) * 0.35
        elif diff >= 70:
            head_term = math.exp(-((diff - 70) / 20.0)) * 0.7
        else:
            head_term = math.exp(-(diff / 40.0))

    # (4) 거리 일치 감쇠
    dist_scale = 60.0 if (curved or complex_zone) else 40.0
    dist_term = math.exp(-abs(rd - gd) / dist_scale)

    # (5) 복잡 노드 보상: 교차로에서는 약간 완화
    degree_bonus = 1.08 if max(rn.node_complexity(u), rn.node_complexity(v)) >= 4 else 1.0

    return max(dist_term * head_term * degree_bonus, 1e-12)


def continuity_factor(rn, committed_last, cand_node_id):
    if committed_last is None:
        return 1.0
    rd = rn.sp_len(int(committed_last), int(cand_node_id))
    if rd is None:
        return 0.7
    if rd <= 20:
        return 1.08
    if rd <= 60:
        return 1.0
    if rd <= 120:
        return 0.9
    return 0.78


# =========================
# Viterbi on window
# =========================
def viterbi_window(rn, W, committed_last=None):
    cand = [rn.candidates(W, i) for i in range(len(W))]
    vit = []

    first = cand[0]
    complex0 = any(c["_complexity"] >= 4 for c in first)
    curved0 = False
    if len(W) >= 3:
        curved0 = is_curve((W[0].lat, W[0].lon), (W[1].lat, W[1].lon), (W[2].lat, W[2].lon), thr=28)

    for c in first:
        base = emit_prob(c["Distance"]) * c["_head_bonus"]
        base *= continuity_factor(rn, committed_last, c["Node_ID"])
        if complex0:
            base *= 1.03
        c["delta"] = base
        c["psi"] = None
    vit.append(first)

    for t in range(1, len(W)):
        prev = vit[t - 1]
        curr = cand[t]

        local_curved = False
        if 0 < t < len(W) - 1:
            local_curved = is_curve(
                (W[t - 1].lat, W[t - 1].lon),
                (W[t].lat, W[t].lon),
                (W[t + 1].lat, W[t + 1].lon),
                thr=28
            )

        local_complex = any(c["_complexity"] >= 4 for c in curr)

        for c in curr:
            best = -1.0
            bestp = None

            for p in prev:
                tp = trans_prob_adaptive(
                    rn,
                    p, c,
                    (W[t - 1].lat, W[t - 1].lon),
                    (W[t].lat, W[t].lon),
                    W[t].dt,
                    W[t].heading,
                    curved=local_curved,
                    complex_zone=(local_complex or p["_complexity"] >= 4 or c["_complexity"] >= 4)
                )

                prob = p["delta"] * tp * emit_prob(c["Distance"]) * c["_head_bonus"]

                # 확정 경로와의 연속성 보정
                prob *= continuity_factor(rn, committed_last, c["Node_ID"])

                if prob > best:
                    best = prob
                    bestp = p

            c["delta"] = best
            c["psi"] = bestp

        vit.append(curr)

    last = max(vit[-1], key=lambda x: x["delta"])
    path = [last]
    for _ in range(len(W) - 1, 0, -1):
        path.insert(0, path[0]["psi"])
    return path


def build_coords_from_committed(rn, committed):
    coords = []
    for u, v in zip(committed[:-1], committed[1:]):
        sp = rn.shortest_path(int(u), int(v))
        if sp is None or len(sp) < 2:
            continue

        for a, b in zip(sp[:-1], sp[1:]):
            geom = rn.edge_geom.get((a, b)) or rn.edge_geom.get((b, a))
            if geom is None:
                seg = [
                    (rn.G.nodes[a]["lat"], rn.G.nodes[a]["lon"]),
                    (rn.G.nodes[b]["lat"], rn.G.nodes[b]["lon"]),
                ]
            else:
                seg = [(y, x) for (x, y) in list(geom.coords)]

            if coords and coords[-1] == seg[0]:
                coords.extend(seg[1:])
            else:
                coords.extend(seg)
    return coords


def load_gps_adapted(path):
    raw = pd.read_csv(path, encoding="utf-8-sig").copy()

    # PosX/PosY 1e6 스케일 처리
    raw["Lon"] = pd.to_numeric(raw["PosX"], errors="coerce") / 1e6
    raw["Lat"] = pd.to_numeric(raw["PosY"], errors="coerce") / 1e6

    ts_col = "ParsedTime" if "ParsedTime" in raw.columns else ("Timestamp" if "Timestamp" in raw.columns else None)
    if ts_col is not None:
        raw["TripData"] = pd.to_datetime(raw[ts_col], errors="coerce")
    else:
        raw["TripData"] = pd.NaT

    if "Heading" in raw.columns:
        raw["HeadingRaw"] = pd.to_numeric(raw["Heading"], errors="coerce")
    else:
        raw["HeadingRaw"] = np.nan

    raw = raw.dropna(subset=["Lat", "Lon"]).reset_index(drop=True)
    raw = raw[raw["Lat"].between(32, 39) & raw["Lon"].between(124, 132)].copy()
    raw = raw.sort_values("TripData", kind="stable").reset_index(drop=True)
    return raw


def main():
    rn = RoadNetwork(NODES_CSV, EDGES_CSV)
    raw = load_gps_adapted(GPS_CSV)

    pre = OnlinePreprocessor(
        speed_thresh=45.0,
        mad_window=9,
        mad_mult=8.0,
        min_mad=1.0,
        heading_dist_thresh=2.0,
        heading_jump_thresh=145.0,
    )

    # 빨간 박스 같은 복잡구간 대응: 더 보수적으로 확정
    window_size = 24
    commit_size = 1

    clean_buf = deque()
    committed = []
    records = []

    total = len(raw)
    ok = 0
    skipped = 0

    for i, row in raw.iterrows():
        cp = pre.update(
            GPSPoint(
                float(row["Lat"]),
                float(row["Lon"]),
                row["TripData"],
                float(row["HeadingRaw"]) if pd.notna(row["HeadingRaw"]) else np.nan,
            )
        )

        if cp is None:
            skipped += 1
            records.append({"i": i, "status": "skip"})
            continue

        ok += 1
        clean_buf.append(cp)

        if len(clean_buf) < 3:
            records.append({"i": i, "status": "warming"})
            continue

        W = list(clean_buf)[-window_size:]
        committed_last = committed[-1] if committed else None
        path = viterbi_window(rn, W, committed_last=committed_last)
        window_nodes = [int(p["Node_ID"]) for p in path]
        cur = window_nodes[-1] if window_nodes else None

        if len(W) >= window_size:
            for n in window_nodes[:commit_size]:
                if not committed or committed[-1] != n:
                    committed.append(n)

            for _ in range(commit_size):
                if clean_buf:
                    clean_buf.popleft()

        records.append({
            "i": i,
            "status": "ok",
            "current_node": cur,
            "committed_len": len(committed)
        })

    # tail flush
    if clean_buf:
        W = list(clean_buf)
        committed_last = committed[-1] if committed else None
        path = viterbi_window(rn, W, committed_last=committed_last)
        for n in [int(p["Node_ID"]) for p in path]:
            if not committed or committed[-1] != n:
                committed.append(n)

    log_df = pd.DataFrame(records)
    log_df.to_csv(OUT_LOG, index=False, encoding="utf-8-sig")

    comm_df = pd.DataFrame({"node_id": committed})
    comm_df.to_csv(OUT_COMM, index=False, encoding="utf-8-sig")

    coords = build_coords_from_committed(rn, committed)

    if len(coords) >= 2:
        line = LineString([(lon, lat) for lat, lon in coords])
        gj = {
            "type": "FeatureCollection",
            "features": [{
                "type": "Feature",
                "geometry": mapping(line),
                "properties": {"name": "realtime_committed_improved"}
            }]
        }
    else:
        gj = {"type": "FeatureCollection", "features": []}

    with open(OUT_GEO, "w", encoding="utf-8") as f:
        json.dump(gj, f, ensure_ascii=False)

    center = [
        float(np.mean([c[0] for c in coords])) if coords else float(raw["Lat"].mean()),
        float(np.mean([c[1] for c in coords])) if coords else float(raw["Lon"].mean()),
    ]

    m = folium.Map(location=center, zoom_start=14, control_scale=True)

    # 배경 도로망
    geom_col = "geometry" if "geometry" in rn.edges.columns else "geometry_wkt"
    for w in rn.edges[geom_col].dropna().astype(str):
        try:
            ln = wkt.loads(w)
            seg = [(y, x) for (x, y) in list(ln.coords)]
            folium.PolyLine(seg, weight=2, opacity=0.16).add_to(m)
        except Exception:
            pass

    # 원본 GPS
    folium.PolyLine(
        list(zip(raw["Lat"].astype(float), raw["Lon"].astype(float))),
        weight=3,
        opacity=0.55,
        color="black",
        tooltip="Raw GPS"
    ).add_to(m)

    # 실시간 확정 경로
    if coords:
        folium.PolyLine(
            coords,
            weight=6,
            opacity=0.95,
            color="blue",
            tooltip="Realtime committed improved"
        ).add_to(m)
        m.fit_bounds(coords)

    m.save(OUT_HTML)

    summary = {
        "raw_points": int(total),
        "accepted_points": int(ok),
        "skipped_points": int(skipped),
        "window_size": int(window_size),
        "commit_size": int(commit_size),
        "committed_nodes": int(len(committed)),
        "route_coords": int(len(coords)),
        "notes": [
            "heading-aware candidate re-ranking",
            "adaptive transition relaxation for curve/complex zones",
            "continuity bonus using last committed node",
            "window_size=24, commit_size=1",
        ],
    }

    with open(OUT_SUMM, "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    with zipfile.ZipFile(OUT_ZIP, "w", compression=zipfile.ZIP_DEFLATED) as z:
        for f in [OUT_LOG, OUT_COMM, OUT_GEO, OUT_HTML, OUT_SUMM]:
            z.write(f, arcname=os.path.basename(f))

    print("DONE")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
