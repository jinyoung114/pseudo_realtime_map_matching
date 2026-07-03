# Pseudo Realtime Map Matching

## 1. 프로젝트 개요
이 프로젝트는 저장된 GPS 데이터를 1점씩 순차적으로 입력받는 방식으로 재생하여,
의사 실시간(pseudo real-time) 환경에서 맵매칭을 수행하는 코드입니다.

기존 GPS trajectory를 도로 네트워크 위에 맞추기 위해
heading-aware candidate re-ranking, adaptive transition constraint,
sliding window 기반 Viterbi 경로 추정을 사용합니다.

원본 단일 파일 코드를 인수인계하기 쉽게 기능별 모듈 구조로 분리했습니다.
기존 백업 파일은 `scripts/original_monolithic_backup.py`에 포함되어 있습니다.

---

## 2. 프로젝트 구조

```text
pseudo_realtime_map_matching/
├─ README.md
├─ requirements.txt
├─ .gitignore
├─ data/
│  ├─ input/
│  └─ output/
├─ src/
│  ├─ __init__.py
│  ├─ config.py
│  ├─ utils.py
│  ├─ preprocessing.py
│  ├─ road_network.py
│  ├─ transition.py
│  ├─ viterbi.py
│  ├─ visualization.py
│  └─ pipeline.py
├─ scripts/
│  ├─ run_pseudo_realtime.py
│  └─ original_monolithic_backup.py
└─ docs/
   ├─ algorithm_overview.md
   ├─ handover_notes.md
   └─ output_description.md
```

---

## 3. 입력 데이터

### (1) Nodes CSV
예상 컬럼:
- `Node_ID`
- `Latitude`
- `Longitude`

또는 내부에서 아래 이름도 자동 변환 가능:
- `id` -> `Node_ID`
- `lat` -> `Latitude`
- `lon` -> `Longitude`

### (2) Edges CSV
예상 컬럼:
- `from`
- `to`
- `geometry` 또는 `geometry_wkt`
- `length` 또는 `length_m`

### (3) GPS CSV
예상 컬럼:
- `PosX`
- `PosY`
- `ParsedTime` 또는 `Timestamp`
- `Heading` (선택)

주의:
- `PosX`, `PosY`는 1e6 스케일로 들어온다고 가정하고 내부에서 `/1e6` 처리합니다.
- 위도/경도 범위가 한국 영역(위도 32~39, 경도 124~132)인지 확인합니다.

---

## 4. 실행 환경

- Python 3.10 이상 권장
- VSCode + Python Extension 사용 권장

### 설치 패키지
```bash
pip install -r requirements.txt
```

---

## 5. 실행 방법

### 1) VSCode에서 폴더 열기
프로젝트 루트 폴더 `pseudo_realtime_map_matching/`를 VSCode에서 엽니다.

### 2) 가상환경 생성(권장)
Windows:
```bash
python -m venv venv
venv\Scripts\activate
```

Mac/Linux:
```bash
python -m venv venv
source venv/bin/activate
```

### 3) 라이브러리 설치
```bash
pip install -r requirements.txt
```

### 4) 입력 파일 준비
다음 파일들을 `data/input/` 폴더에 넣습니다.

- `mm_nodes_dense_27_33.csv`
- `mm_edges_dense_27_33.csv`
- `충남70자1265_0430_filtered.csv`

### 5) 파일명 확인
기본 파일명은 `src/config.py`에 설정되어 있습니다.
파일명이 다르면 `config.py`에서 수정합니다.

### 6) 실행
```bash
python scripts/run_pseudo_realtime.py
```

---

## 6. 출력 결과

실행 후 `data/output/` 폴더에 아래 파일이 생성됩니다.

- `*_log.csv`
  - 각 GPS 인덱스별 처리 상태 기록
  - `warming / skip / ok` 상태 확인 가능

- `*_committed_nodes.csv`
  - 최종 확정된 노드 시퀀스

- `*_route.geojson`
  - 최종 경로를 GIS 형식으로 저장한 파일

- `*_map.html`
  - folium 기반 시각화 결과
  - 검은선: 원본 GPS
  - 파란선: 실시간 확정 맵매칭 경로
  - 배경 얇은 선: 도로 네트워크

- `*_summary.json`
  - 실행 요약 정보
  - raw_points, accepted_points, skipped_points 등 확인 가능

- `*_bundle.zip`
  - 주요 결과 파일 묶음

---

## 7. 핵심 알고리즘 설명

### 1) 전처리
GPS 점을 순차적으로 읽으면서 다음을 수행합니다.
- 비정상적으로 빠른 속도 제거
- 정지 상태 근처에서 급격한 heading 변화 제거
- MAD 기반 속도 이상치 제거

### 2) 후보 노드 탐색
각 GPS 점 주변에서 BallTree를 사용해 후보 노드를 찾습니다.
- 일반 구간: 작은 반경
- 저속 구간: 반경 확대
- 커브/복잡 구간: 반경 더 확대

또한 GPS heading과 후보 노드 주변 도로 방향이 비슷한 후보를 우선시합니다.

### 3) Transition 확률
두 후보 노드 사이 전이를 평가할 때 다음을 반영합니다.
- 실제 도로 최단거리(rd)와 GPS 직선거리(gd)의 차이
- 시간 대비 이동 가능 속도
- heading 일치 여부
- 복잡 교차로/곡선 구간에서는 제약 완화

### 4) Viterbi
sliding window 안의 후보 시퀀스 중
가장 가능성이 높은 경로를 동적으로 선택합니다.

### 5) Commit
window 전체 결과를 한 번에 확정하지 않고,
앞부분 일부만 확정(commit)하여 실시간성을 흉내냅니다.

---

## 8. 주요 파라미터

`src/config.py`에서 수정 가능합니다.

- `WINDOW_SIZE`
- `COMMIT_SIZE`
- `SPEED_THRESH`
- `MAD_WINDOW`
- `MAD_MULT`
- 후보 반경 관련 설정
- heading penalty 관련 threshold
- transition 제약값

---

## 9. 코드 수정 시 먼저 보면 좋은 파일

1. `src/config.py`
2. `src/pipeline.py`
3. `src/road_network.py`
4. `src/viterbi.py`
5. `src/transition.py`

---

## 10. 인수인계 메모

- 입력 파일 컬럼명이 다르면 `load_gps_adapted()`와 `RoadNetwork.__init__()`의 컬럼 매핑 부분을 먼저 확인할 것
- 도로 네트워크 연결성이 부족하면 shortest path가 끊어져 맵매칭이 불안정할 수 있음
- 커브/교차로에서 오매칭이 나면 후보 반경, heading penalty, transition ratio 제한을 조정할 것
- pseudo real-time 실험이므로 실제 센서 스트리밍이 아니라 저장된 GPS를 순차 재생하는 방식임
