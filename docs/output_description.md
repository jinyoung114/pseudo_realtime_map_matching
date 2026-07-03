# Output Description

## log.csv
각 GPS 인덱스 처리 상태 기록
- warming: 초기 버퍼 적재 단계
- skip: 전처리 단계에서 제거된 점
- ok: 정상 처리된 점

## committed_nodes.csv
최종 확정된 노드 시퀀스 저장

## route.geojson
최종 맵매칭 경로를 GeoJSON 형식으로 저장

## map.html
Folium 지도 시각화 결과
- 검은선: raw GPS
- 파란선: matched route
- 배경선: road network

## summary.json
실행 요약 정보 저장
- raw_points
- accepted_points
- skipped_points
- committed_nodes
- route_coords
