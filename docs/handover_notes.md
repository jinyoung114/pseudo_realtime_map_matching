# Handover Notes

## 이 코드의 목적
저장된 버스 GPS 데이터를 의사 실시간으로 재생하면서
도로 네트워크 위에 맞추는 맵매칭 실험 코드이다.

## 실험 흐름
1. GPS 1점 입력
2. 온라인 전처리
3. 최근 window_size 구간으로 후보 탐색
4. Viterbi 경로 추정
5. 앞부분 commit
6. 반복
7. 최종 경로 시각화 및 저장

## 주의할 점
- 입력 CSV 컬럼명이 다르면 로딩 부분 수정 필요
- 도로망 연결성이 부족하면 shortest path 실패 가능
- commit_size가 크면 잘못된 경로가 빨리 확정될 수 있음
- 교차로/곡선 구간은 heading 및 transition 제한에 민감함

## 추천 디버깅 순서
1. GPS 전처리 후 점 수 확인
2. 후보 노드 수 확인
3. shortest path 연결 여부 확인
4. transition 확률이 너무 많이 막히는지 확인
5. committed path가 과하게 튀는지 확인
