# Algorithm Overview

## 전체 흐름
1. GPS 데이터를 1점씩 순차 입력
2. 온라인 전처리 수행
3. 최근 window 구간에 대해 후보 노드 탐색
4. adaptive transition을 포함한 Viterbi 수행
5. 앞부분 일부만 commit
6. 반복
7. 최종 경로 시각화 및 저장

## 핵심 개선 포인트
- heading-aware candidate re-ranking
- curve / complex intersection에서 후보 반경 확대
- adaptive transition constraint 완화
- continuity(stickiness) 보정
- commit_size=1 기반 보수적 확정
