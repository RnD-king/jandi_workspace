# Perception tuning guide

현재 제어·관측·rqt·단독 시험 안내는 [UNIFIED_MISSION_TUNING.md](UNIFIED_MISSION_TUNING.md)를 따른다.

일반 동작은 PREVIEW → COMMIT당 하나씩 실행한다. DOWN 검색·후진·라인 회전 탐색·허들 접촉은 단계 단위로 승인하며, 각 모션의 ACK/DONE은 유지한다. 단계가 끝나면 다음 몸동작은 별도 COMMIT이 필요하다.

`goal_tuning_node`는 FORWARD 접근부터 슛·후진·라인 복귀까지 한 시험을 수행하고 WAIT_START로 돌아간다. 회차 파라미터는 WAIT_START, 일반 파라미터는 WAIT_PREVIEW/WAIT_COMMIT에서 변경한다.
