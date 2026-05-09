# Routing Report

## Summary
- total_pipes: 10
- success_count: 10
- failed_count: 0
- total_length: 14232.690
- total_conflicts: 0
- unresolved_conflicts: False
- cbs_iterations: 5
- cbs_attempts: 28
- cbs_resolved_conflicts: 6
- cbs_remaining_conflicts: 0
- input_invalid: False
- invalid_pipe_ids: []
- invalid_reason: 

## CBS

- iter=1: begin conflicts=6 score=(0, 6, 0, 6, 101, 14545.604175783537)
- iter=1: conflict#1 pair=(pipe_9,pipe_8) d=0.0 req=24.0
- iter=1: try reroute pipe_8
- iter=1: reroute pipe_8 candidate score=(0, 4, 0, 5, 101, 14341.774433944242)
- iter=1: try reroute pipe_9
- iter=1: reroute pipe_9 candidate score=(0, 11, 0, 8, 103, 14801.092795503451)
- iter=1: conflict#2 pair=(pipe_9,pipe_7) d=6.0 req=24.0
- iter=1: try reroute pipe_9
- iter=1: reroute pipe_9 candidate score=(0, 5, 0, 6, 102, 14751.456616350684)
- iter=1: try reroute pipe_7
- iter=1: reroute pipe_7 candidate score=(0, 8, 0, 7, 102, 14804.274961220395)
- iter=1: conflict#3 pair=(pipe_8,pipe_7) d=6.324555320336759 req=24.0
- iter=1: try reroute pipe_8
- iter=1: reroute pipe_8 candidate score=(0, 6, 0, 6, 101, 14546.274652264035)
- iter=1: try reroute pipe_7
- iter=1: reroute pipe_7 candidate score=(0, 8, 0, 7, 102, 14804.274961220395)
- iter=1: conflict#4 pair=(pipe_6,pipe_8) d=16.97056274847714 req=24.0
- iter=1: try reroute pipe_8
- iter=1: reroute pipe_8 candidate score=(0, 4, 0, 5, 101, 14341.774433944242)
- iter=1: try reroute pipe_6
- iter=1: reroute pipe_6 failed
- iter=1: accepted pipe_8, conflicts=4
- iter=2: begin conflicts=4 score=(0, 4, 0, 5, 101, 14341.774433944242)
- iter=2: conflict#1 pair=(pipe_9,pipe_7) d=6.0 req=24.0
- iter=2: try reroute pipe_9
- iter=2: reroute pipe_9 candidate score=(0, 4, 0, 5, 102, 14365.357574970089)
- iter=2: try reroute pipe_7
- iter=2: reroute pipe_7 candidate score=(0, 6, 0, 6, 102, 14582.710236092495)
- iter=2: conflict#2 pair=(pipe_9,pipe_8) d=12.649110640673518 req=24.0
- iter=2: try reroute pipe_9
- iter=2: reroute pipe_9 candidate score=(0, 4, 0, 5, 101, 14331.797628941657)
- iter=2: try reroute pipe_8
- iter=2: reroute pipe_8 candidate score=(0, 6, 0, 6, 101, 14546.274652264035)
- iter=2: conflict#3 pair=(pipe_2,pipe_5) d=17.88854381999832 req=25.0
- iter=2: try reroute pipe_2
- iter=2: reroute pipe_2 candidate score=(0, 4, 0, 6, 100, 14370.994910769941)
- iter=2: try reroute pipe_5
- iter=2: reroute pipe_5 candidate score=(0, 4, 0, 6, 74, 14249.091995697083)
- iter=2: conflict#4 pair=(pipe_2,pipe_5) d=19.595917942265423 req=25.0
- iter=2: try reroute pipe_2
- iter=2: reroute pipe_2 candidate score=(0, 4, 0, 6, 100, 14370.994910769941)
- iter=2: try reroute pipe_5
- iter=2: reroute pipe_5 candidate score=(0, 3, 0, 5, 75, 14338.873412536805)
- iter=2: accepted pipe_5, conflicts=3
- iter=3: begin conflicts=3 score=(0, 3, 0, 5, 75, 14338.873412536805)
- iter=3: conflict#1 pair=(pipe_9,pipe_7) d=6.0 req=24.0
- iter=3: try reroute pipe_9
- iter=3: reroute pipe_9 candidate score=(0, 3, 0, 5, 76, 14362.456553562652)
- iter=3: try reroute pipe_7
- iter=3: reroute pipe_7 candidate score=(0, 5, 0, 6, 76, 14579.809214685058)
- iter=3: conflict#2 pair=(pipe_9,pipe_8) d=12.649110640673518 req=24.0
- iter=3: try reroute pipe_9
- iter=3: reroute pipe_9 candidate score=(0, 3, 0, 5, 75, 14328.89660753422)
- iter=3: try reroute pipe_8
- iter=3: reroute pipe_8 candidate score=(0, 4, 0, 6, 75, 14454.686340008097)
- iter=3: conflict#3 pair=(pipe_4,pipe_5) d=12.806248474865697 req=24.0
- iter=3: try reroute pipe_5
- iter=3: reroute pipe_5 candidate score=(0, 2, 0, 3, 74, 14093.477400664871)
- iter=3: try reroute pipe_4
- iter=3: reroute pipe_4 candidate score=(0, 3, 0, 5, 75, 14489.470010909896)
- iter=3: accepted pipe_5, conflicts=2
- iter=4: begin conflicts=2 score=(0, 2, 0, 3, 74, 14093.477400664871)
- iter=4: conflict#1 pair=(pipe_9,pipe_7) d=6.0 req=24.0
- iter=4: try reroute pipe_9
- iter=4: reroute pipe_9 candidate score=(0, 1, 0, 2, 75, 14244.119851578833)
- iter=4: try reroute pipe_7
- iter=4: reroute pipe_7 candidate score=(0, 2, 0, 4, 76, 14297.888461224891)
- iter=4: conflict#2 pair=(pipe_9,pipe_8) d=12.649110640673518 req=24.0
- iter=4: try reroute pipe_9
- iter=4: reroute pipe_9 candidate score=(0, 1, 0, 2, 75, 14244.119851578833)
- iter=4: try reroute pipe_8
- iter=4: reroute pipe_8 candidate score=(0, 2, 0, 4, 75, 14248.05019105202)
- iter=4: accepted pipe_9, conflicts=1
- iter=5: begin conflicts=1 score=(0, 1, 0, 2, 75, 14244.119851578833)
- iter=5: conflict#1 pair=(pipe_10,pipe_9) d=12.165525060596439 req=24.0
- iter=5: try reroute pipe_9
- iter=5: reroute pipe_9 candidate score=(0, 1, 0, 2, 75, 14244.119851578833)
- iter=5: try reroute pipe_10
- iter=5: reroute pipe_10 candidate score=(0, 0, 0, 0, 74, 14232.689627789872)
- iter=5: accepted pipe_10, conflicts=0

### pipe_1
- success: True
- length: 1362.432
- conflict_count: 0
- bend_rule_violation_count: 0
- input_invalid: False
- degraded_result: False
- error: None

### pipe_2
- success: True
- length: 1548.104
- conflict_count: 0
- bend_rule_violation_count: 0
- input_invalid: False
- degraded_result: False
- error: None

### pipe_3
- success: True
- length: 1360.954
- conflict_count: 0
- bend_rule_violation_count: 0
- input_invalid: False
- degraded_result: False
- error: None

### pipe_4
- success: True
- length: 1421.880
- conflict_count: 0
- bend_rule_violation_count: 0
- input_invalid: False
- degraded_result: False
- error: None

### pipe_10
- success: True
- length: 1387.606
- conflict_count: 0
- bend_rule_violation_count: 0
- input_invalid: False
- degraded_result: False
- error: None

### pipe_5
- success: True
- length: 1298.084
- conflict_count: 0
- bend_rule_violation_count: 0
- input_invalid: False
- degraded_result: False
- error: None

### pipe_9
- success: True
- length: 1552.308
- conflict_count: 0
- bend_rule_violation_count: 0
- input_invalid: False
- degraded_result: False
- error: None

### pipe_6
- success: True
- length: 1695.527
- conflict_count: 0
- bend_rule_violation_count: 0
- input_invalid: False
- degraded_result: False
- error: None

### pipe_8
- success: True
- length: 1346.302
- conflict_count: 0
- bend_rule_violation_count: 0
- input_invalid: False
- degraded_result: False
- error: None

### pipe_7
- success: True
- length: 1259.493
- conflict_count: 0
- bend_rule_violation_count: 0
- input_invalid: False
- degraded_result: False
- error: None
