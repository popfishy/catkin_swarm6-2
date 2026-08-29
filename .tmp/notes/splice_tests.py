#!/usr/bin/env python3
"""Replace old horizon/rolling tests (lines 843-1281) with plan-exec tests."""
from pathlib import Path

p = Path("/home/ub20tg/catkin_swarm6-2/src/ego_planner_driver/test/test_ego_state_machine.cpp")
lines = p.read_text(encoding="utf-8").splitlines(keepends=True)

# sanity: line 843 is the old RollingWindow test, 1283 is int main
assert "RollingWindowAdvancesWithActualPosition" in lines[842], lines[842]
assert "int main" in lines[1282], lines[1282]

new = ""
for f in ["new_tests_part1a.cpp", "new_tests_part1b.cpp", "new_tests_part2.cpp"]:
    new += (Path("/home/ub20tg/catkin_swarm6-2/.tmp/notes") / f).read_text(encoding="utf-8")
    new += "\n"

lines[842:1281] = [new]
p.write_text("".join(lines), encoding="utf-8")
print("spliced lines 843-1281 -> new plan-exec tests")
