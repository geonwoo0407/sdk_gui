# Motion candidate artifacts

`robot_motions_candidates.json` is a review and test artifact. It is not the
runtime motion catalog, and no motion alias or launch file points to it.

Candidate status:

- `공잡기`: `PICKUP_NOW` candidate; not validated on the physical robot.
- `공잡기 리그랩까지`: alternate `PICKUP_NOW` candidate; end-pose continuity
  has not been validated.
- `좌회전1`: left-turn candidate; physical direction and turn amount have not
  been validated.
- `우회전`: right-turn candidate; physical direction and turn amount have not
  been validated.

Generate the artifact with `tools/export_gui_motion_candidates.py`, passing the
GUI state, official catalog, and a distinct output path. Never overwrite the
official catalog.
