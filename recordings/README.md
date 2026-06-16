# Real-camera raw recordings

Local raw-video capture set for `sony_object_inside_container_v0`.

Raw `.mp4` files are intentionally ignored by git. `manifest.json` is the
source of truth for provenance, expected class, camera view, SHA-256, and
capture notes.

## Naming

```text
raw_videos/<category>/oic_<category>_<sequence:03d>__<camera>.mp4
```

Camera suffixes:

```text
sony_front   Sony A7IV, approximately 45 degree frontal view
iphone_top   iPhone, top view
```

Main labels:

```text
success
fail_near_not_inside
fail_on_rim
fail_outside
control_static
control_born_inside
control_born_inside_hand_motion
control_inside_to_inside
control_inside_to_outside
success_tag_obstruction
success_hand_obstruction
calibration
```

## Capture Notes

- Actual cube: `5 x 5 x 5 cm`.
- Actual cube tags: ID `2` on cube top, ID `3` on cube front, both 35 mm print size.
- Actual tray: nominal base `18 x 18 cm`, rim height `4 cm`.
- Tray construction: homemade cardboard, not perfectly precise or consistent; ingestion should use measured/fitted tray geometry rather than assuming a perfect square.
- Confirmed table tags: ID `0` and ID `1`.
- Confirmed tray tags: ID `6` in front of the tray, ID `7` inside the tray.
- Absent tags: ID `4` and ID `5` were not present in the capture.
- IDs `6` and `7` were printed as big-cube tags but were not used as cube tags; they were repurposed as tray markers.
- Tag sheet: `output/pdf/sony_object_inside_container_v0_apriltags_a4.pdf`.
- Tag script: `scripts/make_apriltag_print_sheet.py`.
- The existing `datasets/sony_object_inside_container_v0/calibration/sony_table_v0.calibration.json` is synthetic/obsolete for these videos. It still references marker `7` and `0.04 m` cube geometry, so real ingestion needs a new calibration using IDs `2` and `3`.

Known success visibility caveats from capture notes:

```text
oic_success_004  Sony front cube tag ~20% visible at terminal frame
oic_success_005  Sony front cube tag ~80% visible at terminal frame
oic_success_007  Sony front cube tag ~35% visible at terminal frame
oic_success_013  Sony front cube tag not visible; top cube tag visible
oic_success_015  Cube covers tray/interior tag as it lands
oic_success_016  Sony front cube tag not visible; top cube tag visible
```

## Ingestion Order

1. Author a real calibration for the captured setup.
2. Run `video_to_tracks` on the primary Sony clips first.
3. Use iPhone top clips as fallback/diagnostic evidence when Sony terminal marker visibility is weak.
4. Keep obstruction clips as `UNCERTAIN`/quality-gate probes, not ordinary successes.
