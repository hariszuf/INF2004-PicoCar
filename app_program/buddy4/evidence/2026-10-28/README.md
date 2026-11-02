# IMU evidence checkpoint 28 October 2026

Original capture bytes are preserved in raw/. manifest.json records SHA-256 hashes, calibration versions, complete/valid row counts and extracted event summaries. All captures here use the 69-column integration schema. File labels and reference heights follow the operator's descriptions in the development session; capture timestamps are firmware times since boot, not wall-clock dates.

## Analysis rules

- Preserve raw text; do not rewrite command messages or incomplete lines.
- For numeric-row analysis accept lines beginning with a numeric sequence, exactly 69 fields, and numeric fields except motion state (index 51) and control reason (index 68).
- Select IMU-valid rows using zero-based index 3. Settled height is index 20. Recordings were supplied as settled intervals; do not infer stored event peaks are stationary measurements.
- Reject malformed rows instead of shifting columns. Rejected_numeric_lines counts numeric-prefixed rejected lines, not every non-data fragment. Terminal text is not a sample. Duplicate module timestamps are not fresh independent updates.
- Moving charts use explicit SIDE_EVENT_END summaries. Their counts and values can be checked in the manifest and original captures.
- Left 1 is excluded: ramp at tyre edge rather than centred. It is not a proven algorithmic miss.
- Heights use operator reference 1.00 cm, with unquantified measurement uncertainty. Trials are small and uneven in count. Calibration changes and repositioning prevent claims of a controlled general accuracy comparison.
- Possible-impact counts can predate captures; zero pulses in a capture do not prove zero false detections across a session.

## Figures

![Moving peaks](charts/moving_height_comparison.png)

![Settled height](charts/stationary_height_comparison.png)

The latest after-correction moving stable peaks are 0.994 cm left and 0.989 cm right; median peaks are 1.049 cm left and 1.065 cm right. These support limited single-wheel verification, not collision/yaw validation or general course accuracy. No firmware changes or new robot tests were performed to prepare this evidence bundle.

## Later reported checkpoint: 2 November 2026

The figures above and their original hashes belong to 28 October. The operator
has since reported a completed 1.00 cm SDK height checkpoint, detailed in
[report Section 5.7](../../Buddy4_Terrain_Analysis_Report_Draft.md#57-reported-2-november-1-cm-sdk-height-validation).
The newer original recordings were subsequently identified and reviewed among
conversation uploads, but **have not yet been copied to the repository evidence
folder or hashed here**. The names and revised results are in
[report Section 5.7](../../Buddy4_Terrain_Analysis_Report_Draft.md#57-rechecked-2-november-1-cm-sdk-height-recordings).
The following table records what remains to archive:

| Recording | Reviewed finding | Archival follow-up |
|---|---|---|
| Flat | 505 complete rows, 10.08 s, mean live height 0.3187 cm | Copy exact conversation original and record SHA-256 |
| Left tyre Test 1 | Saved stable/median 1.242/1.312 cm | Preserve event/source bytes |
| Left tyre Test 2 | Saved stable/median 1.295/1.360 cm | Preserve event/source bytes |
| Right tyre Test 1 | Saved stable/median 0.467/0.640 cm, side +1 | Preserve event/source bytes |
| Right tyre Test 2 | Maximum live 0.750 cm, no saved SIDE_EVENT_END in supplied capture | Do not invent saved peak |


Rightward drift was operator-observed during the left-tyre tests and is referred
to Buddy 2 for investigation. The reported identified estimates satisfy +/-1 cm
for these conditions; no general accuracy claim follows. No raw bytes, filenames,
existing manifests or hashes were changed, and no new figure was generated.
