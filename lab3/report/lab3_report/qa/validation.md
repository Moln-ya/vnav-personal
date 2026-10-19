# Validation, 10 October 2026

## Evidence and numerical checks

- Final official CSV: 2,752 rows, all finite. State timestamps strictly increase.
  Reference timestamps are nondecreasing, with 1,832 repeated adjacent values.
  The logger pairs each state with the latest available reference; no rows were
  deduplicated or removed from the time-domain figures.
- The established `trajectory_time >= 8*pi` mask selects 1,495 rows and excludes
  1,257 initial rows from steady metrics. Actual selected endpoints: 25.18 and
  55.02 s. The exclusion is visible in the plots and described in the caption.
- Recomputed RMS, maximum error, radial RMS and observed turns match the existing
  JSON at relative tolerance 1e-10. Component RMS and height-error contribution
  were computed directly from the same CSV, without new flight experiments.
- Both circular trial RMS values were independently recalculated from the trial
  CSVs using `trajectory_time >= 20`, matching their retained JSON summaries.
- Drag bars read the retained numerical validation log. Both use a 1 ms step,
  drag enabled, and one deterministic run; one enables compensation. The
  uncompensated-drag time-series CSV is not retained, so this figure presents the
  logged aggregate values and does not reconstruct a missing time series.
- Normalised allocation matrices have ranks 4 and 3; the dependent row and null
  vector are checked with assertions in the figure source. Rotor positions and
  directions were visually checked against both official course diagrams.
- Build/runtime claims come from existing logs, not a new execution. No simulator,
  ROS build, flight, remote submission or cloud upload was performed.

## Figure and image QA

- Python/matplotlib used exclusively for generated figures. Eight source figures
  inspected visually; PDF/SVG text remains embedded/editable. All nine report
  figures have explanatory captions and are referenced in the text.
- Figure preflight: 12 PASS, 2 WARN, 0 FAIL. PNG preview instead of TIFF is
  intentional for this PDF report. The width warning is a static-parser false
  positive on `160/25.4`; the layout targets the template's 160 mm text width,
  rather than a journal's 89/183 mm columns. PNG exports are 600 dpi.
- RViz screenshot is unchanged, with no crop, recolouring, contrast adjustment,
  generated content or compositing. It is scaled uniformly in LaTeX. Colours
  were checked against the RViz configuration. The screenshot is illustrative;
  all quantitative flight metrics come from the CSV.
- One official flight supplies the main statistics; time samples are correlated.
  No confidence intervals, hypothesis tests or independent replicate claims.

## PDF and writing QA

- PDF inspector fallback is documented in writing_plan.md. Template PDF and
  reference-paper text were extracted with Poppler after the unavailable or
  incompatible pdf-inspector commands were identified.
- Template PDF is an older mechanics example; Lab 3's template.tex determines
  structure. Kept A4, 12 pt, 25 mm margins and all five section headings.
- Local TeX lacks cleveref; explicit Figure/Table/Equation references use standard
  LaTeX instead. Latin Modern supplies scalable fonts for microtype.
- Final LaTeX passes succeeded; no undefined citations/references or overfull
  boxes. Three underfull horizontal-box messages concern sparse text/long source
  URLs and cause no clipping in the reviewed PDF. All fonts are embedded.
- The original pages were visually checked, and all changed pages were reviewed
  again after expansion. The report now has 13 pages; the conclusion ends on
  page 12 and references occupy page 13. Summary,
  equations, figure labels and bibliography are readable.
- humanizer applied in embedded mode: plain technical language, no unsupported
  novelty, no promotional contrasts or rhetorical endings. anti-defensive-writing
  centres the demonstrated circular flight; interpretation remains tied to
  measured data. nature-writing supplies the evidence-led argument and terminology.
- Author metadata remains blank by explicit user instruction. Public video link
  is absent; the existing local-only delivery instruction is retained.

## Expanded figure checks

- Added spatial tracking (all 2,752 rows), component errors (1,495 steady rows),
  gain comparison, and full/steady rotor command curves. No new experiments.
- Gain plot retains 1,999 initial-trial rows and 2,008 higher-gain rows; RMS
  summaries select 1,001 and 1,011 rows respectively at reference time >=20 s.
  Both numbers match the existing trial JSON. Starting-state differences are
  disclosed; no causal attribution to one gain is made.
- Component RMS squared sums to the three-dimensional RMS squared (asserted).
  Common error limits enclose every selected observation, including all peaks.
- Four command channels are finite; signed extrema are -160.3262 and
  162.3279 rad/s. Captions distinguish logged commands from measured rotor motion.
- Spatial axes use different scales, disclosed in the caption; label spacing
  checked in both standalone figure and report rendering.
- Final source preflight and two LaTeX passes succeeded; no overfull boxes or
  unresolved references. Final PDF replaces the previous report at the same path.

## Section 2.1 rewrite

- Replaced only section 2.1 in the report source. Prefix and suffix were checked
  byte-for-byte against the pre-edit copy; all other prose and figures are unchanged.
- Added the reason for each representation, the meaning of each conversion, and
  the actual desired/current-state callback use. Retained all five exercise answers.
- Checked callback descriptions against controller_node.cpp. No controller change.
- Two LaTeX passes succeeded without overfull boxes or unresolved references.
  Section 2.1 and the following pages were rendered and visually inspected.
  The added explanation increases the report from 12 to 13 pages.
