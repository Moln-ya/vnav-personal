# Report and figure contract

Audience: the Lab 3 instructor. Follow the supplied article template: A4, 12 pt,
25 mm margins, and its five main sections. English prose; author, date and team
fields deferred at the user's request.

Argument: the ROS geometric controller follows the specified circular path in
recorded official TESSE flight; rotor allocation explains actuation authority,
and the given drag model motivates a separately evaluated compensation term.
No new controller experiments are required for this report.

Paragraph map: task and achieved result; quaternion conversions; rank proof;
drag compensation; ROS data flow and geometric control; measured tracking;
parameter comparison; interpretation; conclusion.

Terminology ledger:
- geometric controller: implemented feedback law on SO(3)
- TESSE: official Unity simulator used in the recorded flight
- independent rigid-body model: separate numerical validation plant
- R: body-to-world rotation; Omega: body angular velocity
- position RMS error: square root of mean squared 3D position-error norm
- steady window: trajectory time >= 8*pi seconds in the final official run

Figures use Python/matplotlib exclusively, 160 mm report width, white background,
Arial sans-serif, editable SVG and PDF text, and 300 dpi PNG previews.
1. Rotor layout: schematic-led composite. Explain yaw-sign reversal and rank loss.
2. ROS loop: schematic-led composite. Connect reference, feedback and motor commands.
3. Official flight: quantitative grid with a larger trajectory panel, full-run
   altitude/error panels, and observed steady-window phase progression.
4. Drag comparison: quantitative plot using the retained numerical validation log.
5. Existing RViz screenshot: unchanged real recording frame, not synthesized.

Integrity: plot every official CSV row; use the pre-existing t>=8*pi rule only
for reported steady metrics and phase. Record row counts. One flight, correlated
samples; no replicate statistics or hypothesis tests. Independent-model numbers
must not be labelled TESSE. No re-running experiments, replacing records or upload.

PDF fallback: detect-pdf ReportTemplate/template.pdf --json failed with
'command not found'. The installed /opt/homebrew/bin/pdf2md is an unrelated Node
package expecting <file.pdf> <projectname>; it lacks the pdf-inspector interface.
Use pdftotext for textual inspection and pdftoppm for final visual inspection.
The supplied template.pdf is an older mechanics example; template.tex is the
Lab 3-specific structural authority.

## Figure expansion

Add four quantitative figures, keeping existing claims and prose intact:
- Spatial trajectory: asymmetric two-panel view. All official x/y/z samples in
  3D and an x/z projection make the altitude offset visible alongside the circle.
- Component errors: three steady-window time traces plus RMS bars explain the
  vertical contribution. Use the existing 1,495-row mask, no further exclusion.
- Gain trials: compare both complete error traces and the existing t>=20 s RMS
  statistics, with one recorded run per setting and distinct starting states.
- Rotor commands: all four logged command channels over the complete final run,
  plus a steady-window view; rad/s, commands rather than measured rotor motion.
Use the existing Python style and export function; each new figure is a new
composition, not an adaptation of an incompatible statistical template.
