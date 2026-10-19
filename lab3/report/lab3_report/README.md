# Lab 3 report

- [Final English report](lab3_report.pdf): 13 A4 pages, nine figures and two tables.
- [Editable LaTeX](lab3_report.tex) and [references](references.bib).
- [Figure source](make_figures.py): regenerates the eight vector figures from existing records.
- [Recomputed metrics](qa/recomputed_metrics.json) and [validation notes](qa/validation.md).

The report follows `../../ReportTemplate/template.tex`: 12 pt article, 25 mm
margins, and the five main sections. The original template is unchanged.
Name, student number, lab group, lab date, team members and task allocation are
intentionally left blank at the user's request. Edit the macros at the top of
the LaTeX source and the team-information line when those details are available.

## Material-to-file index

Paths below are relative to this directory. Existing evidence stays in place.

| Report content | Existing source |
| --- | --- |
| Exercise specification | [MIT VNAV Lab 3](https://vnav.mit.edu/labs_2023/lab3/exercises.html) |
| Deliverable 1 answers | [deliverable_1.txt](../../deliverable_1.txt) |
| Rotor rank and drag derivation | [deliverable_2.tex](../deliverable_2.tex), verified against the official rotor diagram |
| Controller equations and allocation | [geometric_controller.h](../../controller_pkg/include/controller_pkg/geometric_controller.h) |
| ROS conversions and input handling | [controller_node.cpp](../../controller_pkg/src/controller_node.cpp) |
| Circular trajectory derivatives | [traj_publisher.cpp](../../controller_pkg/src/traj_publisher.cpp) |
| Official gains | [gains_unity.yaml](../../controller_pkg/config/gains_unity.yaml) |
| Figure 2: independent-model drag comparison | [numerical_validation.txt](../numerical_validation.txt) |
| Figures 4, 5, 6 and 8, Table 1: official final flight | [unity_flight.csv](../unity_flight.csv), [unity_metrics.json](../unity_metrics.json) |
| Figure 7, Table 2: parameter trials | [initial CSV](../trials/unity_circle_initial.csv), [higher-gain CSV](../trials/unity_circle_tuned.csv), [validation record](../validation.md) |
| Figure 9: unchanged RViz image | [rviz_official_frame.png](../video/rviz_official_frame.png) |
| Official flight video | [excerpt](../video/rviz_official_tesse.mp4), [full recording](../video/rviz_official_tesse_full.mp4) |
| Video duration and format | [rviz_official_metadata.json](../video/rviz_official_metadata.json) |
| Bag inventory and build evidence | [unity_bag_info.txt](../unity_bag_info.txt), [build_unity_noetic.txt](../build_unity_noetic.txt) |

## Rebuild

Run from this directory, with Python, NumPy, matplotlib and LaTeX installed:

```sh
MPLCONFIGDIR=/tmp/lab3-mpl python make_figures.py
pdflatex -interaction=nonstopmode -halt-on-error -output-directory=qa lab3_report.tex
bibtex qa/lab3_report
pdflatex -interaction=nonstopmode -halt-on-error -output-directory=qa lab3_report.tex
pdflatex -interaction=nonstopmode -halt-on-error -output-directory=qa lab3_report.tex
cp qa/lab3_report.pdf lab3_report.pdf
```

These commands process existing data and compile the report. They do not launch
ROS, a simulator or new experiments. All figures have PDF/SVG vector exports and
600 dpi PNG previews. The screenshot is included directly from its existing path.
