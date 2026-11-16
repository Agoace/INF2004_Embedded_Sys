# SIL barcode geometry and mat edge — observe only

Approved2026-11-16. First payload **A** (`*A*`); large-print widths5.8/17.5mm,
29elements, ink body273.5mm. No reader thresholds, letters or distances changed.

| Item | Value / evidence |
| --- | --- |
| Paper |297mm scan direction x210mm; MEASURED(operator:A4, landscape implied by body) |
| Print |Centred ASSUMED; transverse bar height160±10mm PHOTO-ESTIMATED IMG_1924/1938 |
| Placement |Inner paper edge=line centre+25mm ASSUMED; sheet centre=line+130mm |
| GP28 |Axle+80forward/+77left, height5.5mm; board axis U=body-left at yaw0 |
| Junction |Paper end to junction centre150mm ASSUMED |
| Quiet zone |11.75mm paper each end;10n=58mm total each end therefore includes46.25mm mat |

For line_y157.5 and aligned axle_y150.25, GP28 track_y227.25. Paper spans
182.5..392.5; ink spans207.5..367.5, so GP28 crosses the print while the paper
stays clear of GP26/27. The sheet is not centred on the sensor track.
Zero paper thickness is an explicit coplanar-layer assumption, not a survey.

`calibration/s3_sil_barcode_assumptions.json` records the measured-unlocated
MQTT proxies: white178/dark2321ADC, from boot3162310911,3233observations.
Low/high cluster counts2134/1099 and MAD4/12 are not stationary-noise estimates.
Method: filter robot/state+boot+valid integer raw_bc, initialize two centres
at min/max, assign nearest (ties low), update medians until stable. Source
SHA256 is in the record. These clusters are not identified pure materials;
assigning them to mat white/dark and linear mat coverage power1 are assumptions.
Earlier boot high-cluster3437 is not pooled into this boot's proxy.

```sh
python3 -S -m sim.webots.pil.s3_sil_barcode build/mtk_logs/robot-mqtt-20261111-163412.csv
```

API: `barcode_sheet(sheet_start_mm=..., line_centre_y_mm=..., surface_z_mm=...)`
returns bounds and `.geometry.patches`; caller combines these with common
floor/line geometry. `load_sil_barcode_profile(sigma_along_mm=1)` selects the
sensor-X paper fit plus explicit mat proxies; approved4/8mm sensitivity remains
an along-bar assumption. Frozen and offline GP28 profile files are unchanged.

Use the full scene with rotated `board_axis_u(yaw)`. Visible materials are
integrated across paper/mat/ink edges, never switched by the sensor centre.
At a white-mat/white-paper half-plane edge the mean is171ADC, not fake white
or invalid; far values are178/164ADC. Tests compare128x128 rays to the
independent boundary referee. This edge check does not certify barcode-wide
quadrature accuracy: require convergence/referee checks before reader results.
F1 anisotropic support is unchanged. Reader remains SHADOW/valid0; decoded A
cannot authorize LEFT. This is geometry/model evidence, not hardware acceptance.
