# Open questions for the DRS RADA mentors

Fill in the answers as soon as you get them, then run task 10 in docs/TASKS.md.

| # | Question | Answer |
| --- | --- | --- |
| 1 | What mechanical reference (datum) exists on the radar: pins, machined surface, optical cube? May we clamp a fixture to it? | |
| 2 | Which side may we move? Is there a motorized Az/El positioner for the radar? | We do not move the radar ourselves (positioner not mentioned); radar aligned once before the test, static during it. We move the horn (2026-11-16) |
| 3 | Does +/-0.05 deg apply to radar error vs line of sight, to horn pointing, or both? | |
| 4 | Is roll relevant (polarization)? | |
| 5 | Horn details: size, weight, beamwidth. What is "horn center": aperture center or phase center? | |
| 6 | Typical initial error after manual setup? Time allowed for alignment? | |
| 7 | Field conditions: max wind, temperature, dust, night work, available power? | |
| 8 | "Real-time correction": may motors move during radar measurement, or only between measurements? | |
| 9 | May we place components near the radar aperture (RF reflections)? | |
| 10 | What is expected at submission and what are the judging criteria? Budget or equipment available? | Budget up to USD 30k for hardware if justified (2026-11-16). Judging criteria still open |
| 11 | Does the datum allow rotating a fixture 180 deg about the reference axis (pins, bore, cylindrical feature), or is it a flat face only? | |
| 12 | Does the radar integrate over seconds? I.e. does +/-0.05 deg apply to the mean pointing over a measurement or to the instantaneous pointing? | |
| 13 | Does the radar move during a test? | No. Aligned to its mechanical axis before the test, static during it (2026-11-16) |
| 14 | Mentor point 5: simple sensors as backup, or do not over-rely on them either? | As backup (2026-11-16) |
| 15 | What does "calibrated to 0,0" mean? | Perpendicular plane (90 deg): the reference point the radar's measurement starts from. Our reading: the radar's mechanical axis, face perpendicular to the line of sight (medium confidence) (2026-11-16) |
| 16 | May we show radar data (display only)? | Not needed (2026-11-16). Open: a radar transmitting/recording STATUS for automatic hold? |
| 17 | Will we ever need to move the 35 kg radar ourselves? | No; the system must align by itself (2026-11-16) |
| 18 | Radar frequency band; radio near the radar? | 4-8 GHz (C-band) (2026-11-16). Radio: not asked; we assume none (5 GHz WiFi is in band) |
| 19 | Real range of today's manual radar setup error (our assumption: +/-1 deg)? | |
| 20 | Radar antenna aperture size (far field 2D^2/lambda: 0.5 m at 8 GHz = 13.3 m)? | |
| 21 | Can the radar PC give a transmitting/recording status signal for automatic hold? | |
