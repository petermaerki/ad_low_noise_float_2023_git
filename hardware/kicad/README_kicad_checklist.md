# Plugins used

* https://github.com/bennymeg/JLC-Plugin-for-KiCad

  JLC PCB Plug-in for KiCad

  Fill Symbol Field `JLC Part`

* https://github.com/MitjaNemec/ReplicateLayout

## Hierarchical sheet

* https://forum.kicad.info/t/variables-in-hierarchical-sheet-labels/41978

In pcb, place all relais footprints (silkscreen) correctly.

Select first pcb, `Tools -> External Plugins -> Replicate Layout`

## Production

### Increment version number

Update in all sheets:

VSCode: Search an replace
 * `(date "2025-08-13")` -> `(date "2025-08-13")`
 * `(rev "0.4")` -> `(rev "0.4")`

### In schematics

Menu `Inspect -> Electrical Rule Checker`, button `Run ERC`, No violatons

### In pcb - final check and final commit

Menu `Tools -> Update PCB from Schematic`

Menu `Tools -> Cleanup Tracks & Vias`, select all,  `Build Changes`, No violations

Menu `Inspect -> Design Rules Chacker`
  * check `Refill all zones`
  * check `Test for parity`
  * button `Run DRC . No errors, no warnings.

Delete all files in directory `production`.

Icon `Fabricaton Toolkit`
  * Options empty
  * check `Apply automatic translatons`
  * check `Exluce DNP components`
  * check `Apply automatic fill of all zones`

Rename folder `production` to `production_vX.X`

Rename production folder and add version number

### Print schematics

Schematics, Menu `File -> Print`
 * uncheck all
 * check 'Print drawing sheet - Color`
 * `Print`, `All Pages`, `Print to File`.

Move `~/Documents/output.pdf` to `hardware/octoprobe_kicad_v0.1/production_v1.0/schematics.pdf`

### Upload to JLCPCB

Accept these warnings:
```
Error

The below parts won't be assembled due to data missing.
J17,J30,U204,J202,J12,J201,J14,J13,J16,C201,J15 designators don't exist in the BOM file.
IN_P204,IN_N201,IN_P205,IN_P206,IN_P207,IN_P208,M201,IN_P209,TP201,IN_P210,IN_P211,TP202,IN_P201,TP203,IN_P202,TP204,IN_P203 designators don't exist in the CPL file.
```

BOM
 * Verify that the correct values, specially C and R, have been choosen.
 * J201, ...: Do not place.

Use default values, but:
 * Tooling holes: Added by customer

Manual correction
 * USB Connectors: Manual postition
 * Various chips: rotate

