# User CAD preparation

The authoritative print files are the STL assets in the project-level
`CAD_FILES/` directory. They remain unchanged and use millimetres.

`tools/import_user_cad.py` creates pivot-aligned simulation copies in
`meshes/cad/`:

```bash
cd ~/SELF_LEARNING/ROS/PROJECTS/PNP_ARM
python3 tools/import_user_cad.py
```

The importer centres the base parts, aligns each measured arm pivot with local
+X, and preserves the original source files. The older JavaScript parametric
generator in this directory is retained only as legacy material and is not
used by the current URDF.

Before printing, inspect each original STL in a slicer, select an orientation
appropriate to its load direction, and verify motor pockets and fastener holes
against the exact hardware. Servo and stepper clones vary; print a small fit
coupon or one motor housing before committing to the complete set.

Use at least four walls for loaded links, avoid supports inside bearing or
motor pockets, and bolt the radial `Base.stl` plate to a rigid surface during
operation.
