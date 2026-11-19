# Repository instructions

- For each completed change that affects ParamCAD Studio or its shipped runtime, increment the patch component in `apps/ParamCAD Studio/VERSION` once, unless the user specifies a different release version. Group the edits for one task under a single increment.
- Build the updated client when possible and verify the new version with `pcad --version` from that build. Report its product version and build identity. CMake records the Git commit and working-tree state automatically.
- Keep the product version in `apps/ParamCAD Studio/VERSION`. Documentation-only and test-only edits do not require a product version increment.

- Treat `docs/Outdated/` as an archive of superseded design documents and historical records.
- Exclude `docs/Outdated/` from searches and do not read or use archived documents as context unless the user explicitly asks to read them. In that case, read only the requested documents.

- Treat `deps/colmap/` as an opaque, local runtime dependency.
- Do not recursively list, search, read, summarize, or otherwise add files from that directory to AI context.
- Access only the exact `bin/colmap.exe` path for execution or basic file metadata when a task explicitly requires COLMAP runtime validation.

- Treat `deps/vtk-9.7.0/` as an opaque, local VTK SDK dependency.
- Do not recursively list, search, read, summarize, or otherwise add files from that directory to AI context.
- Access only exact files required for CMake configuration, linking, runtime validation, or basic file metadata.

- Treat `deps/occt-8.0.1/` as an opaque, local OpenCASCADE SDK dependency.
- Do not recursively list, search, read, summarize, or otherwise add files from that directory to AI context.
- Access only exact files required for CMake configuration, linking, runtime validation, or basic file metadata.
