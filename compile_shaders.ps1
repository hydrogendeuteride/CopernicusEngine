Get-ChildItem -Path "shaders" -Include *.frag,*.vert,*.comp,*.geom,*.tesc,*.tese,*.mesh,*.task,*.rgen,*.rint,*.rahit,*.rchit,*.rmiss,*.rcall -Recurse | ForEach-Object {
  glslc $_.FullName -o "$($_.FullName).spv"
}