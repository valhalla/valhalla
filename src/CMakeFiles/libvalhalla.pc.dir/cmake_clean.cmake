file(REMOVE_RECURSE
  "libvalhalla.pc"
  "libvalhalla.pc.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/libvalhalla.pc.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
