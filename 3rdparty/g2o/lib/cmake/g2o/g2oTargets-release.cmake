#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "g2o::freeglut_minimal" for configuration "Release"
set_property(TARGET g2o::freeglut_minimal APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(g2o::freeglut_minimal PROPERTIES
  IMPORTED_LOCATION_RELEASE "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_ext_freeglut_minimal.so"
  IMPORTED_SONAME_RELEASE "libg2o_ext_freeglut_minimal.so"
  )

list(APPEND _cmake_import_check_targets g2o::freeglut_minimal )
list(APPEND _cmake_import_check_files_for_g2o::freeglut_minimal "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_ext_freeglut_minimal.so" )

# Import target "g2o::stuff" for configuration "Release"
set_property(TARGET g2o::stuff APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(g2o::stuff PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "rt"
  IMPORTED_LOCATION_RELEASE "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_stuff.so"
  IMPORTED_SONAME_RELEASE "libg2o_stuff.so"
  )

list(APPEND _cmake_import_check_targets g2o::stuff )
list(APPEND _cmake_import_check_files_for_g2o::stuff "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_stuff.so" )

# Import target "g2o::opengl_helper" for configuration "Release"
set_property(TARGET g2o::opengl_helper APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(g2o::opengl_helper PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "OpenGL::GL;OpenGL::GLU"
  IMPORTED_LOCATION_RELEASE "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_opengl_helper.so"
  IMPORTED_SONAME_RELEASE "libg2o_opengl_helper.so"
  )

list(APPEND _cmake_import_check_targets g2o::opengl_helper )
list(APPEND _cmake_import_check_files_for_g2o::opengl_helper "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_opengl_helper.so" )

# Import target "g2o::core" for configuration "Release"
set_property(TARGET g2o::core APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(g2o::core PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "g2o::stuff"
  IMPORTED_LOCATION_RELEASE "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_core.so"
  IMPORTED_SONAME_RELEASE "libg2o_core.so"
  )

list(APPEND _cmake_import_check_targets g2o::core )
list(APPEND _cmake_import_check_files_for_g2o::core "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_core.so" )

# Import target "g2o::g2o_cli_library" for configuration "Release"
set_property(TARGET g2o::g2o_cli_library APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(g2o::g2o_cli_library PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "g2o::core;dl"
  IMPORTED_LOCATION_RELEASE "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_cli.so"
  IMPORTED_SONAME_RELEASE "libg2o_cli.so"
  )

list(APPEND _cmake_import_check_targets g2o::g2o_cli_library )
list(APPEND _cmake_import_check_files_for_g2o::g2o_cli_library "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_cli.so" )

# Import target "g2o::g2o_cli_application" for configuration "Release"
set_property(TARGET g2o::g2o_cli_application APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(g2o::g2o_cli_application PROPERTIES
  IMPORTED_LOCATION_RELEASE "/home/liuiu/桌面/slambook/3rdparty/g2o/bin/g2o"
  )

list(APPEND _cmake_import_check_targets g2o::g2o_cli_application )
list(APPEND _cmake_import_check_files_for_g2o::g2o_cli_application "/home/liuiu/桌面/slambook/3rdparty/g2o/bin/g2o" )

# Import target "g2o::g2o_hierarchical_library" for configuration "Release"
set_property(TARGET g2o::g2o_hierarchical_library APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(g2o::g2o_hierarchical_library PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "g2o::core"
  IMPORTED_LOCATION_RELEASE "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_hierarchical.so"
  IMPORTED_SONAME_RELEASE "libg2o_hierarchical.so"
  )

list(APPEND _cmake_import_check_targets g2o::g2o_hierarchical_library )
list(APPEND _cmake_import_check_files_for_g2o::g2o_hierarchical_library "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_hierarchical.so" )

# Import target "g2o::g2o_simulator_library" for configuration "Release"
set_property(TARGET g2o::g2o_simulator_library APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(g2o::g2o_simulator_library PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "g2o::types_slam3d_addons;g2o::types_slam3d;g2o::types_slam2d_addons;g2o::types_slam2d;g2o::core"
  IMPORTED_LOCATION_RELEASE "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_simulator.so"
  IMPORTED_SONAME_RELEASE "libg2o_simulator.so"
  )

list(APPEND _cmake_import_check_targets g2o::g2o_simulator_library )
list(APPEND _cmake_import_check_files_for_g2o::g2o_simulator_library "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_simulator.so" )

# Import target "g2o::g2o_simulator2d_application" for configuration "Release"
set_property(TARGET g2o::g2o_simulator2d_application APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(g2o::g2o_simulator2d_application PROPERTIES
  IMPORTED_LOCATION_RELEASE "/home/liuiu/桌面/slambook/3rdparty/g2o/bin/g2o_simulator2d"
  )

list(APPEND _cmake_import_check_targets g2o::g2o_simulator2d_application )
list(APPEND _cmake_import_check_files_for_g2o::g2o_simulator2d_application "/home/liuiu/桌面/slambook/3rdparty/g2o/bin/g2o_simulator2d" )

# Import target "g2o::g2o_simulator3d_application" for configuration "Release"
set_property(TARGET g2o::g2o_simulator3d_application APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(g2o::g2o_simulator3d_application PROPERTIES
  IMPORTED_LOCATION_RELEASE "/home/liuiu/桌面/slambook/3rdparty/g2o/bin/g2o_simulator3d"
  )

list(APPEND _cmake_import_check_targets g2o::g2o_simulator3d_application )
list(APPEND _cmake_import_check_files_for_g2o::g2o_simulator3d_application "/home/liuiu/桌面/slambook/3rdparty/g2o/bin/g2o_simulator3d" )

# Import target "g2o::g2o_anonymize_observations_application" for configuration "Release"
set_property(TARGET g2o::g2o_anonymize_observations_application APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(g2o::g2o_anonymize_observations_application PROPERTIES
  IMPORTED_LOCATION_RELEASE "/home/liuiu/桌面/slambook/3rdparty/g2o/bin/g2o_anonymize_observations"
  )

list(APPEND _cmake_import_check_targets g2o::g2o_anonymize_observations_application )
list(APPEND _cmake_import_check_files_for_g2o::g2o_anonymize_observations_application "/home/liuiu/桌面/slambook/3rdparty/g2o/bin/g2o_anonymize_observations" )

# Import target "g2o::g2o_viewer" for configuration "Release"
set_property(TARGET g2o::g2o_viewer APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(g2o::g2o_viewer PROPERTIES
  IMPORTED_LOCATION_RELEASE "/home/liuiu/桌面/slambook/3rdparty/g2o/bin/g2o_viewer"
  )

list(APPEND _cmake_import_check_targets g2o::g2o_viewer )
list(APPEND _cmake_import_check_files_for_g2o::g2o_viewer "/home/liuiu/桌面/slambook/3rdparty/g2o/bin/g2o_viewer" )

# Import target "g2o::viewer_library" for configuration "Release"
set_property(TARGET g2o::viewer_library APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(g2o::viewer_library PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "g2o::core;g2o::g2o_cli_library;/usr/lib/x86_64-linux-gnu/libQGLViewer-qt5.so;Qt5::Widgets;Qt5::Core;Qt5::Gui;Qt5::Xml;Qt5::OpenGL;/usr/lib/x86_64-linux-gnu/libGL.so;/usr/lib/x86_64-linux-gnu/libGLU.so;g2o::core;g2o::opengl_helper"
  IMPORTED_LOCATION_RELEASE "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_viewer.so"
  IMPORTED_SONAME_RELEASE "libg2o_viewer.so"
  )

list(APPEND _cmake_import_check_targets g2o::viewer_library )
list(APPEND _cmake_import_check_files_for_g2o::viewer_library "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_viewer.so" )

# Import target "g2o::types_data" for configuration "Release"
set_property(TARGET g2o::types_data APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(g2o::types_data PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "g2o::core;g2o::types_slam2d;g2o::freeglut_minimal;g2o::opengl_helper"
  IMPORTED_LOCATION_RELEASE "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_types_data.so"
  IMPORTED_SONAME_RELEASE "libg2o_types_data.so"
  )

list(APPEND _cmake_import_check_targets g2o::types_data )
list(APPEND _cmake_import_check_files_for_g2o::types_data "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_types_data.so" )

# Import target "g2o::types_slam2d" for configuration "Release"
set_property(TARGET g2o::types_slam2d APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(g2o::types_slam2d PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "g2o::core;g2o::opengl_helper;/usr/lib/x86_64-linux-gnu/libGL.so"
  IMPORTED_LOCATION_RELEASE "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_types_slam2d.so"
  IMPORTED_SONAME_RELEASE "libg2o_types_slam2d.so"
  )

list(APPEND _cmake_import_check_targets g2o::types_slam2d )
list(APPEND _cmake_import_check_files_for_g2o::types_slam2d "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_types_slam2d.so" )

# Import target "g2o::types_slam3d" for configuration "Release"
set_property(TARGET g2o::types_slam3d APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(g2o::types_slam3d PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "g2o::core;g2o::opengl_helper;/usr/lib/x86_64-linux-gnu/libGL.so"
  IMPORTED_LOCATION_RELEASE "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_types_slam3d.so"
  IMPORTED_SONAME_RELEASE "libg2o_types_slam3d.so"
  )

list(APPEND _cmake_import_check_targets g2o::types_slam3d )
list(APPEND _cmake_import_check_files_for_g2o::types_slam3d "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_types_slam3d.so" )

# Import target "g2o::types_sba" for configuration "Release"
set_property(TARGET g2o::types_sba APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(g2o::types_sba PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "g2o::core;g2o::types_slam3d"
  IMPORTED_LOCATION_RELEASE "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_types_sba.so"
  IMPORTED_SONAME_RELEASE "libg2o_types_sba.so"
  )

list(APPEND _cmake_import_check_targets g2o::types_sba )
list(APPEND _cmake_import_check_files_for_g2o::types_sba "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_types_sba.so" )

# Import target "g2o::types_sim3" for configuration "Release"
set_property(TARGET g2o::types_sim3 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(g2o::types_sim3 PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "g2o::types_sba"
  IMPORTED_LOCATION_RELEASE "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_types_sim3.so"
  IMPORTED_SONAME_RELEASE "libg2o_types_sim3.so"
  )

list(APPEND _cmake_import_check_targets g2o::types_sim3 )
list(APPEND _cmake_import_check_files_for_g2o::types_sim3 "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_types_sim3.so" )

# Import target "g2o::types_icp" for configuration "Release"
set_property(TARGET g2o::types_icp APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(g2o::types_icp PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "g2o::types_sba;g2o::types_slam3d"
  IMPORTED_LOCATION_RELEASE "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_types_icp.so"
  IMPORTED_SONAME_RELEASE "libg2o_types_icp.so"
  )

list(APPEND _cmake_import_check_targets g2o::types_icp )
list(APPEND _cmake_import_check_files_for_g2o::types_icp "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_types_icp.so" )

# Import target "g2o::types_sclam2d" for configuration "Release"
set_property(TARGET g2o::types_sclam2d APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(g2o::types_sclam2d PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "g2o::types_slam2d;g2o::core;/usr/lib/x86_64-linux-gnu/libGL.so"
  IMPORTED_LOCATION_RELEASE "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_types_sclam2d.so"
  IMPORTED_SONAME_RELEASE "libg2o_types_sclam2d.so"
  )

list(APPEND _cmake_import_check_targets g2o::types_sclam2d )
list(APPEND _cmake_import_check_files_for_g2o::types_sclam2d "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_types_sclam2d.so" )

# Import target "g2o::types_slam2d_addons" for configuration "Release"
set_property(TARGET g2o::types_slam2d_addons APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(g2o::types_slam2d_addons PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "g2o::types_slam2d;g2o::core;/usr/lib/x86_64-linux-gnu/libGL.so"
  IMPORTED_LOCATION_RELEASE "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_types_slam2d_addons.so"
  IMPORTED_SONAME_RELEASE "libg2o_types_slam2d_addons.so"
  )

list(APPEND _cmake_import_check_targets g2o::types_slam2d_addons )
list(APPEND _cmake_import_check_files_for_g2o::types_slam2d_addons "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_types_slam2d_addons.so" )

# Import target "g2o::types_slam3d_addons" for configuration "Release"
set_property(TARGET g2o::types_slam3d_addons APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(g2o::types_slam3d_addons PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "g2o::types_slam3d;g2o::core;/usr/lib/x86_64-linux-gnu/libGL.so"
  IMPORTED_LOCATION_RELEASE "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_types_slam3d_addons.so"
  IMPORTED_SONAME_RELEASE "libg2o_types_slam3d_addons.so"
  )

list(APPEND _cmake_import_check_targets g2o::types_slam3d_addons )
list(APPEND _cmake_import_check_files_for_g2o::types_slam3d_addons "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_types_slam3d_addons.so" )

# Import target "g2o::solver_pcg" for configuration "Release"
set_property(TARGET g2o::solver_pcg APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(g2o::solver_pcg PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "g2o::core"
  IMPORTED_LOCATION_RELEASE "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_solver_pcg.so"
  IMPORTED_SONAME_RELEASE "libg2o_solver_pcg.so"
  )

list(APPEND _cmake_import_check_targets g2o::solver_pcg )
list(APPEND _cmake_import_check_files_for_g2o::solver_pcg "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_solver_pcg.so" )

# Import target "g2o::solver_dense" for configuration "Release"
set_property(TARGET g2o::solver_dense APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(g2o::solver_dense PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "g2o::core"
  IMPORTED_LOCATION_RELEASE "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_solver_dense.so"
  IMPORTED_SONAME_RELEASE "libg2o_solver_dense.so"
  )

list(APPEND _cmake_import_check_targets g2o::solver_dense )
list(APPEND _cmake_import_check_files_for_g2o::solver_dense "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_solver_dense.so" )

# Import target "g2o::solver_structure_only" for configuration "Release"
set_property(TARGET g2o::solver_structure_only APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(g2o::solver_structure_only PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "g2o::core"
  IMPORTED_LOCATION_RELEASE "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_solver_structure_only.so"
  IMPORTED_SONAME_RELEASE "libg2o_solver_structure_only.so"
  )

list(APPEND _cmake_import_check_targets g2o::solver_structure_only )
list(APPEND _cmake_import_check_files_for_g2o::solver_structure_only "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_solver_structure_only.so" )

# Import target "g2o::solver_csparse" for configuration "Release"
set_property(TARGET g2o::solver_csparse APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(g2o::solver_csparse PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "g2o::csparse_extension;g2o::core"
  IMPORTED_LOCATION_RELEASE "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_solver_csparse.so"
  IMPORTED_SONAME_RELEASE "libg2o_solver_csparse.so"
  )

list(APPEND _cmake_import_check_targets g2o::solver_csparse )
list(APPEND _cmake_import_check_files_for_g2o::solver_csparse "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_solver_csparse.so" )

# Import target "g2o::csparse_extension" for configuration "Release"
set_property(TARGET g2o::csparse_extension APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(g2o::csparse_extension PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "/usr/lib/x86_64-linux-gnu/libcxsparse.so"
  IMPORTED_LOCATION_RELEASE "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_csparse_extension.so"
  IMPORTED_SONAME_RELEASE "libg2o_csparse_extension.so"
  )

list(APPEND _cmake_import_check_targets g2o::csparse_extension )
list(APPEND _cmake_import_check_files_for_g2o::csparse_extension "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_csparse_extension.so" )

# Import target "g2o::solver_slam2d_linear" for configuration "Release"
set_property(TARGET g2o::solver_slam2d_linear APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(g2o::solver_slam2d_linear PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "g2o::solver_csparse;g2o::types_slam2d"
  IMPORTED_LOCATION_RELEASE "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_solver_slam2d_linear.so"
  IMPORTED_SONAME_RELEASE "libg2o_solver_slam2d_linear.so"
  )

list(APPEND _cmake_import_check_targets g2o::solver_slam2d_linear )
list(APPEND _cmake_import_check_files_for_g2o::solver_slam2d_linear "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_solver_slam2d_linear.so" )

# Import target "g2o::solver_cholmod" for configuration "Release"
set_property(TARGET g2o::solver_cholmod APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(g2o::solver_cholmod PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "/usr/lib/x86_64-linux-gnu/libcholmod.so;/usr/lib/x86_64-linux-gnu/libamd.so;/usr/lib/x86_64-linux-gnu/libcolamd.so;/usr/lib/x86_64-linux-gnu/libcamd.so;/usr/lib/x86_64-linux-gnu/libccolamd.so;/usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so;/usr/lib/x86_64-linux-gnu/libcblas.so;/usr/lib/x86_64-linux-gnu/libf77blas.so;/usr/lib/x86_64-linux-gnu/libatlas.so;/usr/lib/x86_64-linux-gnu/liblapack.so;g2o::core"
  IMPORTED_LOCATION_RELEASE "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_solver_cholmod.so"
  IMPORTED_SONAME_RELEASE "libg2o_solver_cholmod.so"
  )

list(APPEND _cmake_import_check_targets g2o::solver_cholmod )
list(APPEND _cmake_import_check_files_for_g2o::solver_cholmod "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_solver_cholmod.so" )

# Import target "g2o::solver_eigen" for configuration "Release"
set_property(TARGET g2o::solver_eigen APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(g2o::solver_eigen PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "g2o::core"
  IMPORTED_LOCATION_RELEASE "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_solver_eigen.so"
  IMPORTED_SONAME_RELEASE "libg2o_solver_eigen.so"
  )

list(APPEND _cmake_import_check_targets g2o::solver_eigen )
list(APPEND _cmake_import_check_files_for_g2o::solver_eigen "/home/liuiu/桌面/slambook/3rdparty/g2o/lib/libg2o_solver_eigen.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
