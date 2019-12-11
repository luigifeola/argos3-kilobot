# If Qt+OpenGL dependencies were found, descend into these directories
if(ARGOS_COMPILE_QTOPENGL)
  	add_subdirectory(demoC)
  	add_subdirectory(clustering)
  	add_subdirectory(crwlevy)
  	add_subdirectory(navigation)
	add_subdirectory(crwlevy_with_positioning)
endif(ARGOS_COMPILE_QTOPENGL)
