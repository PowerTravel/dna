configure_file(${MathGL_SOURCE_DIR}/texinfo/version.texi.in ${MathGL_BINARY_DIR}/texinfo/version.texi)

set(MGL_FONTS STIX adventor  bonum  cursor  heroscn  heros  pagella  schola  termes)
set(MGL_FONTS_BIN )
set(MGL_FONTS_SRC )

foreach(SAMPLE ${MGL_FONTS})
	set(MGL_FONTS_BIN ${MGL_FONTS_BIN} ${MathGL_BINARY_DIR}/fonts/${SAMPLE}.vfmb)
	add_custom_command(OUTPUT ${MathGL_BINARY_DIR}/fonts/${SAMPLE}.vfmb
		COMMAND make_bin -p ${MathGL_SOURCE_DIR}/fonts/ -o ${MathGL_BINARY_DIR}/fonts/${SAMPLE}.vfmb ${SAMPLE}
		DEPENDS ${SAMPLE}.vfm
		WORKING_DIRECTORY ${MGL_OUT} )
endforeach(SAMPLE)

add_custom_target(fonts ALL DEPENDS ${MGL_FONTS_BIN})

install(DIRECTORY ${MathGL_BINARY_DIR}/fonts/ DESTINATION ${MGL_FONT_PATH}
			PATTERN "*[mM]ake*" EXCLUDE
			PATTERN ".svn" EXCLUDE
			PATTERN "*.vfmb")
