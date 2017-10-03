# host rules to generate the header pllconfig.h

${CHIBIOS_CONTRIB}/os/hal/ports/KINETIS/K64/hal_lld.c: ${BUILDDIR}/pllconfig.h

${BUILDDIR}/pllconfig.h: mcuconf.h ${BUILDDIR}/findpll
	@echo "HOST: Running findpll"
	@${BUILDDIR}/findpll -o $@

${BUILDDIR}/findpll: ${CHIBIOS_CONTRIB}/os/hal/boards/PJRC_TEENSY_3_5/findpll.c
	@echo "HOST: Compiling findpll.c"
	@gcc -Wall -Werror -std=c99 \
		-I . \
		-I ${CHIBIOS_CONTRIB}/os/hal/ports/KINETIS/K64 \
		-I ${CHIBIOS_CONTRIB}/os/hal/boards/PJRC_TEENSY_3_5 \
		-I ${CHIBIOS}/os/hal/ports/common/ARMCMx \
		-o $@ $^
