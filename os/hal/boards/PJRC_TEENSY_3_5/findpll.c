#include <getopt.h>

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "mcuconf.h"
#include "board.h"
#include "hal_lld.h"

int main(int argc, char *argv[])
{
	int c;
	char *output = NULL;
	while ((c = getopt(argc, argv, "ho:")) != -1) {
		switch (c) {
		case 'h':
			fprintf(stderr,
				"Usage: %s [OPTIONS]\n\n"
				"\t-h\tthis help message\n"
				"\t-o\toutput path\n\n", argv[0]);
			return EXIT_SUCCESS;

		case 'o':
			output = optarg;
			break;
		}
	}

	if (output != NULL) {
		/* pass throgh */
	} else if (optind < argc) {
		output = argv[optind];
	} else {
		fputs("Please provide an output parameter.\n", stderr);
		return EXIT_FAILURE;
	}

	uint64_t error = KINETIS_XTAL_FREQUENCY;
	unsigned prdiv, best_prdiv;
	unsigned vdiv, best_vdiv;
	for (prdiv = (KINETIS_XTAL_FREQUENCY + 3999999UL) / 4000000UL;
	     prdiv <= (KINETIS_XTAL_FREQUENCY + 1999999UL) / 2000000UL;
	     prdiv++) {
		for (vdiv = 24U; vdiv < 56U; vdiv++) {
			uint64_t freq = (uint64_t)KINETIS_XTAL_FREQUENCY * vdiv / prdiv;
			uint64_t e = KINETIS_PLLCLK_FREQUENCY > freq ?
				KINETIS_PLLCLK_FREQUENCY - freq :
				freq - KINETIS_PLLCLK_FREQUENCY;

			if (error > e) {
				error = e;
				best_prdiv = prdiv;
				best_vdiv = vdiv;
			}
		}
	}

	if (error != 0) {
		fprintf(stderr, "\n"
			"ERROR - Couldn't find a suitable PLL configuration:\n"
			"  Required frequency: %10lu\n"
			"  Nearest frequency:  %10lu\n"
			"  Error:              %10ld\n"
			"  PRDIV:              %10u\n"
			"  VDIV:               %10u\n"
			"\n",
			KINETIS_PLLCLK_FREQUENCY,
			(uint64_t)KINETIS_XTAL_FREQUENCY * best_vdiv / best_prdiv,
			error,
			best_prdiv - 1U,
			best_vdiv - 24U);
		return EXIT_FAILURE;
	}

	FILE *out = fopen(output, "wt");
	if (out) {
		fprintf(out, "#define PLL_PRDIV %u\n", best_prdiv - 1U);
		fprintf(out, "#define PLL_VDIV  %u\n", best_vdiv - 24U);
		fclose(out);
	}

	return EXIT_SUCCESS;
}
