/*
 * Host-side unit test for CommandParser. Not part of the firmware
 * build: this directory is not a CubeIDE source root.
 *
 *   cc hosttests/parser_test.c Apps/Src/CommandParser.c -IApps/Inc -o parser_test
 */

#include <stdio.h>
#include <math.h>

#include "CommandParser.h"

static int failures = 0;

#define CHECK(cond) do { \
	if (!(cond)) { \
		failures++; \
		printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
	} \
} while (0)

static CommandFrame lastFrame;

/* Parses into the shared frame record, mirroring CommandTask's use. */
static CommandParserStatus Parse(const char *frame, Command *out, size_t *n) {
	lastFrame = (CommandFrame){ .commands = out, .capacity = 8 };
	CommandParserStatus status = CommandParser_ParseFrame(frame, &lastFrame);
	*n = lastFrame.count;
	return status;
}

int main(void) {
	Command cmd[8];
	size_t n;

	/* Single command, no sequence prefix */
	CHECK(Parse("F100", cmd, &n) == COMMANDPARSER_OK);
	CHECK(n == 1 && cmd[0].type == COMMAND_FORWARD && fabsf(cmd[0].param - 100.0f) < 1e-6f);

	/* Fractional parameter */
	CHECK(Parse("L90.5", cmd, &n) == COMMANDPARSER_OK);
	CHECK(n == 1 && cmd[0].type == COMMAND_LEFT && fabsf(cmd[0].param - 90.5f) < 1e-4f);

	/* Stop takes no parameter */
	CHECK(Parse("S", cmd, &n) == COMMANDPARSER_OK);
	CHECK(n == 1 && cmd[0].type == COMMAND_STOP);

	/* Batch with a sequence prefix */
	CHECK(Parse("3:F100;L90;B50", cmd, &n) == COMMANDPARSER_OK);
	CHECK(n == 3);
	CHECK(lastFrame.hasSeq && lastFrame.seq == 3);
	CHECK(cmd[0].type == COMMAND_FORWARD  && fabsf(cmd[0].param - 100.0f) < 1e-6f);
	CHECK(cmd[1].type == COMMAND_LEFT     && fabsf(cmd[1].param -  90.0f) < 1e-6f);
	CHECK(cmd[2].type == COMMAND_BACKWARD && fabsf(cmd[2].param -  50.0f) < 1e-6f);

	/* Batch without a sequence prefix */
	CHECK(Parse("R45;S", cmd, &n) == COMMANDPARSER_OK);
	CHECK(n == 2 && cmd[0].type == COMMAND_RIGHT && cmd[1].type == COMMAND_STOP);
	CHECK(!lastFrame.hasSeq);

	/* The command count is no longer constrained by the prefix */
	CHECK(Parse("2:F100;L90;B50", cmd, &n) == COMMANDPARSER_OK);
	CHECK(n == 3 && lastFrame.seq == 2);

	/* Sequence range is a full wrapping byte */
	CHECK(Parse("0:F100", cmd, &n) == COMMANDPARSER_OK);
	CHECK(lastFrame.hasSeq && lastFrame.seq == 0);
	CHECK(Parse("255:F100", cmd, &n) == COMMANDPARSER_OK);
	CHECK(lastFrame.seq == 255);
	CHECK(Parse("256:F100", cmd, &n) == COMMANDPARSER_ERROR_BAD_SEQ);
	CHECK(Parse("3F100", cmd, &n) == COMMANDPARSER_ERROR_BAD_SEQ);

	/* A sequence prefix with no commands after it */
	CHECK(Parse("7:", cmd, &n) == COMMANDPARSER_ERROR_BAD_COMMAND);

	/* Unknown command letter */
	CHECK(Parse("X100", cmd, &n) == COMMANDPARSER_ERROR_BAD_COMMAND);

	/* Missing required parameter */
	CHECK(Parse("F", cmd, &n) == COMMANDPARSER_ERROR_BAD_COMMAND);
	CHECK(Parse("F;S", cmd, &n) == COMMANDPARSER_ERROR_BAD_COMMAND);

	/* Stop with a parameter is trailing junk */
	CHECK(Parse("S100", cmd, &n) == COMMANDPARSER_ERROR_BAD_COMMAND);

	/* Signs and whitespace are outside the grammar */
	CHECK(Parse("F-100", cmd, &n) == COMMANDPARSER_ERROR_BAD_COMMAND);
	CHECK(Parse("F 100", cmd, &n) == COMMANDPARSER_ERROR_BAD_COMMAND);
	CHECK(Parse("F100; S", cmd, &n) == COMMANDPARSER_ERROR_BAD_COMMAND);

	/* Zero magnitude is not a motion */
	CHECK(Parse("F0", cmd, &n) == COMMANDPARSER_ERROR_BAD_COMMAND);

	/*
	 * strtof is more permissive than this grammar. Each of these begins
	 * with a digit, so the leading-digit guard alone let them through.
	 * "1e999" was the dangerous one: it arrived as infinity, the motion
	 * never completed, IsBusy() never cleared and the command queue
	 * jammed until reset.
	 */
	CHECK(Parse("F1e999", cmd, &n) == COMMANDPARSER_ERROR_BAD_COMMAND);
	CHECK(Parse("F1E5",   cmd, &n) == COMMANDPARSER_ERROR_BAD_COMMAND);
	CHECK(Parse("F100e2", cmd, &n) == COMMANDPARSER_ERROR_BAD_COMMAND);
	CHECK(Parse("F0x10",  cmd, &n) == COMMANDPARSER_ERROR_BAD_COMMAND);
	CHECK(Parse("F0X10",  cmd, &n) == COMMANDPARSER_ERROR_BAD_COMMAND);
	CHECK(Parse("Fnan",   cmd, &n) == COMMANDPARSER_ERROR_BAD_COMMAND);
	CHECK(Parse("Finf",   cmd, &n) == COMMANDPARSER_ERROR_BAD_COMMAND);
	CHECK(Parse("F99999999999999999999999999999999999999999999999999",
			cmd, &n) == COMMANDPARSER_ERROR_BAD_COMMAND);

	/* A leading digit is still required, and fractions still work. */
	CHECK(Parse("F.5",    cmd, &n) == COMMANDPARSER_ERROR_BAD_COMMAND);
	CHECK(Parse("F0.5",   cmd, &n) == COMMANDPARSER_OK);
	CHECK(fabsf(cmd[0].param - 0.5f) < 1e-6f);

	/* Separator with nothing after it */
	CHECK(Parse("F100;", cmd, &n) == COMMANDPARSER_ERROR_BAD_COMMAND);

	/* Empty frame */
	CHECK(Parse("", cmd, &n) == COMMANDPARSER_ERROR_EMPTY);
	CHECK(Parse(NULL, cmd, &n) == COMMANDPARSER_ERROR_EMPTY);

	/* Capacity limit is enforced before writing past out[] */
	lastFrame = (CommandFrame){ .commands = cmd, .capacity = 2 };
	CHECK(CommandParser_ParseFrame("F1;F2;F3", &lastFrame)
			== COMMANDPARSER_ERROR_TOO_MANY);
	CHECK(lastFrame.count == 0);

	/* Garbage resembling line noise */
	CHECK(Parse("\x7f\x03garbage", cmd, &n) == COMMANDPARSER_ERROR_BAD_COMMAND);

	if (failures == 0)
		printf("parser_test: all checks passed\n");
	return failures == 0 ? 0 : 1;
}
