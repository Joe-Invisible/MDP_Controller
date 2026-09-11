#include "CommandParser.h"

#include <math.h>
#include <stdlib.h>
#include <stdbool.h>

/*
 * Parses one command at *cursor, advancing it past what was consumed.
 * The cursor stops at ';', NUL, or the offending character on error.
 */
static CommandParserStatus ParseOneCommand(const char **cursor, Command *out) {
	const char *p = *cursor;

	CommandType type;
	bool needsParam = true;

	switch (*p) {
	case 'F': type = COMMAND_FORWARD;  break;
	case 'B': type = COMMAND_BACKWARD; break;
	case 'L': type = COMMAND_LEFT;     break;
	case 'R': type = COMMAND_RIGHT;    break;
	case 'S': type = COMMAND_STOP; needsParam = false; break;
	default:
		return COMMANDPARSER_ERROR_BAD_COMMAND;
	}
	p++;

	float param = 0.0f;
	if (needsParam) {
		/*
		 * strtof is far more permissive than this grammar: it accepts
		 * leading whitespace and signs, inf/nan, hex floats ("0x10")
		 * and exponents ("1e4"). Requiring a leading digit stops the
		 * signs and the whitespace, but "0x10" and "1e999" both begin
		 * with a digit and slipped through -- the latter arriving as
		 * infinity, which wedges the command queue permanently because
		 * the motion never completes.
		 *
		 * So walk the plain-decimal form ourselves first and insist the
		 * span ends at a separator. That rejects 'x' and 'e' before
		 * strtof ever sees them.
		 */
		/* A leading digit is required: ".5" is not in the grammar. */
		if (*p < '0' || *p > '9')
			return COMMANDPARSER_ERROR_BAD_COMMAND;

		const char *digits = p;
		while (*digits >= '0' && *digits <= '9')
			digits++;
		if (*digits == '.') {
			digits++;
			while (*digits >= '0' && *digits <= '9')
				digits++;
		}

		if (*digits != ';' && *digits != '\0')
			return COMMANDPARSER_ERROR_BAD_COMMAND;

		char *end = NULL;
		param = strtof(p, &end);

		/*
		 * A long enough run of digits still overflows to infinity, so
		 * the finiteness check stays even with the charset validated.
		 */
		if (end != digits || !isfinite(param) || param <= 0.0f)
			return COMMANDPARSER_ERROR_BAD_COMMAND;
		p = end;
	}

	/* Anything but a separator or end-of-frame here is trailing junk. */
	if (*p != ';' && *p != '\0')
		return COMMANDPARSER_ERROR_BAD_COMMAND;

	out->type = type;
	out->param = param;
	*cursor = p;
	return COMMANDPARSER_OK;
}

CommandParserStatus CommandParser_ParseFrame(
		const char *frame,
		CommandFrame *out) {

	out->count = 0;
	out->seq = 0;
	out->hasSeq = false;

	if (frame == NULL || *frame == '\0')
		return COMMANDPARSER_ERROR_EMPTY;

	const char *p = frame;

	/* Optional sequence prefix: digits followed by ':'. */
	if (*p >= '0' && *p <= '9') {
		char *end = NULL;
		long seq = strtol(p, &end, 10);
		if (*end != ':' || seq < 0 || seq > 255)
			return COMMANDPARSER_ERROR_BAD_SEQ;
		out->seq = (uint8_t)seq;
		out->hasSeq = true;
		p = end + 1;
	}

	size_t count = 0;
	for (;;) {
		if (count >= out->capacity)
			return COMMANDPARSER_ERROR_TOO_MANY;

		CommandParserStatus status =
				ParseOneCommand(&p, &out->commands[count]);
		if (status != COMMANDPARSER_OK)
			return status;
		count++;

		if (*p == '\0')
			break;
		p++;	/* skip ';' */
	}

	out->count = count;
	return COMMANDPARSER_OK;
}

const char *CommandParser_StatusName(CommandParserStatus status) {
	switch (status) {
	case COMMANDPARSER_OK:                    return "OK";
	case COMMANDPARSER_ERROR_EMPTY:           return "EMPTY";
	case COMMANDPARSER_ERROR_BAD_COMMAND:     return "PARSE";
	case COMMANDPARSER_ERROR_BAD_SEQ:         return "BAD_SEQ";
	case COMMANDPARSER_ERROR_TOO_MANY:        return "TOO_MANY";
	default:                                  return "UNKNOWN";
	}
}
