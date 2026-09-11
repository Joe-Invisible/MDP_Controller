#ifndef INC_COMMANDPARSER_H_
#define INC_COMMANDPARSER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Pure frame parser for the RPi command link.
 *
 * Deliberately free of HAL/OS includes so it can be compiled and
 * tested on the host (see hosttests/parser_test.c).
 *
 * Frame grammar (terminator '\n' already stripped by the caller):
 *
 *   frame    :=  [seq ':'] command (';' command)*
 *   command  :=  'F' number | 'B' number | 'L' number | 'R' number | 'S'
 *   number   :=  positive decimal, fraction allowed (e.g. 100, 90.5)
 *   seq      :=  0..255, wrapping
 *
 * No whitespace is permitted anywhere.
 *
 * The sequence number is reported, not judged: gap detection and
 * duplicate suppression need link state, which belongs to the caller.
 * A frame without one is always executed, so hand-typed frames stay
 * usable for bench testing.
 */

typedef enum {
	COMMAND_FORWARD,	/* F<mm>  */
	COMMAND_BACKWARD,	/* B<mm>  */
	COMMAND_LEFT,		/* L<deg> */
	COMMAND_RIGHT,		/* R<deg> */
	COMMAND_STOP,		/* S      */
} CommandType;

typedef struct {
	CommandType type;
	/*
	 * Positive magnitude. Meaningless for COMMAND_STOP.
	 */
	float param;
} Command;

typedef enum {
	COMMANDPARSER_OK = 0,
	COMMANDPARSER_ERROR_EMPTY,
	COMMANDPARSER_ERROR_BAD_COMMAND,
	COMMANDPARSER_ERROR_BAD_SEQ,
	COMMANDPARSER_ERROR_TOO_MANY,
} CommandParserStatus;

typedef struct {
	Command *commands;
	size_t capacity;

	/*
	 * Populated only on COMMANDPARSER_OK.
	 */
	size_t count;

	uint8_t seq;
	bool hasSeq;
} CommandFrame;

/**
 * Parses one NUL-terminated frame.
 *
 * The caller sets out->commands and out->capacity; everything else
 * is written here. On any error out->count is 0 and out->commands
 * must not be used: frames are accepted or rejected atomically,
 * never partially.
 */
CommandParserStatus CommandParser_ParseFrame(
		const char *frame,
		CommandFrame *out);

/**
 * Short human-readable name for a status, for diagnostics.
 */
const char *CommandParser_StatusName(CommandParserStatus status);

#endif /* INC_COMMANDPARSER_H_ */
