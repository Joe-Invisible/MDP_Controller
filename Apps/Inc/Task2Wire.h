#ifndef INC_TASK2WIRE_H_
#define INC_TASK2WIRE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "Task2Route.h"

/*
 * Planning off the STM: one Task2Route_* call and everything it reads, and the route it leaves.
 * The same encoder and decoder run on the STM and on the planning host, so both sides agree
 * byte for byte. Fields are written one by one, little-endian, floats as their IEEE bits; the
 * report's reason travels as text. A trailing FNV-1a checksum covers the payload.
 */

#define TASK2_WIRE_FORMAT 2U
#define TASK2_WIRE_MAX_BYTES 1024U   /* Largest encoded request or reply. */
#define TASK2_WIRE_REASON_BYTES 40U

typedef enum {
    TASK2_WIRE_REPLAN_DEPARTURE = 1,
    TASK2_WIRE_REPLAN_IR_TRAVERSE,
    TASK2_WIRE_REPLAN_OBSTACLE2,
    TASK2_WIRE_REPLAN_PARALLEL_RETURN,
    TASK2_WIRE_REPLAN_PARKING_RETURN,
    TASK2_WIRE_CALL_END
} Task2WireCall;

typedef struct {
    uint8_t call;               /* Task2WireCall */
    uint8_t run;
    uint16_t generation;
    Task2Route route;           /* The route as the call would receive it. */
    Task2RouteConfig config;    /* The config argument (departure and traverse calls). */
    Task2RoutePose pose;
    bool hasIr;
    Task2RouteIRDeparture ir;
    float widthMm;              /* Width argument (edge and return calls). */
    uint32_t step;              /* Completed step argument (edge and return calls). */
} Task2WireRequest;

typedef struct {
    uint8_t run;
    uint16_t generation;
    bool ok;                    /* The planner call's result. */
    uint32_t checks;            /* Its whole planning work, for the record. */
    uint32_t request;           /* Task2Wire_RequestTag of the request it answers. */
    Task2Route route;           /* The route the call left. */
} Task2WireReply;

size_t Task2Wire_EncodeRequest(const Task2WireRequest *request, uint8_t *out, size_t capacity);
bool Task2Wire_DecodeRequest(const uint8_t *in, size_t length, Task2WireRequest *request);
size_t Task2Wire_EncodeReply(const Task2WireReply *reply, uint8_t *out, size_t capacity);
bool Task2Wire_DecodeReply(const uint8_t *in, size_t length, Task2WireReply *reply);

/* Identifies one exact request: its sealed checksum, which covers every field including the call. */
uint32_t Task2Wire_RequestTag(const uint8_t *request, size_t length);

/* Planning host: decode a request, run the planner call, encode its reply. 0 if the request is bad. */
size_t Task2Wire_Execute(const uint8_t *request, size_t length, uint8_t *reply, size_t capacity);

/* Lowercase hex; returns characters written (2 per byte) or bytes read, 0 on error. */
size_t Task2Wire_ToHex(const uint8_t *in, size_t length, char *out, size_t capacity);
size_t Task2Wire_FromHex(const char *in, size_t length, uint8_t *out, size_t capacity);

#endif
