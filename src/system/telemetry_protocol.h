#ifndef TELEMETRY_PROTOCOL_H
#define TELEMETRY_PROTOCOL_H

#include <stddef.h>

#include "telemetry_service.h"

/* Hand-rolled JSON serialization - deliberately NOT using an external
   JSON library, since none is confirmed available in this QNX image
   (item 55: "do not introduce Linux-only APIs without verifying QNX
   compatibility" applies just as much to a third-party dependency as
   to a libc call). Every string field written here is one of ours
   (robot_id, module names, decision_reason built by
   telemetry_service.c) - never raw user input - so a minimal escaper
   (json_escape() in the .c file, handling quote/backslash/control
   chars) is enough; this is not a general-purpose JSON writer.
 *
 * Row shape matches the suggested schema in item 40: scalar top-level
 * columns for what a dashboard would filter/sort by (robot_id,
 * sequence, safety_state, timestamps), with the rest grouped into
 * JSONB objects (sensors/motor/navigation/watchdog/recovery/latency)
 * so adding a field later doesn't require a schema migration.
 */

/* Serializes one telemetry_snapshot_t (system/telemetry_service.h)
   as a single JSON object suitable for a PostgREST single-row POST
   to a `robot_telemetry` table. Returns the number of bytes written
   (excluding the null terminator), or -1 if it wouldn't fit in
   `buf_size` (TELEMETRY_MAX_PAYLOAD_BYTES, config.h, is sized well
   above what this actually produces - see telemetry_service.c). */
int telemetry_protocol_build_snapshot_json(const telemetry_snapshot_t *snap,
                                            char *buf, size_t buf_size);

/* Serializes up to `count` telemetry_event_record_t entries as a JSON
   ARRAY, suitable for a single PostgREST bulk-insert POST to a
   `robot_events` table (item 40/45's event upload). Returns bytes
   written, or -1 if it wouldn't fit. */
int telemetry_protocol_build_events_json(const telemetry_event_record_t *events,
                                          int count, char *buf, size_t buf_size);

#endif /* TELEMETRY_PROTOCOL_H */
