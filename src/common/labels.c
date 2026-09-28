#include "labels.h"

const char *safety_state_label(safety_state_t s)
{
    switch (s)
    {
        case SAFETY_SAFE:      return "SAFE";
        case SAFETY_WARNING:   return "WARNING";
        case SAFETY_EMERGENCY: return "EMERGENCY";
        case SAFETY_FAULT:     return "FAULT";
        default:                return "?";
    }
}

const char *lane_position_label(lane_position_t p)
{
    switch (p)
    {
        case LANE_FAR_LEFT:     return "FAR_LEFT";
        case LANE_LEFT:         return "LEFT";
        case LANE_SLIGHT_LEFT:  return "SLIGHT_LEFT";
        case LANE_CENTER:       return "CENTER";
        case LANE_SLIGHT_RIGHT: return "SLIGHT_RIGHT";
        case LANE_RIGHT:        return "RIGHT";
        case LANE_FAR_RIGHT:    return "FAR_RIGHT";
        case LANE_LOST:         return "LOST";
        case LANE_UNKNOWN:
        default:                 return "UNKNOWN";
    }
}

const char *sensor_health_label(sensor_health_t h)
{
    switch (h)
    {
        case SENSOR_OK:       return "OK";
        case SENSOR_DEGRADED: return "DEGRADED";
        case SENSOR_FAULT:    return "FAULT";
        default:               return "?";
    }
}
