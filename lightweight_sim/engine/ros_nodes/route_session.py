"""Versioned routes using the existing uint64 Path.sequence field.

High 44 bits identify a simulator run; low 20 bits identify a local plan.
Version zero is reserved for the reference path. Context is latched JSON on
sim/context (std_msgs/String), paired with the reference by run_id.
"""
import json

VERSION_BITS = 20
VERSION_MASK = (1 << VERSION_BITS)-1


def encode_sequence(run_id, version=0):
    if not (0 <= version <= VERSION_MASK and 0 < run_id < (1 << 44)):
        raise ValueError("route sequence out of range")
    return (run_id << VERSION_BITS) | version


def decode_sequence(sequence):
    return int(sequence) >> VERSION_BITS, int(sequence) & VERSION_MASK


def parse_context(message):
    value = json.loads(message.data)
    if value["schema_version"] != 1 or value["run_id"] <= 0:
        raise ValueError("unsupported route context")
    return value
