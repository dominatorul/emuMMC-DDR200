#!/usr/bin/env python3
"""Select an alternate SD KIP slot in the official Atmosphère 1.11.2 Fusee."""

import argparse
import ctypes
from hashlib import sha256
from pathlib import Path
import struct


FUSEE_SHA256 = 'ca62e177e0f5e305e36b455d37f26b853fbe938fd2ff1e7538d4e373ce20a183'
STUB_LOAD = 0x40010000
LZ4_LITERAL_PAIR = 0xB4
PROGRAM_SIZE = 0x3C000
SLOT = b'sdmc:/atmosphere/emummc.kip\0'
PROFILES = {'matrix': b'matrix', 'known-good': b'goodfs', 'clock-trial': b'clocks'}


def loader_unpack(packed):
    """Model Fusee's backward relocation and overlapping raw LZ4 expansion."""
    guard = bytes(range(32))
    ram = bytearray(PROGRAM_SIZE) + guard
    source = PROGRAM_SIZE - len(packed)
    ram[source:PROGRAM_SIZE] = packed
    dest = 0

    def length(nibble):
        nonlocal source
        size = nibble
        if nibble == 15:
            while source < PROGRAM_SIZE:
                extra = ram[source]
                source += 1
                size += extra
                if extra != 255:
                    return size
            raise ValueError('Truncated LZ4 length')
        return size

    while source < PROGRAM_SIZE:
        token = ram[source]
        source += 1
        literals = length(token >> 4)
        if source + literals > PROGRAM_SIZE or dest + literals > len(ram):
            raise ValueError('LZ4 literals exceed the Fusee image')
        for _ in range(literals):
            ram[dest] = ram[source]
            source += 1
            dest += 1
        if source == PROGRAM_SIZE:
            break
        if source + 2 > PROGRAM_SIZE:
            raise ValueError('Truncated LZ4 offset')
        distance = struct.unpack_from('<H', ram, source)[0]
        source += 2
        matches = length(token & 15) + 4
        if distance > dest or dest + matches > len(ram):
            raise ValueError('LZ4 match exceeds the Fusee image')
        for _ in range(matches):
            ram[dest] = ram[dest - distance]
            dest += 1
    if ram[PROGRAM_SIZE:] != guard:
        raise ValueError('Fusee expansion modifies memory after the boot image')
    return bytes(ram[:PROGRAM_SIZE])


def payload_for(official, profile):
    if sha256(official).hexdigest() != FUSEE_SHA256:
        raise ValueError('Expected the official Atmosphère 1.11.2 Fusee payload')
    start_ptr, end_ptr = struct.unpack_from('<II', official, LZ4_LITERAL_PAIR)
    start, end = start_ptr - STUB_LOAD, end_ptr - STUB_LOAD
    lz4 = ctypes.CDLL('liblz4.so.1')
    lz4.LZ4_decompress_safe.argtypes = (ctypes.c_char_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_int)
    unpacked = ctypes.create_string_buffer(PROGRAM_SIZE)
    original = official[start:end]
    if lz4.LZ4_decompress_safe(original, unpacked, len(original), len(unpacked)) != len(unpacked):
        raise ValueError('Unexpected LZ4 block in official Fusee')
    program = unpacked.raw
    if loader_unpack(original) != program:
        raise ValueError('Official payload does not match the Fusee layout check')
    if program.count(SLOT) != 1:
        raise ValueError('Unexpected emuMMC override filename in Fusee')
    replacement = SLOT.replace(b'emummc', PROFILES[profile])
    program = program.replace(SLOT, replacement)

    lz4.LZ4_compress_HC.argtypes = (
        ctypes.c_char_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int,
    )
    packed = ctypes.create_string_buffer(end - start)
    size = lz4.LZ4_compress_HC(program, packed, len(program), len(packed), 12)
    if not size or loader_unpack(packed.raw[:size]) != program:
        raise ValueError('Modified payload cannot be expanded in the original Fusee layout')
    payload = bytearray(official)
    struct.pack_into('<I', payload, LZ4_LITERAL_PAIR + 4, start_ptr + size)
    payload[start:end] = packed.raw
    return bytes(payload)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('official', type=Path)
    parser.add_argument('profile', choices=PROFILES)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    if args.official.resolve() == args.output.resolve():
        parser.error('Choose a separate output for the profile payload')
    args.output.write_bytes(payload_for(args.official.read_bytes(), args.profile))
