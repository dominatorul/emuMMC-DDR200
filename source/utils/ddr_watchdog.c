/* SPDX-License-Identifier: GPL-2.0-only */
#include <stdatomic.h>
#include <stddef.h>
#include "ddr_watchdog.h"
#include "fatal.h"

#define OP_TIMEOUT_US 15000000u
#define BUTTON_HOLD_US 3000000u
#define SUSPEND_GAP_US 2000000u
#define POLL_NS 100000000
#define VOLUME_UP_PIN (1u << 6)
#define VOLUME_DOWN_PIN (1u << 7)
#define VOLUME_PINS (VOLUME_UP_PIN | VOLUME_DOWN_PIN)

enum { SlotCount = 32, NoSlot = SlotCount };
typedef struct {
    _Atomic uintptr_t owner;
    _Atomic u32 depth, operation, phase;
    _Atomic u64 started;
} op_slot_t;

typedef struct {
    u64 operation, phase, age, depth;
} op_snapshot_t;

static op_slot_t slots[SlotCount];
static _Alignas(16) u8 watchdog_stack[0x4000];
static _Atomic(volatile const u32 *) button_input;

bool ddr_watchdog_volume_down(void)
{
    const volatile u32 *input = atomic_load_explicit(&button_input, memory_order_acquire);
    return input && ((*input & VOLUME_PINS) == VOLUME_UP_PIN);
}

static u32 find_slot(uintptr_t thread)
{
    for (u32 i = 0; i < SlotCount; ++i)
        if (atomic_load_explicit(&slots[i].owner, memory_order_acquire) == thread)
            return i;
    return NoSlot;
}

u32 ddr_watchdog_enter(uintptr_t thread, enum DdrOperation op, u32 mmc_id)
{
    if (!thread)
        return NoSlot;
    u32 i = find_slot(thread);
    if (i != NoSlot) {
        atomic_fetch_add_explicit(&slots[i].depth, 1, memory_order_release);
        return i;
    }
    for (i = 0; i < SlotCount; ++i) {
        uintptr_t empty = 0;
        if (atomic_compare_exchange_strong(&slots[i].owner, &empty, thread)) {
            atomic_store_explicit(&slots[i].operation, ((u32)op << 16) | mmc_id, memory_order_relaxed);
            atomic_store_explicit(&slots[i].phase, DdrOpEntered, memory_order_relaxed);
            atomic_store_explicit(&slots[i].started, get_tmr_us(), memory_order_relaxed);
            atomic_store_explicit(&slots[i].depth, 1, memory_order_release);
            return i;
        }
    }
    return NoSlot;
}

void ddr_watchdog_phase(uintptr_t thread, enum DdrOpPhase phase)
{
    if (!thread)
        return;
    u32 i = find_slot(thread);
    if (i != NoSlot && atomic_load_explicit(&slots[i].depth, memory_order_relaxed) == 1)
        atomic_store_explicit(&slots[i].phase, phase, memory_order_relaxed);
}

void ddr_watchdog_leave(u32 i)
{
    if (i == NoSlot)
        return;
    if (atomic_fetch_sub_explicit(&slots[i].depth, 1, memory_order_acq_rel) == 1)
        atomic_store_explicit(&slots[i].owner, 0, memory_order_release);
}

static op_snapshot_t oldest_operation(u64 now)
{
    op_snapshot_t oldest = {0};
    for (u32 i = 0; i < SlotCount; ++i) {
        u32 depth = atomic_load_explicit(&slots[i].depth, memory_order_acquire);
        if (!depth)
            continue;
        uintptr_t owner = atomic_load_explicit(&slots[i].owner, memory_order_acquire);
        u64 started = atomic_load_explicit(&slots[i].started, memory_order_relaxed);
        op_snapshot_t current = {
            .operation = atomic_load_explicit(&slots[i].operation, memory_order_relaxed),
            .phase = atomic_load_explicit(&slots[i].phase, memory_order_relaxed),
            .depth = depth,
        };
        /* Discard a sample if the slot was released or recycled while read. */
        if (owner && started <= now && (now - started >= oldest.age) &&
            atomic_load_explicit(&slots[i].depth, memory_order_acquire) &&
            atomic_load_explicit(&slots[i].owner, memory_order_acquire) == owner &&
            atomic_load_explicit(&slots[i].started, memory_order_relaxed) == started) {
            current.age = now - started;
            oldest = current;
        }
    }
    return oldest;
}

static void ignore_suspend_gap(u64 last, u64 now)
{
    for (u32 i = 0; i < SlotCount; ++i) {
        if (!atomic_load_explicit(&slots[i].depth, memory_order_acquire))
            continue;
        u64 started = atomic_load_explicit(&slots[i].started, memory_order_relaxed);
        if (started <= last)
            atomic_compare_exchange_strong_explicit(&slots[i].started, &started,
                started + (now - last), memory_order_relaxed, memory_order_relaxed);
    }
}

static void watchdog_entry(void *unused)
{
    (void)unused;
    u64 last_poll = get_tmr_us(), pressed_at = 0;
    bool released = false;
    const volatile u32 *input = atomic_load_explicit(&button_input, memory_order_acquire);
    for (;;) {
        svcSleepThread(POLL_NS);
        u64 now = get_tmr_us();
        if (now - last_poll >= SUSPEND_GAP_US) {
            ignore_suspend_gap(last_poll, now);
            pressed_at = 0;
        }
        last_poll = now;

#ifdef EMUMMC_FORCE_DDR200
        if (oldest_operation(now).age >= OP_TIMEOUT_US)
            fatal_abort(Fatal_DdrStalled);
#endif

        /* Volume buttons X6/X7 are active low. Arm after both are released. */
        u32 volume = *input & VOLUME_PINS;
        if (volume) {
            if (volume == VOLUME_PINS)
                released = true;
            pressed_at = 0;
        } else if (released) {
            if (!pressed_at)
                pressed_at = now;
            if (now - pressed_at >= BUTTON_HOLD_US)
                fatal_abort(Fatal_DdrSnapshot);
        }
    }
}

void ddr_watchdog_start(volatile const u32 *buttons)
{
    Handle thread;
    atomic_store_explicit(&button_input, buttons, memory_order_release);
    Result result = svcCreateThread(&thread, watchdog_entry, NULL, watchdog_stack + sizeof(watchdog_stack), 0x10, 3);
    if (!result) {
        result = svcStartThread(thread);
        svcCloseHandle(thread);
    }
#ifdef EMUMMC_FORCE_DDR200
    if (result)
        fatal_abort(Fatal_DdrWatchdog);
#else
    (void)result;
#endif
}

void ddr_watchdog_report(u64 *out)
{
    op_snapshot_t op = oldest_operation(get_tmr_us());
    out[0] = op.operation;
    out[1] = op.phase;
    out[2] = op.age;
    out[3] = op.depth;
}
