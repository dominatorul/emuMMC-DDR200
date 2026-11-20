# Overlapped read-ahead design

The existing read-ahead command fills one 2 MiB window synchronously. A
second 2 MiB window would let the CPU copy from the verified window while
the SD controller fills the next sequential window. The SD controller would
still run one command at a time. This is a design proposal only.

## Buffer ownership and scheduling

- Keep one ready window and one filling window. Track each window's sector
  range, mapping generation, link configuration and qualification generation.
  Before kicking ADMA, require the existing sequential run-in, eye, margin and
  two-tap qualification. Use the same 2 MiB transfer size and write invalidation
  rules. A third request gets no additional speculative command while one is
  outstanding.
- Under the existing FS storage mutexes and in their existing acquisition
  order, prepare the descriptor chain, perform pre-DMA cache maintenance,
  establish a deadline and issue CMD18. Return after the kick so the next
  request can overlap with the transfer. Give the outstanding command
  exclusive ownership of its descriptor range, destination buffer and host
  command slot until completion or confirmed cancellation. No worker should
  hold the FS mutexes between requests.
- At the next storage entry point, reacquire the mutexes and poll the command
  first. Promote the filling window only after transfer-complete, a clean
  command/DMA status, the full block count, post-DMA cache maintenance and
  generation checks. A ready-window hit can copy while ADMA is still active.
  A read that needs the filling window can wait within the existing command
  budget. A miss, write, retune, mode change or native access that needs the
  host must drain or cancel the speculative command before issuing another.
  Enable the kick only after auditing that every path to the SD host observes
  this ownership rule, including native FS operations.

## Errors and sleep/wake

On timeout, CRC, ADMA error, partial completion or a mismatched generation,
discard the speculative window. Keep both of its mappings pinned until DMA
has stopped. Use the existing CMD12/CMD13 and CMD/DAT cleanup, count one
failed-fill strike, and honor the existing cooldown and fresh verification
requirement. If cleanup fails, propagate the existing storage error. Serve
the requested data through the normal synchronous path once the host is
available. A write first retires any overlapping in-flight fill, invalidates
overlapping ready data, and follows the existing write-error handling.

Suspend, native close, eject and SD address-space changes must block further
kicks, drain or cancel the command, and invalidate both windows and tap
qualification before handing the controller back. Unmap each data window
and descriptor mapping once after DMA is quiescent and before native close
destroys the address space. Tear-down on a failed handoff follows the same
ownership checks. At wake, wait for the existing link verification, then
create fresh mappings lazily in the current device address space. Keep the
boot-wide failed-fill strike count across the mapping change.

## Memory budget and measurements

Two page-aligned buffers use 4 MiB total, an increase of 2 MiB. Budget up to
one extra 4 KiB descriptor page, a small amount of per-window metadata and
another 2 MiB of device address space. Keep the current 2 MiB DMA command
size. A previous 4 MiB buffer trial blue-screened during Hekate boot. Before
assuming a second static window fits, compare linked segment sizes, loader
and FS memory headroom, alignment and address-space availability. If the
second allocation or mapping fails, keep synchronous read-ahead.

Measure the current pre/post-DMA maintenance costs from trace v28, CPU copy
time, time between sequential requests, command duration, sequential hit
rate and filesystem throughput. Compare those with overlap time, time spent
waiting or cancelling, wasted speculative bytes, and p95/p99 request latency.
Run interleaved reads and writes and long sleep/wake tests alongside the
sequential benchmark. Compare eye width, clock drops and DMA errors at the
same workload and watch for increased sustained card load. The projected
gain depends on how often useful transfers finish before the next request.
