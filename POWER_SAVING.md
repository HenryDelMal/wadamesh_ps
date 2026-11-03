# Power saving

The `powersaving`, `powersaving on`, and `powersaving off` CLI commands control
Wadamesh's existing Battery saver setting. This uses the existing idle-loop
throttling and its screen, USB, Wi-Fi, BLE and mesh activity gates; it is not a
port of EasySkyMesh's ESP32 automatic power management. The Battery saver switch
in Settings controls this same setting. It persists.

RX power saving (RXPS) is separate and defaults to off, including on existing
installations. In the mesh CLI, use `get radio.rxps` to inspect requested and
actual status, `set radio.rxps balanced` to enable a profile, and
`set radio.rxps off` to return to continuous receive. The EasySkyMesh CLI's
other profile and timing options are also available. A setting persists across
reboot. Interrupt-driven SX1262 and LR1110 radios are supported; LR1121 and
polled-interrupt configurations report an error and stay in continuous receive.
The Radio settings page also offers Off, Balanced, Conservative, and Custom;
Custom lets you enter the receive-window and sleep durations in microseconds.

RXPS periodically sleeps the receiver. Reception depends on the sender's
preamble, radio parameters, board hardware and timing. The timing calculations
come from EasySkyMesh PowerSaving v18; they are not a guarantee of reception on
Wadamesh hardware. `overdrive` and `riskyWorkingMax` intentionally operate
outside the guarded timing profile and may lose packets. Before relying on this
firmware, compare packet reception against RXPS off, test send/receive after
wake and reboot, and measure idle current on each board. Keep RXPS off if
reliable reception is more important than battery life.

The PlatformIO build uses the vendored MeshCore source in
`vendor/meshcore-ps`, based on Wadamesh's pinned core, to preserve its receive
queue and radio locking. See that directory's README for source provenance.
