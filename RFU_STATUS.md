# GBA Wireless Adapter (RFU) Mode — Status & Handoff

**Goal:** make the RP2040 link-cable adapter emulate the GBA Wireless Adapter
(AGB-015) so real GBAs running Pokémon FireRed/LeafGreen/Emerald can link over
the internet, relayed through Celio-Server. Firmware mode `0x05`
(`RfuWirelessModule`), ported from gpsp `rfu.c` + the pokefirered `librfu` decomp.

**State (2026-07-02 late): reverse phase redesigned (bit geometry HW-derived in
test #31), full-stack legacy audit applied (17 confirmed findings — resets,
closed rooms, devid stability, router hail flow), SD-line reset wired (GP4) —
awaiting HW test.** Firmware v2.4.0 (`gblink.rfu-v2.4.0.uf2`). The
"communication error when opening a trade" root cause was found: THREE stacked
defects in the reverse phase (see below), each individually masked the others'
fixes across HW tests #17-27. All three are fixed by construction against the
librfu contract. Rooms of up to 5 players (1 host + 4 joiners) are supported
end-to-end (firmware protocol, Celio-Server routing, web client).

## The three reverse-phase defects (all fixed)

The reverse phase = after a WAIT-class command (0x25/0x27/0x35/0x37) the GBA
becomes an externally-clocked SIO32 slave and the ADAPTER must clock event
words into it. Ground truth: `pokefirered/src/librfu_intr.c`
(`sio32intr_clock_master` state 3 arms the slave in the same IRQ that reads
the WAIT ACK; `sio32intr_clock_slave` runs the per-word handshake).

1. **SI driven on the wrong edge (primary).** The GBA-as-slave latches SI on
   the SC FALLING edge — data must be valid through the SC-high window
   (GBATEK: "SC=HIGH → read SI; SC=LOW → output next SO bit"; confirmed by the
   V0 bit-bang sweep rejection). The old bit loop drove SI *at* the falling
   edge (`out pins,1 side0` — data and edge in the same PIO cycle), a
   hold-time race. The GBA's own SO does the same thing (shifts on falling,
   MSB presented at idle once armed) — which the old loop's sample point also
   missed by one phase in some builds.
2. **Word-1 handshake deadlock.** An armed slave presents its `0x80000000`
   pre-load MSB, so its SO reads HIGH — the old program misread that as the
   "SO went high" handshake step and then waited (unboundedly) for SO-low,
   which never comes before word 1. The delivery only ever proceeded
   *because of* defect 3.
3. **SC glitches at the role handoff.** The master SM uses non-optional
   side-set on SC, so `pio_sm_exec`'d instructions drive SC from bit 12: the
   handoff exec'd a bare `jmp` (side 0) — a guaranteed SC low pulse — and the
   first delivery also glitched via the pindir flip with a low output latch.
   An armed GBA counts every such edge as a data bit: framing shifted one bit
   in BOTH directions (`rev=[0x00000000, 0x40000000]`, GBA receives
   `0x99660028>>1` → header ≠ 0x9966 → reject → deaf with SO high → its
   100.2 ms error timer → the in-game "communication error"). Glitch duration
   is CPU-speed, not clkdiv — which is why the 5× clock slowdown changed
   nothing.

## The redesign

- **`rfu_master32` is now a 6-instruction pure 32-bit exchanger** (pull, set
  x, `out` early in SC-high, `in` late in SC-high, falling-edge `jmp` loop,
  push). It never waits on a pin (cannot hang), raises no IRQ (read
  synchronously), and gives ~4 µs SI setup before each falling edge at
  ~144 kHz. Budget: WS2812 4 + slave 12 + master 6 = 22/32 slots on pio1.
- **The librfu inter-word handshake moved to C**
  (`RfuProtocolSection::runDelivery`): per word — clock 32 bits, drive SI LOW
  (its `handshake_wait(0)`), poll its SO HIGH (bounded 4 ms), drive SI HIGH
  (`handshake_wait(1)`). Between words there is NO observable "re-armed"
  level: the GBA drops SO for <1 µs between two SIOCNT writes under IME=0 and
  then presents the staged MSB (always 1) — so a fixed 250 µs settle (covers
  the worst-case TM0-proximity spin) gates the next word instead of an SO-low
  wait. Word 1 needs no dance (the GBA armed itself right after the ACK
  read; delivery is additionally gated on that ACK-read transfer,
  `m_waitAckRead`). Only after the FINAL word is SO-low sustained (state-8
  exit) — it is detected and the role swapped back ATOMICALLY under
  `irq_lock` (a late ISR in that window would let the GBA's next command —
  issued within tens of µs — clock into a disabled slave SM and bit-slip
  every word after; the old fixed 2 ms guard was itself a bus-fight bug).
  The takeover is also atomic with a comstate re-check, so a soft reset
  racing the delivery start can't be clobbered.
- **Glitch-free handoff** (`rfuLink_setRole`): seed the SC output latch HIGH
  (exec `set pins,0 side1`) *before* the pindir flip; only side-1 encodings
  are ever forced on the master SM.
- **Delivery gate** (`m_waitAckRead`): the WAIT ACK is staged in-band and the
  GBA clocks exactly ONE more master transfer to read it before dropping to
  slave; `pollWaitEvent` only fires after that transfer (flipping earlier
  would discard the staged ACK and fight the GBA for SC). Plus an SO-high
  pre-check (armed-slave signature) + 45 µs settle before word 1.
- **Removed:** the speculative client "empty RESP_DATA keepalive" (librfu
  arms NO timer between the WAIT ack and word 1 — librfu_intr.c:107,161 — so
  gpsp's plain 533 ms TIMEO is correct), the drive-sweep machinery, the
  deferred-handoff/`parkMasterIdle` path. `rfuLink_captureReverse` (the slow
  bit-bang SO capture) is kept as a diagnostic entry point.
- **Event framing (unchanged, verified):** header `0x9966LLCC` + LL payload
  words + `0x80000000` trailer, during which the GBA shifts out its ACK
  `0x996600A8`. The word-2 readback (`rev[]` telemetry) is the accept oracle:
  `rev=[0x80000000, 0x996600A8]` = the GBA accepted the DATA event.

## 5-player rooms (1 host + 4 joiners)

- **Firmware protocol** (`rfuProtocol.hpp`): peers keyed by broadcast devid
  (up to 4, TTL-evicted); CONNECT slot scan capped by SYSCFG maxPlayers bits
  16-17 (0=5 players … 3=2); HOST_SEND emitted ONCE (the relay fans out);
  client-side NET_DISCONNECT now devid-checked; CFGSTAT (0x15) implemented;
  0x35 treated as wait-class; SEND_DATA payload clamped.
- **Celio-Server** (`src/rfu1Router.ts` + `session.ts`): sessions accept a
  `maxMembers` (2–5) on `sessionCreate`; rooms (>2) route parsed RFU1 frames
  per ptype — BROADCAST → everyone (rate-capped 400 ms/sender), CONNECT_REQ →
  the devid's owner (serialized per target, so the requester-anonymous
  ACK/NACK routes back correctly), HOST_SEND → that host's connected clients,
  CLIENT_SEND/ACK → the sender's host, DISCONNECT → the addressed peer.
  Each receiver gets a re-sequenced 64-byte-chunk stream + replay map
  (`requestData`). Host-leave evicts the room; other members come and go.
  Legacy 2-member sessions (AW, gen1-3) never touch the router.
- **Celio-Client**: the wireless page creates rooms with `maxMembers: 5`,
  shows the roster (`Players: N/5`), tolerates member leaves, and gates on
  firmware ≥2.4.0. Joiners all use role-lock variant 2 (client); union-room
  visibility is a symmetric broadcast mesh, so client↔client trades inside
  the room also route (the connect direction is what's locked, host never
  connects out).

## HW test plan (next session)

1. Flash BOTH adapters with `gblink.rfu-v2.4.0.uf2`. Rebuild/relaunch
   Celio-Server locally (`PORT=3000`, see below) and rebuild the client.
2. Two-GBA trade first (host = Create, joiner = Join): enter Union Room,
   both see each other, joiner initiates the trade.
   - Success oracle (server `DBG` lines): `rev=[0x80000000 0x996600A8]`,
     `mt` climbing (2 per DATA event), `da=0`, `lr` stable, client ring shows
     `0x26` (RECV) after `0x27` (WAIT), `CLIENT_SEND` frames appear, then the
     host finalizes (host `mt` > 0) → trade completes.
   - If the GBA still rejects (`rev` word2 ≠ 0x996600A8): the readback now
     isolates drive vs sample unambiguously; capture with
     `rfuLink_captureReverse` (diagnostic hook kept) before touching timing.
3. Then 3-5 players: one Create + up to four Join. Everyone should see
   everyone in the Union Room; trades pair-wise; berry crush/AW-style 5P
   activities exercise the multi-client host path (RECV concat, per-slot
   masks).

## Build / flash / run

```
cd GBLink-Firmware && west build -b rpi_pico -s . -d build-wl
                            # UF2: GBLink-Firmware/build-wl/zephyr/zephyr.uf2
cd Celio-Client && node_modules/.bin/ng build --configuration development
cd Celio-Server && rm -f dist/server.js && \
  npx esbuild src/index.ts --bundle --outfile=dist/server.js --platform=node --format=esm --packages=external && \
  PORT=3000 node dist/server.js          # ALWAYS verify: grep -c RfuRoomRouter dist/server.js (stale-bundle gotcha)
```
**Stale-build trap (bit us 2026-07-02):** the old cross-tree build-wl caches
(GBLink-Firmware/build-wl configured against the -Wireless tree and vice
versa) had broken ninja header-dependency tracking — header edits (e.g. the
control.hpp version bump) did NOT recompile their includers, shipping a UF2
that mixed new and stale objects. Both were deleted and the canonical build is
now the in-tree one above. After any suspicious build, verify the binary:
`python3 -c "d=open('build-wl/zephyr/zephyr.bin','rb').read(); print(d.count(bytes([0x0F,2,4,0])))"`
must print 1 (the GetFirmwareInfo version bytes).
GBC (grey) cable required (full-duplex). Client dev `environment.ts` apiUrl =
`ws://localhost:3000`. Host tests: `make -C GBLink-Firmware/tests/host` (all
green as of this revision — note the committed test binaries had been stale;
always rebuild).

## Code map

Firmware (`src/`):
- `layers/linkLayer_rfu.c` — PIO substrate. `rfu_slave32` (command phase,
  HW-proven, untouched) and the new `rfu_master32` exchanger; glitch-free
  `rfuLink_setRole`; synchronous `rfuLink_masterExchange` /
  `rfuLink_masterDriveSi`; `rfuLink_captureReverse` diagnostic.
  Pins: `GP0=SC`, `GP1=GBA SO` (adapter input), `GP2=GBA SI` (adapter output).
- `sections/rfuProtocol.hpp` — `RfuCore` HLE (gpsp port): consume/produce,
  `pollWaitEvent` (+`m_waitAckRead` gate), devid-keyed peers, 4-client host.
- `sections/rfuProtocolSection.cpp` — section thread; `runDelivery` (the
  reverse-phase engine + librfu dance); telemetry frames 0x0E/0x0F/0x1D.
- `module/rfuWireless.{hpp,cpp}`, `control.hpp` — mode wiring, FW 2.4.0.

Telemetry: counters/rev frame is tag `0x2E` as of 2.4.0 (was 0x0F, which
collided with GetFirmwareInfo responses on the raw-data channel) + `rev=[w0
w1]` readback → web client → server log `DBG <id> <who>#<seat> com= wifi= mt=
we= da= lr= ring=[] rev=[]`; router logs `RFU <from>-><to> <PTYPE> devid=`
including synthesized DISCONNECTs (`router__-><id> DISCONNECT (synthesized)`)
emitted when a connected member leaves or reconnects with a stale slot.

### Superseded diagnosis docs
The pre-2026-07 sections of this file chronicled HW tests #1-27 and concluded
"a logic analyzer is required" — that conclusion is obsolete: the three-defect
interaction explained every observation (including why the 5× slowdown and
each single fix changed nothing). History preserved in git and in the memory
notes.
