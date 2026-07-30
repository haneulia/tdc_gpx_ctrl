# C08 Inclusive Face Boundary Contract v039

- Date: 2026-07-30
- Scope: Motor Decoder face geometry, Motor-to-Laser AXIS boundary, C08 column policy

## 1. Confirmed RTL contract

For total decoded states `N`, face center `C`, and half-width `H`, the Motor
Decoder derives the circular boundaries below.

```text
lower = (C - H + N) mod N
upper = (C + H) mod N
```

Both boundaries are active. A non-wrapping window uses
`lower <= position <= upper`; a wrapping window uses
`position >= lower OR position <= upper`.

```text
active position samples / face = 2H + 1
angular state intervals / face = 2H
```

`upper - 1` is therefore not an equivalent display correction. It changes the
contract to a half-open interval, removes one active position, and makes the
integer sample set asymmetric about `C`. It also needs a special case at
`H = 0`.

## 2. Direction-independent naming

The two XGUI values are counter coordinates, not temporal entry/exit events.
With an increasing counter the temporal exit is the inclusive upper boundary.
With a decreasing counter the temporal exit is the inclusive lower boundary.
The XGUI therefore presents them as `Lower boundary (inclusive)` and
`Upper boundary (inclusive)`.

Motor AXIS `TLAST` continues to identify the final active beat in the actual
travel direction. Laser `face_end` is derived from that `TLAST`, so no RTL
latency or firing behavior is changed by the XGUI clarification.

## 3. C08 comparison policy

The Motor Decoder active-window membership corresponds to C08
`inclusive endpoints` mode. The integrated system scan contract remains
`centered bins`, because scan columns describe accepted Laser/TDC samples, not
every position admitted by the Motor active gate. C08 therefore keeps
`centered bins` as the default system profile and labels `inclusive endpoints`
as the Motor-window comparison.

The two counts must not be substituted for one another:

```text
Motor active positions = 2H + 1
System scan columns     = configured centered-bin / scheduler contract
```

The C08 v025 regression compares the latter with `rtl_result.json`. Changing
the system default to endpoints correctly fails that regression with a
`Columns / Face` mismatch, which confirms that the distinction is executable
rather than documentation-only.

Example for `CPR=3600`, `x1`, `center=450`, `H=300`:

```text
lower = 150
upper = 750
active positions = 601
angular intervals = 600
```

At `0.2 optical degree/state`, the 600 intervals span 120 optical degrees.

## 4. Change boundary

This update changes XGUI/IP-XACT reference metadata and the C08 default
comparison policy only. It does not change VHDL generics, ports, CSR addresses,
the inclusive comparator, AXIS framing, or Laser scheduling RTL.
