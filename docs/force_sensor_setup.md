# ATI Axia80 F/T Sensor — Setup (this machine)

How to make the ATI Axia80-M20 force/torque sensor reachable on **this** machine
(Pop!_OS/Linux), and how to fill in its calibration. The human-trial (Windows)
machine already reaches the sensor and needs none of this. If the sensor is
unreachable, the recording scripts print a `[ForceTorque]` diagnostic pointing
here and continue with `None` force fields (they never crash).

Sensor identity: NET-Axia80-M20, S/N FT76858, MAC `00:16:BD:00:5D:AA`.
Protocol: RDT over **UDP port 49152**. Default IP on a direct point-to-point
link (no DHCP server): **`192.168.1.1`**, subnet `255.255.255.0`.

---

## 1. The problem on this machine (confirmed)

The sensor plugs into a USB-to-Ethernet adapter that enumerates as interface
**`enx00e04c6807dc`**. Right now:

- `ip -brief addr show enx00e04c6807dc` → link is **UP but has no `inet` address**.
- `nmcli device status` → `enx00e04c6807dc  ethernet  disconnected`.
- `ping -c1 192.168.1.1` → 0 received.

So the interface is physically up but **unconfigured** — it has no IP on the
sensor's subnet, so packets to `192.168.1.1` have nowhere to go. This is a
network-config problem, not a cable/hub fault. (The other Ethernet interface,
`enx6c6e072d3ca1` @ `192.168.123.x`, is the **Unitree G1 robot** — leave it alone.)

## 2. Fix A — quick, non-persistent (needs sudo)

Assigns a static IP on the sensor's subnet for this session only (lost on
reboot / replug):

```bash
sudo ip addr add 192.168.1.100/24 dev enx00e04c6807dc
sudo ip link set enx00e04c6807dc up
```

Verify:
```bash
ping -c3 192.168.1.1                 # should now get replies
python scripts/sensor_test.py        # should print live F/T readings
```
Also check the sensor's **L/A LED** is solid/blinking green (valid Ethernet link).

## 3. Fix B — persistent, via NetworkManager (needs sudo/polkit once)

`nmcli` is installed here. Create a manual-IP profile bound to this adapter's
MAC (survives reboots and name changes):

```bash
sudo nmcli connection add type ethernet con-name ati-ft ifname enx00e04c6807dc \
    ipv4.method manual ipv4.addresses 192.168.1.100/24
sudo nmcli connection up ati-ft
```
To remove later: `sudo nmcli connection delete ati-ft`.

Once an admin creates this profile **once**, NetworkManager re-applies it
automatically on every plug-in — no further sudo needed for day-to-day use.

## 4. If sudo is NOT available on your account

Everything above needs root. Hand your admin exactly this and ask them to run
Fix B (preferred — persistent, one-time):

```bash
sudo nmcli connection add type ethernet con-name ati-ft ifname enx00e04c6807dc \
    ipv4.method manual ipv4.addresses 192.168.1.100/24
sudo nmcli connection up ati-ft
```
- Interface: `enx00e04c6807dc` (the USB-Ethernet adapter for the ATI sensor;
  **not** `enx6c6e072d3ca1`, which is the G1 robot).
- Static host IP `192.168.1.100/24`; sensor is at `192.168.1.1`.
- No gateway/DNS needed — it's a point-to-point sensor link.
- After they run it once, you can verify with `ping 192.168.1.1` and
  `python scripts/sensor_test.py` without any sudo.

Meanwhile, both recording scripts still run without the sensor — force fields
are simply `None`. Use `--no-force-sensor` to skip the connect attempt entirely.

## 5. Alternative connection paths / things to try

- **Plug the USB-Ethernet adapter in directly, not through the USB hub.** This
  won't assign the IP by itself (that's still steps 2–4), but an unpowered/shared
  hub can cause flaky enumeration and drop UDP packets on the fast F/T stream —
  worth removing as a variable, especially before a real recording session.
- If the interface **name changes** after a replug/reboot, re-check with
  `ip -brief addr show` and substitute the new `enx…` name (Fix B binds by MAC,
  so it's immune to this once created).
- If the sensor is on a network **with** a DHCP server (not a direct link), it
  may take a DHCP address instead of `192.168.1.1`. `arp-scan` isn't installed
  here; install it (`sudo apt install arp-scan`) then
  `sudo arp-scan --interface=enx00e04c6807dc --localnet | grep -i 00:16:bd`, and
  set the discovered IP in `configs/default.yaml`'s `force_sensor.ip`.

## 6. Calibration — filling in cpf / cpt (manual)

`configs/default.yaml`'s `force_sensor.cpf` / `cpt` are **placeholder 1000000**.
The sensor reports raw int32 counts; real force/torque = counts / cpf, / cpt.
Get the true per-sensor values once the network link is up (steps 2–4):

- **Web UI (easiest):** open `http://192.168.1.1/config.htm` → "F/T
  Configurations" shows Counts per Force / Counts per Torque directly.
- **Calibration XML:** `http://192.168.1.1/netftcalapi.xml` (not `netftapi2.xml`).

Put those numbers into `force_sensor.cpf` / `force_sensor.cpt` and re-run
`python scripts/sensor_test.py` — readings should now be in real N / N·m (e.g.
the at-rest reading ≈ the weight of whatever hangs below the sensor).

## 7. Gravity reference (recorded automatically)

No manual step. At the start of a `--record` run, the scripts capture a ~1 s
at-rest baseline (`force_sensor.baseline_frames`) as the gravity reference —
**have the neck device NOT pulling at that moment** (startup pose ≈ pure
gravity). The pickle then stores both the raw wrench and the gravity-cancelled
wrench. Convention details: `docs/data_recording_conventions.md`.
