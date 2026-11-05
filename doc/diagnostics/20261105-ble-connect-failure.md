# Bridge never connects to the AC200L — BLE name match fails, then watchdog sleeps it

Captured 5 Nov 2026 over USB serial (`/dev/cu.usbserial-020AB103`, CP2104 → ESP32 dev
board, 115200). Firmware on the device is local branch `ac200l-grid-current`
(`03ae721`). Read-only serial: the chip was reset into normal boot to capture the
banner; nothing was written to flash.

## Symptom as reported

The bridge "always falls over" and/or the Bluetti BLE link does, needing a physical
power cycle to recover. Home Assistant sees the unit unavailable.

## What the device actually does

Two independent boots (35 s and 120 s captures) show the same sequence:

1. Boots clean, joins Wi-Fi, gets `192.168.20.140`, MQTT connects to
   `192.168.0.200:1883`, subscribes to its three command topics.
2. Publishes `state/device_status`: `{"MQTTconnected":1, "BTconnected":0}`.
3. Performs the single 5-second BLE scan from `initBluetooth()`
   (`BTooth.cpp:66-77`) and prints every advertiser.
4. **Never prints a single `[BT]` line.** No `Forming a connection`, no
   `onConnect`. The scan result is discarded and nothing else happens.
5. Goes silent for the rest of the capture.

## Root cause 1 — the AC200L advertises no name, and the match requires one

`BTooth.cpp:58`:

```cpp
if (advertisedDevice->haveServiceUUID()
    && advertisedDevice->isAdvertisingService(serviceUUID)
    && (strcmp(advertisedDevice->getName().c_str(), settings.bluetti_device_id) == 0))
```

The unit is present in both scans, and identifiable:

```
[BLE] Advertised Device found: Name: , Address: 08:d1:f9:90:f3:8a,
      manufacturer data: 424c424c5545545449, serviceUUID: 0xff00
```

`424c424c5545545449` is ASCII `BLBLUETTI`; the service is `0xff00`, which is
`serviceUUID`. So two of the three conditions pass. The third fails:
**`Name:` is empty**, so `strcmp("", "AC200L…")` can never be zero, whatever the
stored device id is. This is not a case of a stale/typo'd id in EEPROM — an empty
advertised name cannot match any stored string.

Consequence: `doConnect` and `doScan` are only ever set inside that match branch
(`BTooth.cpp:60-62`). Both stay `false` forever.

## Root cause 2 — the scan is never repeated

`initBluetooth()` ends with `pBLEScan->start(5, false)` — one 5-second scan, once
per boot. The only re-scan path is `handleBluetooth()`'s `else if (doScan)`
(`BTooth.cpp:252`), and `doScan` is only set by a successful match. So if the unit
is not caught inside that one 5-second window *with its name present*, the bridge
never scans again for the rest of that boot.

## Root cause 3 — the watchdog turns that into a visible "crash"

`lastBTMessage` is initialised to 0 and only updated when polling a *connected*
device (`BTooth.cpp:239`). With no connection it never moves, so
`handleBluetooth()` (`BTooth.cpp:216-223`) fires after
`MAX_DISCONNECTED_TIME_UNTIL_REBOOT` = 5 minutes:

```cpp
Serial.println(F("[BT] disconnected over allowed limit, reboot device"));
#ifdef SLEEP_TIME_ON_BT_NOT_AVAIL
    esp_deep_sleep_start();      // config.h defines it as 2
#else
    ESP.restart();
#endif
```

`SLEEP_TIME_ON_BT_NOT_AVAIL` is defined (2), so it deep-sleeps rather than
restarting. The wake timer *is* armed, in `setup()` (`Bluetti_ESP32.ino:19-21`),
so this is a 2-minute sleep, not a permanent one: 5 min awake (only the first 5 s
of which does anything) → 2 min asleep → boot → repeat, roughly a 7-minute cycle.

That is why it reads as a dead/hung device that a replug "fixes": replugging does
not fix the BLE match, it only restarts the same cycle, and for the 5-second scan
window right after the replug the unit may be caught. It also explains the
operator's uncertainty over which end is failing — while asleep the board is
invisible for 2 minutes at a time.

## Confirmed in a full watchdog cycle (470 s capture)

`evidence/bluetti-cycle-470s.log` shows the complete chain in one trace — and it
contains a full wake cycle, so the sleep is directly evidenced rather than inferred:

```
  1  ets Jun  8 2016 ...            ← boot
  3  rst:0x1 (POWERON_RESET)
 52  Name: , Address: 08:d1:f9:90:f3:8a, mfr 424c424c5545545449, serviceUUID: 0xff00
 60  {"MQTTconnected":1, "BTconnected":0}
 61  {"MQTTconnected":1, "BTconnected":0}
 62  [BT] disconnected over allowed limit, reboot device      ← 5 min after boot
 63  ets Jun  8 2016 ...
 65  rst:0x5 (DEEPSLEEP_RESET)                                 ← woke from deep sleep
110  Name: , Address: 08:d1:f9:90:f3:8a, mfr 424c424c5545545449, serviceUUID: 0xff00
117  {"MQTTconnected":1, "BTconnected":0}
```

`rst:0x5 (DEEPSLEEP_RESET)` is the direct proof that the watchdog path taken was
`esp_deep_sleep_start()` and not `ESP.restart()`, and that the 2-minute timer wake
works — the board is never permanently dead, it just never connects. 32
advertisements captured across the cycle; the Bluetti appears in both scans with an
empty name; no `[BT] Forming a connection` line appears anywhere.

## What consumes this data

- **Home Assistant: yes, entirely through this bridge.** `~/unifi/scripts/ha-config/mqtt.yaml`
  defines flat MQTT sensors on `bluetti/AC200L2347001013180/state/*`
  (`total_battery_percent`, `pack_battery_percent`, powers), all with
  `expire_after: 600`. The `bluetti_bt` BLE integration is **disabled, not removed** as
  the rollback — and only one of the two may hold the AC200L's single BLE slot at a time.
- **The UDR7: no.** It appears in HA only as `binary_sensor.udr7_online`; nothing in the
  repo subscribes the Bluetti topics on the router or pushes SOC to it. UPS coverage for
  the UDR7 is still listed as a physical gap in `~/unifi/STATE.md`.

Because field topics are only published while polling a *connected* device, an
unconnected bridge means `sensor.ac200l_total_battery_percent` is never published at
all — not stale, absent.

## Fix applied (2026-11-05) and what it proved

`BTooth.cpp` now:

- accepts the old exact-name match **or** `service 0xff00` + a `BLBLUETTI`
  manufacturer payload + the address in `BLUETTI_BLE_ADDRESS` (config.h, local;
  documented in `config.sample.h` under `#ifdef`);
- rescans every `BLUETOOTH_SCAN_INTERVAL_IN_SECONDS` instead of once per boot,
  using the non-blocking `start(duration, cb)` overload so MQTT never stalls;
- reuses a single `BLEClient` instead of creating (and leaking) one per attempt;
- clears remote characteristic handles and resets both queues on disconnect, so
  a dropped link cannot be polled through a stale handle or pair a notification
  with the previous link's page/offset;
- keeps the 5-minute watchdog strictly as a last resort.

Flashed to app0 (`0x10000`) only, hash-verified. Bootloader, partition table,
otadata and the NVS partition holding the WiFi/MQTT config were untouched.

**Result on hardware:** identity matching now works, and the bridge rescans on a
~10 s cadence, repeatedly. The connection is still refused by the station:

```
[BT] matched power station at 08:d1:f9:90:f3:8a (advType=0 connectable=0 addrType=0)
[BT] - connect() failed: rc=574 () addrType=0
```

`advType=0` is `ADV_IND` — connectable undirected advertising — on a public
address that matches the configured value. `rc=574` is `0x23E` = HCI `0x3E`
`BLE_ERR_CONN_ESTABLISHMENT`: the CONNECT_IND went unanswered. Retrying the same
address as `BLE_ADDR_RANDOM` also failed, so the address type is not the cause.

Nothing on the Home Assistant side holds the slot: the `bluetti_bt` entities have
been frozen since 2026-10-29 18:25 (integration disabled) and the MQTT sensors since
2026-11-03 20:38.

So the peer is advertising connectably and ignoring both SCAN_REQ (which is why its
name is empty) and CONNECT_IND. That is the second half of the operator's own
symptom — "or the Bluetti BT does" — and it is not addressable from the host: the
station has to free its own BLE (app reconnect or a power cycle).

What this fix changes for that failure mode is the recovery: previously a
recovering station was never noticed, because the bridge had already stopped
scanning for good. Now it retries every ~10 s and will attach within one scan
cycle of the station coming back, with no reboot and no replug.

## Evidence files

- `evidence/bluetti-boot-1.log` — 35 s capture, 18 advertisers, Bluetti present
  with empty name, `BTconnected:0`, no `[BT]` lines.
- `evidence/bluetti-boot-2.log` — 120 s capture, 15 advertisers, same, still no
  `[BT]` lines after two minutes.

Both were taken with the board on the desk, USB-powered, Wi-Fi and MQTT up.

## Not established

- **Why the name is absent.** It may be that this unit only puts its name in a
  scan response (or only while its display is awake), or a Bluetti firmware change
  removed it. The device is advertising, so it is not simply "connected to the
  phone app" — a connected peripheral stops advertising altogether.
- **Independent confirmation from a second radio.** A CoreBluetooth scan from the
  Mac could not initialise (delegate never fires from a bare `swift` script), so
  the empty-name observation rests on the ESP32's own scans — two boots, same
  result — not on an external sniffer.
- **Whether the EEPROM device id is still `AC200L2347001013180`.** Not read
  (requires flash access); the question is moot while the advertised name is
  empty, since no value would match.

## Candidate minimal fix (not applied)

Match on identity the unit actually advertises, and keep scanning:

1. In the scan callback, accept a device that advertises `serviceUUID` **and**
   either matches the configured name (when a name is present) **or** is the sole
   candidate advertising that service. Optionally corroborate with the
   `BLBLUETTI` manufacturer prefix, which is stable and was observed in both
   scans.
2. Restart the scan on an interval instead of relying on `doScan` (which is only
   set after a successful match), so a missed 5-second window is retried.
3. Keep the 5-minute watchdog as the last resort, but it should not be the only
   recovery path.

Not applied: it is a firmware change to the live power-monitoring bridge and
requires a build (`pio`) plus a USB flash of the board currently on the desk.
