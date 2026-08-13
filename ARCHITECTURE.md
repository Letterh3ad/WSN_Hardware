# WSN System Architecture

Energy-efficient wireless sensor network for environmental and agricultural monitoring. Single-node deployment with a sensing node, a base station, and a laptop control client. This document records the full system as extended to include dual INA219 power monitoring and a persistent, SSH-controllable sleep interval.

## 1. Topology

```
  ESP32 node                 LoRa 868 MHz              Jetson Orin Nano             Laptop
  (battery + solar)         SF7 BW125 CR4/5            base station                (over SSH)
 +-----------------+                                +--------------------+       +-----------+
 | sensors         |  ---- data packet ---------->  | radio_loop (SPI)   |       | wsn_laptop|
 | 2x INA219       |                                | SQLite wsn_data.db |  ssh  |  live     |
 | SX1262 (HSPI)   |  <--- ack "sleep:<s>" -------  | Flask dashboard    | <---> |  export   |
 | deep sleep      |                                | CLI flags          |       |  set-int  |
 +-----------------+                                +--------------------+       +-----------+
```

Only the Jetson touches the radio. The laptop never connects to the ESP32 or the SX1262 directly; it drives the Jetson's CLI flags over SSH. The radio receive loop is a single long-lived process; every laptop action is a short-lived process that only reads or writes SQLite.

## 2. Node (ESP32) — `wsn_node.ino`

### Wake cycle

Each deep-sleep wake re-runs `setup()` from scratch (RAM is lost; only RTC memory persists):

1. Restore GPIO, init radio, power the switched sensor rail (`switchOn`, high-side switch on GPIO23, held LOW during sleep).
2. Read sensors: CO2, DHT22, AS7341, RS485 soil, then both INA219 power monitors.
3. Build the payload string, transmit over LoRa.
4. Open a short RX window (`ACK_WINDOW_MS`, 3 s) and listen for `sleep:<seconds>`. Exit the moment the ack lands.
5. Apply the ack to the RTC-persisted sleep interval if in range; otherwise keep the last value.
6. Put the radio to sleep, hold the switch pin LOW, enter deep sleep for the current interval.

### Sensor and radio pins (unchanged)

| Function | Pin | Function | Pin |
|---|---|---|---|
| MH-Z19 RX/TX | 13 / 14 | LoRa CS | 15 |
| DHT22 | 4 | LoRa CLK | 18 |
| RS485 RX/TX/DE_RE | 25 / 26 / 27 | LoRa MOSI/MISO | 5 / 19 |
| High-side switch | 23 | LoRa RESET | 17 |
| | | LoRa BUSY/DIO1 | 34 / 35 |
| | | LoRa RXEN/TXEN | 33 / 32 |

### Power monitors (new)

Two INA219 on the same I2C bus as the AS7341:

| Monitor | I2C address | Wired between | Measures |
|---|---|---|---|
| Solar | 0x40 | Harvester output and battery | Charge into the battery |
| Load | 0x41 | Buck-boost output and node circuit | Node's own draw |

Both read bus voltage (V), current (mA), and power (mW) each cycle. Default library calibration is 32 V / 2 A; switch to `setCalibration_16V_400mA()` for finer resolution if both currents stay below 400 mA. Addresses assume solar has A0/A1 unstrapped and load has A0 tied high — swap `INA_SOLAR_ADDR` / `INA_LOAD_ADDR` if your strapping differs.

### Sleep interval persistence

`sleepIntervalS` lives in RTC memory (`RTC_DATA_ATTR`), so it survives deep sleep but resets to `DEFAULT_SLEEP_S` (10 s) on a cold boot. The node adopts a new value only from a valid ack (5 s to 86400 s); a missing or malformed ack leaves the last good value in place.

## 3. Payload format

Comma-separated `key:value` pairs. A field is present only if that sensor read succeeded, so the base station must treat every field as optional.

| Key | Field | Key | Field | Key | Field |
|---|---|---|---|---|---|
| `co2` | CO2 ppm | `sm` | Soil moisture % | `sv` | Solar bus V |
| `t` | Temp C | `st` | Soil temp C | `si` | Solar current mA |
| `h` | Humidity % | `ec` | Soil EC | `sp` | Solar power mW |
| `clear` | Light clear | `ph` | Soil pH | `lv` | Load bus V |
| `nir` | Light NIR | `n` `p` `k` | Soil NPK | `li` | Load current mA |
| | | | | `lp` | Load power mW |

Example:
```
co2:412,t:23.4,h:55.2,clear:1234,nir:567,sm:34.5,st:23.4,ec:1234,ph:6.5,n:123,p:45,k:678,sv:5.012,si:250.5,sp:1032.4,lv:3.987,li:85.2,lp:339.7
```

The ack from base station to node is a single field: `sleep:<seconds>`.

## 4. Base station (Jetson) — `wsn_dashboard.py`

### Processes

- **Radio loop** (long-lived, holds SPI/GPIO): arms a 5-minute RX window, decodes each packet, acks with the current interval, then logs to SQLite. Run under systemd or a persistent session.
- **Flask dashboard** (same process as the radio loop): serves the live web view and `/api/*` endpoints.
- **CLI flags** (short-lived, DB-only, no SPI/GPIO): safe to invoke over SSH concurrently with the radio loop.

The radio loop sends the ack **before** the SQLite write. The node's ack window is short, so the write's latency is kept off the critical path, minimising the node's radio-on time.

### SQLite schema (`wsn_data.db`, WAL mode)

`readings` — one row per received packet:

```
id, ts, co2, temp, hum, clear, nir,
soil_moisture, soil_temp, soil_ec, soil_ph, soil_n, soil_p, soil_k,
rssi, snr,
solar_v, solar_i, solar_p, load_v, load_i, load_p, charging
```

`config` — persistent key/value:

```
key TEXT PRIMARY KEY, value TEXT      # holds sleep_interval_s
```

The sleep interval lives here, not in process memory, so it survives restarts and is shared between the radio loop, the dashboard, and the CLI. An older `wsn_data.db` is migrated automatically at startup (`ALTER TABLE` adds the power columns; the config table and interval are seeded if absent).

### Charging state — single source of truth

`compute_charging(solar_i)` is the only place the charging flag is derived. It is computed once at insert time and stored in the `charging` column. Every consumer (dashboard, `--get-latest`, `--export-csv`, laptop client) reads the stored value, so realtime and historical views can never disagree.

```
charging = (solar_i * SOLAR_CURRENT_SIGN) > CHARGING_THRESHOLD_MA
```

- `SOLAR_CURRENT_SIGN` (default `1`): flip to `-1` if your shunt is wired so charging reads negative.
- `CHARGING_THRESHOLD_MA` (default `5`): deadband so noise near zero does not flicker the flag.

### CLI interface (callable over SSH)

| Command | Effect |
|---|---|
| `--get-latest` | Prints one JSON object: latest reading, both power monitors, charging, link status (last seen, age, rssi, snr), current interval. |
| `--export-csv [--limit N]` | Prints the full (or last N) readings table as CSV to stdout, including power columns and charging. |
| `--set-interval SECONDS` | Persists a new interval to `config` and prints a confirmation JSON. The node applies it on its next wake/ack. |

`--console` and the default dashboard mode are unchanged.

### HTTP endpoints

`/` dashboard, `/api/latest`, `/api/history?limit=N`, `/api/config` (GET/POST). The web dashboard now shows the six power fields and a charging tag alongside the existing sensor grid.

## 5. Laptop client — `wsn_laptop.py`

Single file, standard library only, uses the system `ssh` binary (key-based auth assumed). Nothing extra needs installing on the Jetson.

| Subcommand | Effect |
|---|---|
| `live [--interval S]` | Polls `--get-latest` and redraws a terminal view every S seconds. |
| `once` | Prints the latest reading once. |
| `export [--out FILE] [--limit N]` | Runs `--export-csv` remotely and saves the result locally. |
| `set-interval SECONDS` | Runs `--set-interval` remotely. |

Connection defaults (`--host`, `--user`, `--remote-python`, `--remote-script`) are overridable per invocation. The client displays the stored `charging` field verbatim in both live and exported views; it never recomputes it, guaranteeing parity with the base station.

Examples:
```
python3 wsn_laptop.py --host jetson.local --user faye live --interval 5
python3 wsn_laptop.py --host jetson.local --user faye export --out today.csv --limit 500
python3 wsn_laptop.py --host jetson.local --user faye set-interval 30
```

## 6. LoRa parameters (both ends must match)

| Parameter | Value |
|---|---|
| Frequency | 868.000 MHz (`FREQ_REG = 0x36,0x40,0x00,0x00`, 32 MHz TCXO) |
| Spreading factor | SF7 |
| Bandwidth | 125 kHz |
| Coding rate | 4/5 |
| Sync word | 0x1424 (private) |
| TCXO | DIO3, 1.8 V, ~5 ms startup |
| Regulator | DC-DC + LDO |
| PA / TX power | +14 dBm |

## 7. Deployment note

Run the radio loop as a systemd service so it survives SSH disconnects and reboots, for example a unit that runs `python3 /home/faye/wsn/wsn_dashboard.py` and restarts on failure. Because SQLite is in WAL mode with a busy timeout, the CLI flags can run concurrently over SSH without blocking the loop.

## 8. Open decision: INA219 duty cycle (flag)

The INA219s sit on the always-on I2C rail, so the hardware sees charging continuously. But the ESP32 only samples them when it wakes, so charging that happens mid-sleep is not captured — each reading is a snapshot at wake time, not an integral over the sleep interval. Options:

- **(a) Wake-time snapshots (implemented).** Lowest node energy; charging is sampled once per cycle. Adequate if you only need the instantaneous charge/discharge state and coarse trend.
- **(b) Shorter sleep interval.** Denser charging samples at the cost of more wake energy — self-defeating for the energy-characterisation goal if pushed too far.
- **(c) Always-on coulomb counter or a low-power companion MCU** integrating charge across the sleep interval, read by the ESP32 at wake. Most accurate energy accounting, at the cost of extra always-on quiescent draw.

For single-node energy characterisation, (a) plus a bench measurement of the sleep-interval quiescent draw (INA219s and other always-on rail loads) is usually enough to close the energy budget. Move to (c) only if in-field charge accounting during sleep becomes a requirement.
