# WSN Energy Harvesting — Hardware Implementation

Solar-powered wireless sensor node with LoRa 868 MHz backhaul. ESP32 sense-and-sleep node transmits environmental and power data to a Jetson Orin Nano base station; a laptop client drives the Jetson over SSH.

See [ARCHITECTURE.md](ARCHITECTURE.md) for full system documentation.

## Files

| File | Where it runs | Purpose |
|---|---|---|
| `wsn_node/wsn_node.ino` | ESP32 | Production firmware — sense, transmit, deep sleep |
| `wsn_node_sleeptest/wsn_node_sleeptest.ino` | ESP32 | Bench test variant — fixed 15s duty cycle, LED blink |
| `wsn_dashboard.py` | Jetson Orin Nano | LoRa receive loop, SQLite, Flask dashboard, CLI flags |
| `wsn_laptop.py` | Laptop | SSH client — live view, export, set-interval |
| `wsn_visualize.py` | Laptop | Quick plot of exported CSV data |
| `poweranal.py` | Laptop | Energy characterisation analysis |
| `wsn-dashboard.service` | Jetson | systemd unit for the dashboard process |

## Quick start

**Flash the node** — open `wsn_node/wsn_node.ino` in Arduino IDE and flash to ESP32.

**Run the base station** (Jetson):
```bash
python3 wsn_dashboard.py
```

**Laptop client**:
```bash
python3 wsn_laptop.py --host jetson.local --user live
python3 wsn_laptop.py --host jetson.local --user export --out today.csv
python3 wsn_laptop.py --host jetson.local --user set-interval 60
```

## Dependencies

**Base station / laptop** — standard library only for `wsn_laptop.py`; `wsn_dashboard.py` requires:
```
pip install flask pyserial RPi.GPIO  # adjust for Jetson GPIO library
```

**Node** — Arduino libraries: `MHZ19`, `DHT`, `Adafruit_AS7341`, `Adafruit_INA219`.
