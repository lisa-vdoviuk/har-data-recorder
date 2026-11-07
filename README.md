# HAR Project Data Recorder

Local web app for recording accelerometer + gyroscope data from several MbientLab MetaWear sensors (MetaMotion S / BMI270).

## Folder layout
```
metawear-recorder/
├── app.py
├── requirements.txt
└── static/
    └── index.html
```

## Run
```
pip install -r requirements.txt
python app.py
```
Open http://127.0.0.1:8000 (Bluetooth must be turned on).

## Usage
1. "Find sensors" (or add a MAC address manually) → "Connect". The sensor vibrates for ~2.5 s and blinks blue so you can tell which one it is.
2. Give it a label (right arm, waist, …). Labels are saved in `sensors.json`.
3. Enter the movement, duration (or "No limit" and press Stop), rates and ranges → "Start recording".
4. Default mode is "Keep on sensors, download later": record as many takes as you want back to back (e.g. 5 takes x 5 sensors), then press "Download all" in the "Waiting on sensors" panel. The long download is split into one CSV per take and sensor. Choose "Download now" to download after every take instead.
5. The progress panel shows each sensor waiting, transferring, and saving. Sensors transfer one at a time. Saved CSVs appear in the dashboard (download CSV, ZIP, delete).

## Output
`recordings/<timestamp>_<subject>_<activity>/` holds one CSV per sensor plus `meta.json`.

CSV columns (core schema, always present, in this order):
`subject_id, timestamp, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, activity`

- `timestamp` is Unix time in milliseconds, acc is in g, gyro is in deg/s.
- If you record only one channel, the other channel's columns stay empty so the schema never changes.
- With "Extra metadata columns" ticked (default) these are appended: `time_s, sensor_position, sensor_mac, acc_odr_hz, acc_range_g, gyro_odr_hz, gyro_range_dps, recording_id`.
- Accelerometer and gyroscope are merged into one row by time (if rates differ, the slower channel's last value is held).

## Notes
- Live preview streams ONE sensor at a time (25/50/100 Hz) and shows the measured rate, so you can see if Bluetooth drops samples. It is unavailable while recording.
- Battery level is read on connect and refreshed every 90 s (click it to refresh).
- The panel shows which sensor failed and whether it was waiting for memory, transferring, or saving. For a stalled transfer, reconnect the affected sensor while the dashboard is running, then press "Download all" again. If writing the CSV failed, fix the disk issue and retry in the same app session.
- MetaMotion S consumes log entries as it transfers them. Keep the app running until all CSVs are saved. It retains samples received during an interrupted transfer or a save failure in memory for the next attempt.
- Takes are separated by the pause between them (sensors do not sample while idle), so leave at least ~0.5 s between takes. "Discard take" ignores a bad take without breaking the split.
- Scanning uses `bleak` first (most reliable on Windows) and falls back to the MetaWear SDK's scanner. If scanning fails, check that Bluetooth is on, or add the sensor by MAC.
- Data is logged to the sensor's own memory (like your `record.py`), so Bluetooth bandwidth does not limit the sample rate. Memory is finite: "No limit" at 1600 Hz fills it quickly.
- Sensors start in parallel; `epoch_ms` comes from the computer clock, so files from different sensors can be aligned (typically within tens of ms).
- If your sensor has no vibration motor, only the LED blinks.
