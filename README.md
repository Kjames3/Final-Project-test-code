# Final Project Test Code

## Layout

```
Final-Project-test-code/
├── README.md
├── requirements.txt
├── config/
│   └── robot.yaml          # serial port, hip motor IDs, per-side offsets/limits
├── src/
│   ├── drivers/
│   │   └── feetech_servo.py    # wraps scservo_sdk; set_angle/read_angle in radians
│   └── utils/
│       └── config.py       # YAML loader, platform-aware serial-port default
├── scripts/
│   ├── set_hip_angles.py       # command both hips to absolute angles
│   ├── test_feetech_motor.py   # cycle one hip through ±90° (bring-up)
│   └── assign_motor_id.py      # change a servo's bus ID (one servo at a time)
└── Documents/
```

## Install (once per Pi)

```bash
pip install -r requirements.txt
```

The Feetech Python SDK is published as **`ftservo-python-sdk`** (the import name `scservo_sdk` does not exist on PyPI under that name — `pip install scservo_sdk` will fail).

## First-time bring-up

Each Feetech servo ships with **ID 1**. To control two on one bus they need distinct IDs — do this once per servo, **with only one servo plugged in at a time**:

```bash
python scripts/assign_motor_id.py        # plug in left or right hip  
```

Then verify each one cycles correctly:

```bash
python scripts/test_feetech_motor.py     # prompts for port + ID, cycles ±90° until ENTER
```

The IDs the scripts assume live in [config/robot.yaml](config/robot.yaml) under `hips.left.id` / `hips.right.id` — change those if you used different numbers.

## Running

Plug both hip servos into the bus, then:

```bash
python scripts/set_hip_angles.py 0 0                  # both hips to center (radians)
python scripts/set_hip_angles.py 0.7854 -0.7854       # left +45°, right -45° (radians)
python scripts/set_hip_angles.py 45 -45 --deg         # same, in degrees
python scripts/set_hip_angles.py 0 0 --port /dev/ttyUSB1
```

Per-side direction signs, offsets, and joint limits also live in [config/robot.yaml](config/robot.yaml).

