# Migration Guide

## SDK Structure Changes

The SDK has been restructured into three independent packages:

### Before (Monolithic)
```python
from realman_whj import WHJMotor, KincoMotor, ZlgCanDriver
```

### After (Modular)
```python
from zlg_can import ZlgCanDriver, ZCANDeviceType
from realman_whj import WHJMotor
from kinco_motor import KincoMotor
```

## Package Breakdown

| Old Import | New Import | Package |
|------------|------------|---------|
| `realman_whj.ZlgCanDriver` | `zlg_can.ZlgCanDriver` | zlg_can |
| `realman_whj.ZCANDeviceType` | `zlg_can.ZCANDeviceType` | zlg_can |
| `realman_whj.CANInterface` | `zlg_can.CANInterface` | zlg_can |
| `realman_whj.WHJMotor` | `realman_whj.WHJMotor` | realman_whj |
| `realman_whj.KincoMotor` | `kinco_motor.KincoMotor` | kinco_motor |
| `realman_whj.KincoProtocol` | `kinco_motor.KincoProtocol` | kinco_motor |

## Installation Changes

### Install All (Recommended for mixed use)
```bash
cd python_sdk
pip install -e .
```

### Install Individual Packages
```bash
# CAN driver only
pip install -e src/zlg_can

# WHJ only (includes zlg_can)
pip install -e src/realman_whj

# Kinco only (includes zlg_can)
pip install -e src/kinco_motor
```

## Code Migration Examples

### WHJ Only Application

**Before:**
```python
from realman_whj import ZlgCanDriver, ZCANDeviceType, WHJMotor

can = ZlgCanDriver()
can.open(ZCANDeviceType.USBCANFD_MINI)
motor = WHJMotor(can, motor_id=7)
```

**After:**
```python
# Option 1: Import from realman_whj (re-exports)
from realman_whj import ZlgCanDriver, ZCANDeviceType, WHJMotor

# Option 2: Import CAN driver separately
from zlg_can import ZlgCanDriver, ZCANDeviceType
from realman_whj import WHJMotor

can = ZlgCanDriver()
can.open(ZCANDeviceType.USBCANFD_MINI)
motor = WHJMotor(can, motor_id=7)
```

### Kinco Only Application

**Before:**
```python
from realman_whj import ZlgCanDriver, ZCANDeviceType, KincoMotor

can = ZlgCanDriver()
can.open(ZCANDeviceType.USBCANFD_MINI)
motor = KincoMotor(can, node_id=1)
```

**After:**
```python
from zlg_can import ZlgCanDriver, ZCANDeviceType
from kinco_motor import KincoMotor

can = ZlgCanDriver()
can.open(ZCANDeviceType.USBCANFD_MINI)
motor = KincoMotor(can, node_id=1)
```

### Hybrid Application (WHJ + Kinco)

**Before:**
```python
from realman_whj import (
    ZlgCanDriver, ZCANDeviceType,
    WHJMotor, KincoMotor
)
```

**After:**
```python
from zlg_can import ZlgCanDriver, ZCANDeviceType
from realman_whj import WHJMotor
from kinco_motor import KincoMotor
```

## Breaking Changes

1. **Kinco moved to separate package**
   - `from realman_whj import KincoMotor` no longer works
   - Use `from kinco_motor import KincoMotor`

2. **CAN driver moved to zlg_can**
   - Still re-exported from realman_whj for backward compatibility
   - But recommended to import from zlg_can directly

3. **Configuration separation**
   - `KincoMotorConfig` moved to `kinco_motor` package
   - `WHJMotorConfig` remains in `realman_whj` package

## Benefits of New Structure

1. **Decoupled packages**: Use only what you need
2. **Independent versioning**: Each package can be updated separately
3. **Clear dependencies**: realman_whj and kinco_motor both depend on zlg_can
4. **Better organization**: WHJ and Kinco are separate products, now in separate packages
