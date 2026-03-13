# Silabs BLE Channel Abstraction

This directory contains the BLE channel abstraction for Silabs platforms (EFR32, SiWx917) as part of the BLE Manager refactor.

## Contents (upstream / CSA)

- **BlePlatformTypes.h** – Platform-agnostic types: `BleConnectionHandle`, `BleEventType`, `BleEvent`, `BleAdvertisingParams`, `BleConnectionParams`, `BleEventCallback`.
- **BleChannel.h** – Abstract BLE channel interface. Implementations in this tree: `BleChannelMatterEfr32`, `BleChannelMatterSiWx` (Matter BLE only).
- **efr32/** – Matter BLE channel skeleton for EFR32.
- **SiWx/** – Matter BLE channel skeleton for SiWx917.

## Optional second channel (side channel hook)

The manager supports an **optional second channel** (e.g. for provisioning/CLI) without defining side-channel-specific types upstream:

- **In this repo (CSA):** Only the **hook** is defined. When BLEManagerImpl is integrated, it will hold `BleChannel* mSideChannel` and provide `InjectSideChannel(BleChannel*)`. The manager uses the injected channel **only** via the `BleChannel` interface (`Init`, `Shutdown`, `ParseEvent`, `CanHandleEvent`, etc.). No `BleSideChannel` base class, no `BleSideChannelAdvConfig`, and no side-channel-specific APIs exist in the upstream tree.
- **In matter_sdk:** The full side-channel abstraction (e.g. `BleSideChannel`, `BleSideChannelAdvConfig`, `BleSideChannelEfr32`, `BleSideChannelSiWx`) and all implementation details live in the SDK. The SDK implements a subclass of `BleChannel` (or a compatible interface) and injects it via `InjectSideChannel()`. This keeps Silabs-only side-channel behavior out of the CSA codebase, consistent with other Silabs-only features (e.g. WifiSleepManager placement).

## Build

- **ble** – Shared interface and types (headers); no side-channel headers.
- **ble-efr32** – EFR32 Matter BLE channel implementation.
- **ble-siwx** – SiWx917 Matter BLE channel implementation.
