# Changelog

## [1.1.0] - 2025-08-29

### Added
- Integrated **st-mem-bank-macro** to ease register reads/writes.
- Added missing register structs: `WhoAmI`, `TempOut`.

### Changed
- Moved register definitions into a dedicated module with improved documentation.
- Improved I2C address handling:
  - Fixed existing I2C addresses.
  - Added support for two additional I2C addresses (now supporting four in total).
- Added chunked read/write support based on the `if_add_inc` bit.
- Inverted logic of the data ready flag: now returns `1` when data is ready.
- Unified register type usage to `u8` for read/write functions.
- Removed `Result` from the constructor for simplification.
- Removed `SpiDevice` for a cleaner API.
- Optimized temperature threshold set functions by removing unnecessary reads.
- General code cleanup and formatting.

### Miscellaneous
- Bumped library version to `1.1.0`.
- Updated dependency: `st-mems-bus`.

---

**Contributor:**  
Gioele Fiorenza <gioele.fiorenza2000@gmail.com>
