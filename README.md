# UAV Flight Test Suite - Documentation & Interview Guide

## 📋 Overview

This test suite contains two critical test cases for UAV/drone flight testing:

1. **Battery Performance Test (TC-BAT-001)** - Analyzes power consumption in idle vs flight states
2. **Navigation Validation Test (TC-NAV-001)** - Validates waypoint navigation accuracy

## 🎯 Test Objectives

### Test Case 1: Battery Performance Test
**Purpose:** Measure and compare battery consumption between idle (armed but stationary) and active flight operations.

**Key Metrics:**
- Voltage decay rate
- Current draw (Amperes)
- Battery level percentage
- Power consumption differential

**Why This Matters:** Battery performance is critical for:
- Mission planning and flight time estimation
- Safety margins and emergency procedures
- Identifying power-hungry systems or anomalies
- Optimizing flight profiles for endurance

### Test Case 2: Navigation Validation Test
**Purpose:** Verify GPS-based waypoint navigation accuracy and system reliability.

**Key Metrics:**
- Horizontal position error (meters)
- GPS fix quality
- Satellite coverage
- Waypoint success rate

**Why This Matters:** Navigation accuracy is essential for:
- Autonomous flight operations
- Mission critical positioning
- Regulatory compliance (e.g., FAA Part 107)
- Safety in GPS-denied environment preparation

---

## 🏗️ Architecture & Design Patterns

### 1. **Fixture Pattern** (Test Setup/Teardown)
```python
@pytest.fixture(scope="function")
def drone_fixture():
    fixture = DroneTestFixture()
    fixture.setup_sitl()
    fixture.connect_vehicle()
    yield fixture
    fixture.cleanup()
```

**Why:** Ensures consistent test environment setup and proper resource cleanup. Each test gets a fresh SITL instance.

### 2. **SITL (Software-In-The-Loop) Simulation**
- Simulates drone flight without physical hardware
- Safe for development and testing
- Repeatable test conditions
- Cost-effective

### 3. **Data Collection Pattern**
- Real-time telemetry logging
- Structured data storage (pandas DataFrame)
- CSV export for post-analysis
- Visualization generation

### 4. **Phase-Based Test Execution**
Tests are broken into logical phases:
- Pre-flight checks
- Data collection
- Analysis
- Validation/Assertions

---

## 🚀 How to Run the Tests

### Installation
```bash
# Install all required dependencies
pip install dronekit dronekit-sitl pymavlink mavsdk pytest pandas numpy matplotlib

to upgrade MAVProxy pip install --upgrade pymavlink MAVProxy --user
```

### Running Tests


***Start Ardupilot STIL***
Linux (or WSL) 
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py --map --console --out=tcpin:0.0.0.0:5761

or 

Alternatively, if not starting in the ArduCopter directory, but at the base ardupilot directoty:


./sim_vehicle.py -v copter --console --map -w --out=tcpin:0.0.0.0:5760 / --out=tcpin:0.0.0.0:5761 (TCP Connection)
Note: for UDP the first IP address connects to Dronekit (5760) & the second IP address is for Mission Planner (5761).

or

./sim_vehicle.py -v copter --console --map -w --out=udp:127.0.0.1:14550 / --out=udp:172.30.230.1:14550 (UDP Connection)
Note: for UDP the first IP address connects to WSL & the second is your PCs IP address within WSL. (Find with in WSL with command ip addr show eth0)



then

Start Mission Planner &

**Run all tests:**
```bash
pytest test_drone_flight.py -v
```

**Run specific test:**
```bash
# Battery test only
pytest test_drone_flight.py -v -k battery

# Navigation test only
pytest test_drone_flight.py -v -k navigation
```

**Run with output capture:**
```bash
pytest test_drone_flight.py -v -s
```

---

## 📊 Test Outputs

### 1. Console Output
- Real-time test progress
- Telemetry data streams
- Test phase transitions
- Pass/fail assertions

### 2. CSV Data Files
- `battery_test_data_YYYYMMDD_HHMMSS.csv` - Battery telemetry
- `navigation_test_data_YYYYMMDD_HHMMSS.csv` - Navigation metrics

### 3. Visualization Plots
- `battery_analysis_YYYYMMDD_HHMMSS.png` - 4-panel battery analysis
- `navigation_analysis_YYYYMMDD_HHMMSS.png` - Navigation path and error plots

---

## 🔍 Understanding the Code

### Key Libraries & Their Roles

| Library | Purpose |
|---------|---------|
| **dronekit** | MAVLink communication protocol, vehicle control |
| **dronekit-sitl** | Software-in-the-loop simulator for testing |
| **pytest** | Test framework and fixture management |
| **pandas** | Data manipulation and analysis |
| **numpy** | Numerical computations (distances, coordinates) |
| **matplotlib** | Data visualization and plotting |

### Critical Functions

#### `arm_and_takeoff(target_altitude)`
- Pre-arm safety checks
- Mode switching (GUIDED mode)
- Arming sequence
- Altitude monitoring loop
- **Why:** Standard procedure for safe UAV takeoff

#### `simple_goto(location)`
- Sends navigation command to autopilot
- Non-blocking (allows monitoring)
- Uses GPS coordinates
- **Why:** Industry-standard waypoint navigation

#### GPS Coordinate Calculations
```python
# Convert meters to GPS degrees
target_lat = home_lat + (north_offset / 111320)
target_lon = home_lon + (east_offset / (111320 * cos(lat)))
```
**Why:** GPS uses degrees, but we think in meters. 1 degree ≈ 111.32 km at equator.

---

## 🎤 Interview Talking Points

### 1. **Test Design Philosophy**
"I designed these tests following aerospace industry best practices:
- Isolated test cases with independent fixtures
- Comprehensive data logging for traceability
- Clear pass/fail criteria
- Automated execution for CI/CD integration"

### 2. **Real-World Applications**
"In production, these tests would be part of:
- Pre-flight automated checks
- Regression testing after firmware updates
- Acceptance testing for new vehicles
- Continuous monitoring during flight test campaigns"

### 3. **Safety Considerations**
"Safety is paramount. Notice how:
- We verify GPS fix before flight
- Monitor battery levels continuously
- Set position error tolerances
- Include timeout protections
- Always have RTL (Return to Launch) capability"

### 4. **Scalability**
"This framework scales easily:
- Add new test cases as classes
- Parameterize tests with @pytest.mark.parametrize
- Run tests in parallel with pytest-xdist
- Integrate with Jenkins/GitHub Actions for CI/CD"

### 5. **Data-Driven Insights**
"The visualizations and CSV outputs enable:
- Post-flight analysis
- Trend analysis across multiple flights
- Regression detection
- Performance benchmarking"

---

## 🛠️ Common Issues & Solutions

### Issue 1: SITL Won't Start
**Solution:** Check firewall settings, ensure ports 5760-5763 are available

### Issue 2: GPS Fix Not Acquired
**Solution:** SITL needs time to initialize GPS. The test waits for fix automatically.

### Issue 3: Tests Timeout
**Solution:** Increase timeout values if running on slow hardware. SITL is CPU-intensive.

### Issue 4: Import Errors
**Solution:** Ensure all dependencies installed: `pip install -r requirements.txt`

---

## 📈 Expected Results

### Battery Test
- **Idle current:** 0-5A (minimal)
- **Flight current:** 10-30A (active flight)
- **Power increase:** 200-500% during flight
- **Voltage:** Should remain above minimum threshold (typically >10.5V for 3S)

### Navigation Test
- **Position accuracy:** <2 meters typical for GPS
- **GPS fix:** Type 3 (3D fix) required
- **Satellites:** 6+ visible
- **Success rate:** >75% of waypoints within tolerance

---

## 🎓 Advanced Concepts to Discuss

### 1. **MAVLink Protocol**
- Standard communication protocol for drones
- Message-based system
- Supports telemetry, commands, mission planning
- Used by ArduPilot, PX4, DJI

### 2. **Test Coverage**
"For production, I'd add:
- Wind resistance testing
- Sensor failure scenarios
- Communication loss recovery
- Geofence boundary testing
- Emergency landing procedures"

### 3. **Regulatory Compliance**
"These tests support FAA Part 107 requirements by:
- Validating pre-flight checks
- Documenting system performance
- Ensuring safety margins
- Maintaining flight logs"

### 4. **Continuous Integration**
"In a CI/CD pipeline:
- Tests run on every commit
- SITL enables hardware-less testing
- Results logged to dashboard
- Failed tests block deployment"

---

## 🔗 Further Resources

- **DroneKit Documentation:** http://python.dronekit.io/
- **MAVLink Protocol:** https://mavlink.io/
- **ArduPilot:** https://ardupilot.org/
- **Pytest Documentation:** https://docs.pytest.org/

---

## Notes
- ** When running the code, if there is an class error:
-    Class Parameters(collections.MutableMapping, HasObservers):
                     ^^^^^^^^^^^^^^^^^^^^^^^^^^
AttributeError: module 'collections' has no attribute 'MutableMapping'
-  There is an issue with dronekit using a depreciated collections library within its own library for Python versions up to 3.9.
   From Python 3.10+ You will have to follow one of the options below for a fix 

Fixes are as below:

- ** Option 1: Downgrade Python (Quick Fix) (python3.9 UAVFlightTestSuite.py -v)
- ** Option 2: Update dronekit Library (Recommended) (pip install dronekit --upgrade) or community maintained (pip install dronekit-python)
- ** Option 3: Use a Virtual Environment with Python 3.9 
- ** Option 4: Patch dronekit (Workaround Best Quick Fix) Add this monkey-patch at the very top of your UAVFlightTestSuite.py file, before the dronekit imports:

# PATCH: Fix for Python 3.10+ compatibility with dronekit
import collections
import collections.abc
for name in dir(collections.abc):
    if not hasattr(collections, name):
        setattr(collections, name, getattr(collections.abc, name))