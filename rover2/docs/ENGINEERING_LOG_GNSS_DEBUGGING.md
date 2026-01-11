# Engineering Log: GNSS Health Monitor Debugging Session

**Date**: December 26, 2025  
**System**: Rover1 GNSS Health Monitor Web Dashboard  
**Status**: ✅ RESOLVED - RTK status now working correctly

## Summary

Complex debugging session to fix GNSS health monitoring that appeared to have "all the ingredients" (satellites, RTCM corrections, accuracy data) but was incorrectly showing "NO_FIX" status in the web dashboard. Root cause was a combination of **message type mismatches**, **QoS incompatibilities**, **timing race conditions**, and **flawed fallback logic**.

---

## Issues Discovered & Solutions

### 1. **Wrong Message Type Import** 
**Problem**: Satellite count always showed 0
- Code imported `ublox_msgs.msg.NavSAT` (wrong package)
- Should have been `ublox_ubx_msgs.msg.UBXNavSat`

**Root Cause**: Package namespace confusion between old and new u-blox message packages

**Solution**: 
```python
# Wrong
from ublox_msgs.msg import NavSAT

# Correct  
from ublox_ubx_msgs.msg import UBXNavSat
```

### 2. **Incorrect Message Field Names**
**Problem**: UBXNavSat message structure was misunderstood
- Used `msg.svs` (wrong field name)
- Used generic flags instead of specific boolean

**Solution**: Use correct UBXNavSat fields:
```python
# Correct field names
satellites = msg.sv_info  # not msg.svs
used = sat.flags.sv_used  # boolean, not bit mask
```

### 3. **Numpy Array Truth Value Error**
**Problem**: `if not covariance` failed with numpy arrays
```
"The truth value of an array with more than one element is ambiguous"
```

**Root Cause**: Numpy arrays don't allow direct boolean evaluation

**Solution**:
```python
# Wrong
if not covariance:

# Correct
if covariance is None:
```

### 4. **QoS Mismatches** 
**Problem**: Subscriptions couldn't receive messages due to incompatible QoS

**Discovery**: Different topics have different QoS requirements:
- `/ubx_nav_sat`: RELIABLE + TRANSIENT_LOCAL
- `/gps/filtered`: RELIABLE + VOLATILE  
- `/fix`: BEST_EFFORT + VOLATILE

**Critical Rule**: 
- ✅ RELIABLE subscriber + RELIABLE publisher = Compatible
- ✅ BEST_EFFORT subscriber + BEST_EFFORT publisher = Compatible
- ❌ RELIABLE subscriber + BEST_EFFORT publisher = **INCOMPATIBLE**

**Solution**: Mixed QoS profiles per topic requirement

### 5. **Stale Satellite Data**
**Problem**: `/ubx_nav_sat` only had latched startup data, not active updates

**Solution**: Subscribe to `/ubx_nav_pvt` for real-time satellite count:
```python
# Active data source
self.sat_visible = msg.num_sv  # from UBXNavPVT
```

### 6. **Timing Race Condition** ⚠️ 
**Problem**: `/fix` publishes at 1 Hz but timeout was 2.0 seconds
- With 5 Hz health checks, timing drift caused intermittent failures
- Valid data appeared "stale" due to tight timing margins

**Solution**: Increase timeout to 3.0 seconds (3x message interval)

### 7. **Flawed Fallback Logic** 💥
**Problem**: `navsat_fallback_callback` only updated when primary was already stale
- Created race condition where fallback data was ignored
- Caused intermittent NO_FIX even with valid `/fix` data

**Solution**: Always update from fallback (like primary):
```python
def navsat_fallback_callback(self, msg: NavSatFix):
    # Always update - no conditional check
    self.last_navsat_msg = msg
    self.last_navsat_time = self.get_clock().now()
```

---

## Key Debugging Strategies That Worked

### 1. **Systematic Layer-by-Layer Approach**
- Started with obvious issues (imports, field names)
- Progressed to QoS compatibility 
- Finally discovered timing and logic issues

### 2. **Message Structure Investigation**
- Used `ros2 topic info` and message inspection
- Verified actual publisher QoS settings vs assumptions
- Checked field names in actual message definitions

### 3. **Timing Analysis**
- Calculated message intervals vs timeout windows
- Identified race conditions through timing math
- Applied safety margins (3x interval rule)

### 4. **QoS Compatibility Matrix**
- Systematically tested publisher/subscriber QoS combinations
- Created topic-specific QoS profiles
- Verified compatibility with `ros2 topic echo`

---

## Best Practices Learned

### **Message Type Integration**
1. **Always verify package namespaces** - don't assume similar names are compatible
2. **Check message field names** in actual `.msg` files, not documentation
3. **Test with real data** before assuming message structure

### **QoS Management**
1. **Use `ros2 topic info -v`** to check publisher QoS settings
2. **Create topic-specific QoS profiles** instead of one-size-fits-all
3. **Remember the compatibility matrix** - RELIABLE can't receive from BEST_EFFORT
4. **Test QoS compatibility** before assuming it works

### **Timing Design**
1. **Use 3x interval rule** for timeouts (not 2x)
2. **Account for timing jitter** in embedded systems
3. **Monitor actual vs expected message rates**
4. **Consider scheduling variations** with multiple subscribers

### **Fallback Logic**
1. **Fallback should always accept data** when available
2. **Don't add artificial restrictions** to fallback paths
3. **Test failover scenarios** thoroughly
4. **Keep fallback logic simple** - complex conditions create race conditions

### **Numpy Integration**
1. **Use explicit None checks** instead of truthiness tests
2. **Convert to lists** when needed for non-numpy operations
3. **Handle numpy arrays specially** in boolean contexts

---

## Anti-Patterns to Avoid

### ❌ **Assumption-Based Development**
- Assuming QoS compatibility without checking
- Assuming message field names from similar packages
- Assuming timing margins are adequate

### ❌ **Complex Fallback Logic**  
- Adding conditions to when fallback should work
- Race condition-prone "smart" fallback behavior
- Over-engineering simple backup mechanisms

### ❌ **Tight Timing Margins**
- Using 2x intervals for timeouts
- Ignoring jitter and scheduling variations  
- Not testing edge case timing scenarios

### ❌ **One-Size-Fits-All QoS**
- Using same QoS profile for different topic types
- Not checking actual publisher QoS requirements
- Guessing at compatibility instead of testing

---

## Tools That Saved Us

1. **`ros2 topic info -v`** - Revealed actual QoS settings
2. **`ros2 topic echo`** - Tested subscription compatibility  
3. **`journalctl -u service -f`** - Real-time service log monitoring
4. **Message field inspection** - Verified actual structure vs assumptions
5. **Timing analysis** - Mathematical approach to race condition detection

---

## Final Architecture

```
┌─ /gps/filtered (RELIABLE) ──→ reliable_qos ──→ Primary NavSat
├─ /fix (BEST_EFFORT) ───────→ sensor_qos ───→ Fallback NavSat  
├─ /ubx_nav_pvt (RELIABLE) ──→ ublox_qos ────→ Active Satellites
└─ /ubx_nav_sat (RELIABLE) ──→ ublox_qos ────→ Fallback Satellites
```

**Timing**: 3.0s timeout for 1 Hz topics  
**Logic**: Always update from any available source  
**QoS**: Topic-specific profiles for compatibility

---

## Success Metrics

- ✅ **Satellite count**: Now shows real-time data (was always 0)
- ✅ **RTK status**: Correctly shows RTK_FIXED/RTK_FLOAT/NO_FIX  
- ✅ **No timing glitches**: Stable status with 1 Hz data
- ✅ **Multi-source resilience**: Works with primary or fallback topics
- ✅ **Error elimination**: No more numpy truth value errors

**Result**: GNSS web dashboard now provides reliable real-time status for autonomous rover navigation! 🎯

---

*Engineering lesson: Complex systems require systematic debugging, proper QoS design, adequate timing margins, and simple fallback logic. Never assume - always verify!*