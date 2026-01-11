# Rover1 Refactoring Roadmap
**Created:** 2025-12-29  
**Status:** Planning Phase - Post-MVP Consideration  
**Principle:** Zero Functionality Loss - Any refactor that breaks existing features is considered a failure

## Executive Summary

Current codebase is **85% production-quality, 15% placeholder** with excellent engineering practices. The main refactoring opportunities focus on **consolidating duplicate patterns** rather than removing necessary complexity. All refactors should preserve 100% of current functionality while improving maintainability and development velocity.

## Refactor Priority Matrix

### **IMMEDIATE (Pre-MVP) - Low Risk, High Value**
*Safe to do now without disrupting autonomous navigation development*

#### 1. Empty Package Cleanup
**Risk:** None | **Benefit:** High | **Time:** 15 minutes
- **Problem:** Empty `rover1_control/` and `rover1_nav/` directories cause confusion
- **Impact:** Zero functional change, cleaner project structure

#### 2. Environment Validation  
**Risk:** Very Low | **Benefit:** High | **Time:** 30 minutes
- **Problem:** Missing NTRIP credentials cause mysterious GPS failures
- **Impact:** Fail-fast on startup vs mysterious field failures

#### 3. I2C Bus Singleton (Optional - Higher Risk)
**Risk:** Medium | **Benefit:** High | **Time:** 2 hours + testing
- **Problem:** Multiple `smbus.SMBus(1)` instances may cause bus contention
- **Impact:** More reliable I2C communication
- **Caution:** Could break working motor/battery systems

### **POST-MVP (Phase 1) - After Autonomous Navigation Works**
*High impact refactors requiring comprehensive testing*

#### 4. Hardware Abstraction Layer
**Risk:** High | **Benefit:** Very High | **Time:** 1-2 days
- **Problem:** I2C logic duplicated across 3 drivers
- **Impact:** Single interface, easier testing, better error handling

#### 5. Configuration Management Unification  
**Risk:** Medium | **Benefit:** High | **Time:** 4 hours
- **Problem:** Environment variables and parameters scattered
- **Impact:** Single source of truth, better validation

### **POST-MVP (Phase 2) - After Field Testing**
*Lower priority optimizations based on real usage patterns*

#### 6. GNSS Health Monitor Simplification
**Risk:** Medium | **Benefit:** Medium | **Time:** 4 hours
- **Problem:** 537 lines handling edge cases that may never occur
- **Impact:** Easier maintenance, focus on core metrics

#### 7. Web Dashboard Architecture  
**Risk:** Medium | **Benefit:** Medium | **Time:** 6 hours
- **Problem:** Complex threading, dual servers
- **Impact:** Simpler deployment, better performance

#### 8. QoS Configuration Consolidation
**Risk:** Low | **Benefit:** Medium | **Time:** 2 hours
- **Problem:** QoS patterns duplicated across nodes
- **Impact:** Consistent topic handling

---

## Detailed Refactor Specifications

### **IMMEDIATE REFACTORS**

## 1. Empty Package Cleanup

### Investigation Prompt
```
Please analyze the rover1 project structure and identify empty or placeholder packages that can be safely removed. I want to clean up the project structure without affecting any working functionality.

Specifically check:
1. rover1_control/ directory contents and dependencies
2. rover1_nav/ directory contents and dependencies  
3. Any references to these packages in launch files, CMakeLists.txt, package.xml files
4. Whether removing them would break any existing functionality

Provide a step-by-step removal plan that ensures zero functional impact.
```

### Implementation Prompt
```
Please clean up the empty placeholder packages in the rover1 project. Remove:
1. rover1_control/ directory (if truly empty)
2. rover1_nav/ directory (if only contains empty config/)
3. Update any package.xml dependencies that reference these packages
4. Update any CMakeLists.txt files that reference these packages

Requirements:
- Preserve 100% of current functionality
- Test that all existing launch files still work
- Verify the system builds successfully after changes
- Document what was removed in the commit message

If you find ANY references that would break functionality, do not proceed with the removal.
```

## 2. Environment Validation

### Investigation Prompt
```
Please analyze how environment variables are currently handled across the rover1 project, specifically:

1. Map all environment variables used (NTRIP_*, ROVER_*, WIFI_*, etc.)
2. Identify where each is used (launch files, python files, shell scripts)
3. Determine which are required vs optional
4. Find current error handling when variables are missing
5. Identify opportunities to add validation at startup

Focus on the GPS/NTRIP credentials since missing credentials cause the most mysterious failures in the field.

Provide a design for early validation that fails fast with clear error messages.
```

### Implementation Prompt
```
Please implement environment variable validation for the rover1 project that fails fast on startup if required credentials are missing.

Requirements:
1. Create a validation function that checks all required environment variables
2. Add this validation to the main launch files (rover.launch.py, gps.launch.py)
3. Provide clear error messages indicating which variables are missing
4. Include the .env.example file path in error messages for user guidance
5. Make validation optional via a parameter (validate_env:=true by default)

Required variables to check:
- NTRIP_USER, NTRIP_PASS, NTRIP_HOST (critical for GPS)
- Any others you find that cause failures when missing

Implementation approach:
```python
def validate_environment():
    required_vars = {
        'NTRIP_USER': 'NTRIP username for RTK corrections',
        'NTRIP_PASS': 'NTRIP password for RTK corrections', 
        'NTRIP_HOST': 'NTRIP server hostname'
    }
    
    missing = []
    for var, desc in required_vars.items():
        if not os.getenv(var):
            missing.append(f"{var} ({desc})")
    
    if missing:
        raise EnvironmentError(
            f"Missing required environment variables:\n" + 
            "\n".join(f"  - {var}" for var in missing) +
            f"\n\nCopy .env.example to .env and set values."
        )
```

Test that GPS still works after implementation. Zero functionality loss is required.
```

## 3. I2C Bus Singleton (CAUTION)

### Investigation Prompt
```
Please analyze the I2C usage patterns in the rover1 hardware drivers to assess if multiple SMBus instances could cause bus contention issues.

Investigate:
1. How many places create smbus.SMBus(1) instances
2. Timing of I2C operations across drivers
3. Whether operations are properly synchronized
4. Any existing evidence of I2C conflicts in logs/code comments
5. Risk assessment of implementing a singleton pattern

Files to analyze:
- rover1_hardware/rover1_hardware/hiwonder_driver.py
- rover1_hardware/rover1_hardware/battery_monitor.py
- Any other I2C users

Provide a risk assessment and implementation strategy that preserves all current functionality. This is a CAUTION item - only recommend if there's clear evidence of problems.
```

### Implementation Prompt
```
⚠️ CAUTION: This refactor has medium risk of breaking working motor/battery systems.

Only proceed if investigation shows clear I2C contention issues.

Please implement an I2C bus singleton to prevent multiple SMBus instances, while preserving 100% of current functionality.

Requirements:
1. Create thread-safe singleton I2C bus manager
2. Update all I2C users to use the singleton
3. Preserve all current timing, error handling, and functionality
4. Add proper resource cleanup and error recovery
5. Maintain the same external interfaces for all drivers

Implementation pattern:
```python
import threading
import smbus2 as smbus

class I2CBusManager:
    _instance = None
    _lock = threading.Lock()
    
    def __new__(cls, bus_number=1):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
                    cls._instance.bus = smbus.SMBus(bus_number)
                    cls._instance.bus_lock = threading.Lock()
        return cls._instance
    
    def read_i2c_block_data(self, addr, reg, length):
        with self.bus_lock:
            return self.bus.read_i2c_block_data(addr, reg, length)
    
    def write_byte_data(self, addr, reg, value):
        with self.bus_lock:
            return self.bus.write_byte_data(addr, reg, value)
```

Critical testing required:
1. Motor control still works identically
2. Battery monitoring maintains same update rate
3. Encoder readings remain accurate
4. No performance degradation
5. Error handling preserved

If ANY functionality is lost, revert immediately.
```

---

### **POST-MVP PHASE 1 REFACTORS**

## 4. Hardware Abstraction Layer

### Investigation Prompt
```
Please analyze the current hardware driver architecture in rover1 to design a unified hardware abstraction layer.

Current state analysis:
1. Map all I2C operations in hiwonder_driver.py, battery_monitor.py
2. Document all register addresses, data formats, timing requirements
3. Identify common patterns and duplicate code
4. Map the complete hardware interface (motors, encoders, battery)
5. Analyze current error handling and recovery strategies

Design requirements:
- Single hardware interface that preserves ALL current functionality
- Support for the exact motor mapping currently working
- Maintain current safety features (watchdogs, error handling)
- Enable easier testing via hardware mocking
- Preserve performance characteristics

Provide a detailed design that shows how to consolidate without losing any capability.
```

### Implementation Prompt
```
Please implement a unified hardware abstraction layer for the Hiwonder motor controller HAT.

This is a HIGH RISK refactor - any loss of functionality is considered a failure.

Requirements:
1. Create HiwonderHAT class that encapsulates ALL I2C operations
2. Preserve exact motor mapping: FL(52), FR(54), RL(51), RR(53)
3. Maintain current encoder mapping and polarity corrections
4. Keep all safety features (watchdog timers, error handling)
5. Support all current parameters and configurations
6. Enable hardware mocking for unit tests

Implementation structure:
```python
class HiwonderHAT:
    """Unified interface to Hiwonder Motor Controller HAT"""
    
    def __init__(self, bus_number=1, address=0x34):
        self.bus = I2CBusManager(bus_number)
        self.address = address
        
        # Preserve current register mapping
        self.motor_registers = {
            'front_left': 52,
            'front_right': 54, 
            'rear_left': 51,
            'rear_right': 53
        }
        
        # Preserve current encoder mapping and polarity
        self.encoder_mapping = {
            0: ('rear_left', 1),    # Index 0: RL, no invert
            1: ('front_left', -1),  # Index 1: FL, invert 
            2: ('rear_right', -1),  # Index 2: RR, invert
            3: ('front_right', 1)   # Index 3: FR, no invert
        }
    
    def set_motor_speeds(self, fl, fr, rl, rr):
        """Set motor speeds while preserving current behavior"""
        # Implement exact current logic
        
    def get_wheel_encoders(self):
        """Read encoders with current mapping and polarity"""
        # Implement exact current logic
        
    def get_battery_voltage(self):
        """Read battery voltage in volts"""
        # Implement exact current logic
        
    def stop_all_motors(self):
        """Emergency stop with current safety behavior"""
        # Implement exact current logic
```

Refactor existing drivers to use this interface:
1. Update hiwonder_driver.py to use HiwonderHAT.set_motor_speeds()
2. Update battery_monitor.py to use HiwonderHAT.get_battery_voltage()  
3. Update kinematics to use HiwonderHAT.get_wheel_encoders()

Testing requirements:
1. Motor control identical to before (same speeds, directions)
2. Encoder readings identical (same values, timing)
3. Battery monitoring identical (same voltage readings, rate)
4. Safety systems work identically (watchdog, emergency stop)
5. Performance characteristics preserved

If ANY test fails, revert immediately and document the failure.
```

## 5. Configuration Management Unification

### Investigation Prompt
```
Please analyze how configuration is currently managed across the rover1 project to design a unified configuration system.

Current analysis:
1. Map all parameter declarations across ROS nodes
2. Document all environment variable usage
3. Identify configuration scattered across launch files
4. Find hardcoded values that should be configurable
5. Analyze current validation and error handling

Design a unified configuration system that:
- Centralizes all rover parameters
- Provides validation and default values
- Supports environment variable override
- Maintains backward compatibility with current launch files
- Enables easier deployment and field configuration

Show how this would work with current nodes without breaking functionality.
```

### Implementation Prompt
```
Please implement a unified configuration management system for rover1.

Requirements:
1. Create RoverConfig class that centralizes all parameters
2. Support environment variable overrides
3. Provide validation with clear error messages
4. Maintain full backward compatibility with existing launch files
5. Enable easier parameter tuning and deployment

Implementation structure:
```python
class RoverConfig:
    """Centralized rover configuration with validation"""
    
    def __init__(self):
        # Hardware parameters
        self.i2c_bus = int(os.getenv('I2C_BUS', 1))
        self.i2c_address = int(os.getenv('I2C_ADDRESS', 0x34))
        
        # Motor parameters  
        self.max_rpm = int(os.getenv('MAX_RPM', 200))
        self.pwm_limit = int(os.getenv('PWM_LIMIT', 100))
        self.wheel_radius = float(os.getenv('WHEEL_RADIUS', 0.04))
        
        # Safety parameters
        self.watchdog_timeout = float(os.getenv('WATCHDOG_TIMEOUT', 0.5))
        self.max_linear_speed = float(os.getenv('MAX_LINEAR_SPEED', 2.0))
        
        # GPS/NTRIP parameters
        self.ntrip_user = os.getenv('NTRIP_USER')
        self.ntrip_pass = os.getenv('NTRIP_PASS')
        self.ntrip_host = os.getenv('NTRIP_HOST', '165.206.203.10')
        
        # Validate critical parameters
        self.validate()
    
    def validate(self):
        """Validate configuration and fail fast on errors"""
        errors = []
        
        if not self.ntrip_user:
            errors.append("NTRIP_USER required for RTK corrections")
        if not self.ntrip_pass:
            errors.append("NTRIP_PASS required for RTK corrections")
            
        if self.wheel_radius <= 0:
            errors.append(f"Invalid wheel radius: {self.wheel_radius}")
            
        if errors:
            raise ValueError("Configuration errors:\n" + "\n".join(f"  - {e}" for e in errors))
    
    def to_ros_params(self):
        """Convert to ROS parameter dictionary"""
        return {
            'i2c_bus': self.i2c_bus,
            'i2c_address': self.i2c_address,
            'max_rpm': self.max_rpm,
            # ... all parameters
        }
```

Integration requirements:
1. Update launch files to use unified config
2. Update each ROS node to accept config parameters
3. Preserve all current parameter names and values
4. Maintain backward compatibility for existing deployments
5. Add config validation to main launch files

Zero functionality loss required. All current parameter values must work identically.
```

---

### **POST-MVP PHASE 2 REFACTORS**

## 6. GNSS Health Monitor Simplification

### Investigation Prompt
```
Please analyze the gnss_health_monitor_node.py to identify simplification opportunities based on real usage patterns.

Analysis focus:
1. Which message types and topics are actually used in practice
2. What QoS complexity is truly needed vs defensive programming
3. Which accuracy calculations and RTK state logic are essential
4. What fallback scenarios actually occur vs theoretical edge cases
5. Which parts of the 537-line implementation provide real value

Provide recommendations for simplifying to ~200 lines while preserving core functionality for autonomous navigation support.
```

### Implementation Prompt
```
Please simplify the GNSS health monitor while preserving all essential functionality for autonomous navigation.

Current: 537 lines with complex fallback logic
Target: ~200 lines focused on core use cases

Keep essential features:
1. Satellite count (visible/used)
2. RTK state detection (NO_FIX/DGPS/FLOAT/FIXED)  
3. NTRIP connection status
4. Horizontal accuracy reporting
5. Battery voltage integration

Simplify/remove:
1. Complex QoS fallback matrices (focus on working QoS)
2. Multiple message type support (use what actually works)
3. Unused accuracy calculations
4. Complex error handling for edge cases
5. Verbose logging and debug modes

Requirements:
1. Maintain the /gnss/health topic output format
2. Preserve Foxglove dashboard compatibility
3. Keep real-time update rates
4. Maintain essential error handling
5. Preserve battery monitoring integration

Implementation approach:
- Single QoS profile that works
- One primary topic subscription per data type
- Simplified RTK state logic
- Essential error handling only
- Clean, readable code structure

Test that Foxglove dashboard continues to work identically after simplification.
```

## 7. Web Dashboard Architecture Simplification

### Investigation Prompt
```
Please analyze the web dashboard architecture to identify simplification opportunities.

Current analysis:
1. Why are there separate HTTP (8080) and WebSocket (8081) servers?
2. Is the threading model (3 threads) necessary for the current load?
3. Should image encoding happen in the ROS node or web layer?
4. What are the actual performance requirements for the dashboard?
5. Could this be simplified to a single Flask app?

Design a simpler architecture that maintains all current functionality while reducing complexity.
```

### Implementation Prompt
```
Please simplify the web dashboard architecture while maintaining all current functionality.

Current: HTTP server + WebSocket server + complex threading
Target: Single Flask application with WebSocket support

Requirements:
1. Maintain live GNSS health data streaming
2. Preserve camera feed integration
3. Keep sub-second update rates for critical data
4. Support multiple simultaneous clients
5. Serve static files (HTML/CSS/JS)

Simplified architecture:
```python
from flask import Flask, render_template
from flask_socketio import SocketIO

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

class SimplifiedDashboard(Node):
    def __init__(self):
        super().__init__('web_dashboard')
        
        # Single Flask-SocketIO app handles everything
        self.app = app
        self.socketio = socketio
        
        # ROS subscriptions
        self.health_sub = self.create_subscription(...)
        self.image_sub = self.create_subscription(...)
        
    def health_callback(self, msg):
        # Broadcast via SocketIO
        self.socketio.emit('health_update', data)
        
    def image_callback(self, msg):  
        # Broadcast via SocketIO
        self.socketio.emit('image_update', base64_data)

@app.route('/')
def index():
    return render_template('index.html')

# Single server, single port, simplified deployment
```

Benefits:
1. Single port (8080) instead of two
2. No complex threading coordination
3. Standard Flask deployment patterns
4. Easier development and debugging
5. Simpler client-side JavaScript

Test that all dashboard features continue to work after simplification.
```

## 8. QoS Configuration Consolidation

### Investigation Prompt
```
Please analyze QoS usage patterns across rover1 ROS nodes to identify consolidation opportunities.

Analysis:
1. Document all QoS profiles used across the project
2. Identify which QoS patterns are duplicated
3. Determine which QoS complexity is truly needed
4. Map topic publishers and their actual QoS requirements
5. Find opportunities for standardization

Provide a design for centralized QoS management that reduces duplication while maintaining compatibility.
```

### Implementation Prompt
```
Please consolidate QoS configuration patterns across the rover1 project.

Current state: QoS profiles scattered and duplicated
Target: Centralized QoS management

Implementation:
```python
class RoverQoS:
    """Centralized QoS profiles for rover topics"""
    
    @staticmethod
    def sensor_data():
        """For sensor topics like IMU, GPS"""
        return QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
    
    @staticmethod 
    def control_commands():
        """For cmd_vel and control topics"""
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, 
            depth=10
        )
    
    @staticmethod
    def status_data():
        """For health monitoring and status"""
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
```

Update all ROS nodes to use centralized QoS:
1. Replace scattered QoS definitions with RoverQoS calls
2. Maintain identical behavior for all existing topics
3. Preserve compatibility with current publishers/subscribers
4. Document QoS choices and requirements

Requirements:
- Zero functional change to topic communication
- All existing subscriptions continue to work
- Same reliability and performance characteristics
- Easier QoS management for future development

Test all ROS topic communication after changes.
```

---

## Risk Assessment Summary

### **Critical Success Factors**
1. **Zero functionality loss** - Any refactor that breaks existing features fails
2. **Comprehensive testing** - Hardware, GPS, controls, monitoring must work identically  
3. **Staged approach** - Do immediate refactors first, validate before proceeding
4. **Rollback plan** - Each refactor must be easily reversible

### **Risk Mitigation Strategies**
1. **Test in simulation first** - Use hardware mocks where possible
2. **Field testing required** - All hardware refactors need real rover validation
3. **Version control checkpoints** - Commit working state before each refactor
4. **Documentation** - Record current behavior for comparison

### **Success Metrics**
- All current functionality preserved
- Development velocity improved  
- Code maintainability increased
- Testing coverage enhanced
- No regression in field reliability

---

## Implementation Timeline

**Immediate (Pre-MVP):** Items 1-3 (1 hour total)
**Phase 1 (Post working autonomy):** Items 4-5 (2-3 days)  
**Phase 2 (After field testing):** Items 6-8 (1-2 days)

**Total estimated effort:** 4-6 days spread over multiple development cycles

---

## Future Considerations

This refactoring roadmap assumes current architectural decisions are sound. After autonomous navigation is working and field-tested, reassess priorities based on:

1. **Performance bottlenecks** discovered during autonomous operation
2. **Reliability issues** encountered in field deployment  
3. **Development pain points** when adding new features
4. **Integration challenges** with additional hardware/sensors

The goal is **working autonomy first, clean code second**. This roadmap preserves that priority while providing a clear path to improved maintainability.