# Automated Shooting System - User Guide

## Overview
The automated shooting system coordinates multiple subsystems to accurately shoot at the hub using vision and ballistics calculations.

## Components Involved
1. **Shooter** - Spins flywheels to target RPM
2. **Indexer** - Feeds balls with conveyor and kickers
3. **Hood** - Adjusts angle for different distances
4. **Vision** - Provides robot position for distance calculation
5. **Drivetrain** - Auto-aims robot at hub

## How It Works

### Triggering the Shot
- **Button**: Right trigger on joystick 1
- Holding the trigger activates both aiming and shooting

### Automatic Aiming
The robot automatically:
1. Calculates angle to hub based on current position
2. Rotates to face the hub while allowing translation control
3. You can still drive with left stick while aiming

### Ballistics Calculation
When the command starts:
1. Measures distance to hub using odometry
2. Looks up shooter RPM and hood angle from shot table
3. Uses interpolation for distances between table values
4. Sets shooter and hood to calculated targets

### Feeding Sequence
1. Shooter spins up to target RPM
2. When shooter is within ±200 RPM of target, feeding begins
3. Conveyor and kickers activate to feed the ball
4. Continues until trigger is released

## SmartDashboard Values

Monitor these values while tuning:

### Shooting Status
- `Shoot/DistanceToHub` - Distance to target (meters)
- `Shoot/TargetRPM` - Calculated shooter speed
- `Shoot/CurrentRPM` - Actual shooter speed
- `Shoot/RPMError` - Difference between target and actual
- `Shoot/TargetHoodAngle` - Calculated hood position
- `Shoot/Feeding` - Whether ball is being fed

### Vision Status
- `Vision/HasTarget` - AprilTags detected
- `Vision/TagCount` - Number of tags visible
- `Vision/AvgTagDist` - Distance to tags
- `Vision/PoseX`, `Vision/PoseY` - Robot position

## Tuning Guide

### 1. Hub Position
Edit in `commands/shoot_at_hub.py`:
```python
HUB_POSITION = Translation2d(8.23, 4.115)  # X, Y in meters
```
And in `robotcontainer.py`:
```python
HUB_POSITION = Translation2d(8.23, 4.115)  # Same coordinates
```

**To Find Correct Values:**
- Place robot at known position facing hub
- Read robot pose from odometry
- Measure physical distance to hub
- Calculate hub coordinates

### 2. Shot Lookup Table
Edit in `commands/shoot_at_hub.py`:
```python
SHOT_TABLE = [
    (distance_m, hood_angle_deg, shooter_rpm),
    (1.0, 20.0, 3500),   # Close shot
    (2.0, 25.0, 3800),   # Medium shot
    # Add more entries...
]
```

**Tuning Process:**
1. Start with one distance (e.g., 2 meters from hub)
2. Manually test different RPM/angle combinations
3. Record successful combinations in table
4. Repeat for 3-5 different distances
5. The system interpolates between table entries

### 3. Shooter Tolerance
Edit in `commands/shoot_at_hub.py`:
```python
SHOOTER_TOLERANCE_RPM = 200  # Acceptable error before feeding
```

**Adjust if:**
- Too slow to shoot: Increase tolerance (300-400 RPM)
- Inconsistent shots: Decrease tolerance (100-150 RPM)

### 4. Indexer Speeds
Edit in `commands/shoot_at_hub.py` in the `execute()` method:
```python
self.indexer.set_conveyor_target_speed(2000)  # Conveyor RPM
self.indexer.set_left_kicker_target_speed(3000)  # Left kicker RPM
self.indexer.set_right_kicker_target_speed(3000)  # Right kicker RPM
```

**Adjust if:**
- Ball feeds too slowly: Increase speeds
- Ball jams: Decrease speeds or check timing
- Inconsistent: Ensure kickers match shooter speed

### 5. Hood Conversion Factor
The hood position might need conversion from angle to encoder units.
Check `subsystems/hood.py` for position conversion factor:
```python
self.motor_config.encoder.positionConversionFactor(1)
```

**If angles are wrong:**
- Measure physical hood angle vs encoder reading
- Calculate: `conversion_factor = encoder_units / degrees`
- Update the conversion factor

## Common Issues

### Robot Doesn't Aim Correctly
- **Check**: Vision subsystem has targets (`Vision/HasTarget`)
- **Check**: Robot pose is updating (`Vision/PoseX/Y`)
- **Fix**: Verify AprilTag field layout in Limelight settings

### Shooter Never Reaches Target RPM
- **Check**: Target RPM is achievable (not too high)
- **Check**: Shooter PID tuning in `subsystems/shooter.py`
- **Fix**: Adjust feedforward or PID gains

### Hood Angle Wrong
- **Check**: Hood encoder reading makes sense
- **Check**: Hood position conversion factor
- **Fix**: Adjust lookup table values or conversion

### Ball Doesn't Feed
- **Check**: `Shoot/Feeding` becomes true
- **Check**: RPM error is within tolerance
- **Fix**: Increase tolerance or check indexer motor directions

### Shots Miss Target
- **Check**: Distance calculation is accurate
- **Check**: Shot table values are realistic
- **Fix**: Re-tune shot table with field testing

## Advanced Tuning

### Using Vision for Distance
Currently uses odometry. To use vision distance instead:

In `commands/shoot_at_hub.py`, modify `calculate_distance_to_hub()`:
```python
def calculate_distance_to_hub(self) -> float:
    # Get vision estimate distance to hub
    # This would require adding a method to VisionSubsystem
    # to calculate distance from AprilTag data
    pass
```

### Dynamic RPM Adjustment
For moving shots, consider:
- Adding robot velocity to calculations
- Adjusting RPM based on spin direction
- Lead the target for moving hub (defense mode)

### Shot Optimization
Use logged data to optimize:
1. Record: distance, hood angle, RPM, result (hit/miss)
2. Analyze patterns in successful shots
3. Adjust lookup table accordingly
4. Consider using regression for smooth curves

## Testing Procedure

1. **Static Test**
   - Place robot at known distance
   - Hold right trigger
   - Verify shooter spins up
   - Verify hood moves to position
   - Verify feeding starts when ready
   - Observe shot accuracy

2. **Distance Test**
   - Test from multiple distances
   - Record actual results vs expected
   - Adjust lookup table

3. **Moving Test**
   - Drive while shooting
   - Verify aim tracking works
   - Adjust if shots lead/lag

4. **Competition Simulation**
   - Rapid successive shots
   - Different field positions
   - Different robot orientations
   - Verify consistency

## Safety Notes

- Always test with proper field barriers
- Start with low RPM values
- Verify motor directions before full speed
- Have emergency stop ready
- Monitor motor temperatures during extended testing
