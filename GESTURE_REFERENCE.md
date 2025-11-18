# 🤖 Robot Gesture Control Reference Guide

Complete reference for all hand gestures and their corresponding robot movements.

---

## 📋 Quick Reference Table

| Gesture | Finger Pattern | HandCtrl Action | FingerCtrl Action |
|---------|----------------|-----------------|-------------------|
| **Fist** ✊ | [0,0,0,0,0] | Backward (1.5s) | - |
| **👍 Thumb Up** | [1,0,0,0,0] | Strafe Left (1.5s) | Counter-clockwise Circle (13s) |
| **🖕 Middle Finger (Spin)** | [0,0,1,0,0] | Fast Spin (1.5s) | Fast Spin (12s) |
| **☝️ Index Only** | [0,1,0,0,0] | Rotate Right (0.5 rad/s, 1.5s) | - |
| **✌️ Peace/Yes** | [0,1,1,0,0] | Rotate Left (0.5 rad/s, 1.5s) | Square Pattern (~18s) |
| **🤟 Three** | [0,1,1,1,0] | Forward + Arc Right (1.5s) | Triangle Pattern (~24s) |
| **✋ Four** | [0,1,1,1,1] | Forward + Arc Left (1.5s) | Back & Forth 2x (~10s) |
| **🖐️ Five (Open Hand)** | [1,1,1,1,1] | Forward (1.5s) | Stop/Idle |
| **🤙 Shaka** | [1,0,0,0,1] | Strafe Right (1.5s) | Figure-8 ∞ (~19.2s) |
| **🤘 Rock On** | [1,1,0,0,1] | Backward (1.5s) | Diamond Pattern ◇ (~14.5s) |
| **👌 OK** | Special† | - | Clockwise Circle (13s) |
| **👎 Thumb Down** | Special‡ | - | Forward 4s |
| **Index + Pinky** | [0,1,0,0,1] | - | S-Pattern (8s) |

† OK gesture: Thumb touching index (angle < 10°), other 3 fingers up  
‡ Thumb Down: Thumb tip is lowest point in frame (cyList[4] == max)

---

## 🎮 NEW GESTURES ADDED

### 1. 🤘 Rock On Sign
- **Pattern**: Thumb + Index + Pinky [1,1,0,0,1] (exactly 3 fingers)
- **Detection**: `gesture == "Rock_on"` - thumb, index, and pinky up
- **HandCtrl**: Move backward for 1.5 seconds (x = -0.3 m/s)
- **FingerCtrl**: Execute **DIAMOND PATTERN ◇** (4 diagonal sides at 45° angles)
- **Duration**: ~1.5s (HandCtrl) / ~14.5s (FingerCtrl with pauses)
- **Details**: Diagonal movements (↗↘↙↖) at 0.424 m/s diagonal speed, ~1.27m per side

### 2. 🖕 Spin (Middle Finger Only)
- **Pattern**: Middle finger only [0,0,1,0,0]
- **Detection**: `gesture == "Spin"` - single middle finger extended
- **HandCtrl**: Fast spin for 1.5 seconds (2.0 rad/s angular velocity, 0 linear)
- **FingerCtrl**: **VERY FAST SPIN for 12 SECONDS** (2.0 rad/s, 0 linear)
- **Note**: Robot spins in place with no forward movement (~3.8 rotations in 12s)

### 3. 🤙 Shaka (Hang Loose)
- **Pattern**: Thumb + Pinky only [1,0,0,0,1]
- **Detection**: `gesture == "Shaka"` - only thumb and pinky extended
- **HandCtrl**: Strafe right for 1.5 seconds (y = -0.3 m/s)
- **FingerCtrl**: **TRUE INFINITY SYMBOL (∞)** - Two complete circles
- **Duration**: ~19.2s total (includes pauses and transitions)
- **Details**: Clockwise loop (9s) → straight transition (0.3s) → counter-clockwise loop (9s) → return to start (0.3s)

---

## 📐 Movement Patterns in Detail

### FingerCtrl.py Patterns:

#### **Square Pattern** (Yes/Peace gesture)
- 4 sides, each side 3 seconds forward
- 90° turns between sides (π/2 rad ≈ 1.57s at 1.0 rad/s)
- Total: ~18 seconds
- Pattern: Forward → Turn → Repeat 4x

#### **Diamond Pattern ◇** (Rock On gesture) - NEW!
- 4 diagonal sides at 45° angles
- Each side: 3 seconds diagonal movement (↗↘↙↖)
- Uses combined forward/backward + strafe velocities (0.3 m/s each axis)
- Diagonal speed: ~0.424 m/s (sqrt(0.3² + 0.3²))
- Total: ~14.5 seconds
- Returns to starting position
- Pattern: Forward-Right → Back-Right → Back-Left → Forward-Left

#### **Circle Patterns**
- **Clockwise** (OK gesture): 0.3 m/s forward + 0.5 rad/s (positive) for 13s
- **Counter-clockwise** (Thumb Up gesture): 0.3 m/s forward + -0.5 rad/s (negative) for 13s
- Forms complete circular path while moving forward
- Creates circle by combining forward motion with continuous rotation

#### **Figure-8 / Infinity ∞** (Shaka) - IMPROVED!
- First loop: Clockwise circle (0.698 rad/s, 9s) - exact 360°
- Brief straight transition to center (0.3s)
- Second loop: Counter-clockwise circle (-0.698 rad/s, 9s)
- Return to starting position (0.3s)
- Creates proper closed infinity symbol shape
- Circle radius: ~0.43m (43cm)
- Total: ~19.2 seconds
- Returns to starting position ✅

#### **Triangle** (Three fingers)
- 3 sides with **135° turns** (wider, more obtuse angles)
- Each side: 4.0s forward (LARGER triangle - 1.2m per side)
- Turn: 1.0 rad/s for 3.4s = 135°
- Total: ~24 seconds
- Note: Makes a wider triangle shape than standard equilateral (120°)

#### **S-Pattern** (Index + Pinky)
- Arc right: 0.3 m/s forward + -0.5 rad/s for 3s
- Pause 1s
- Arc left: 0.3 m/s forward + 0.5 rad/s for 3s
- Pause 1s
- Total: ~8 seconds
- Note: This uses generic finger pattern [0,1,0,0,1], not named gesture

#### **Back & Forth** (Four fingers)
- Forward 2s → Pause 0.5s → Back 2s → Pause 0.5s (repeat 2x)
- Linear velocity: 0.3 m/s forward, -0.3 m/s backward
- Total: ~10 seconds (2 complete cycles)

#### **Fast Spin** (Middle finger) - NEW!
- Spins at 2.0 rad/s for 12 seconds
- No forward movement (0 linear velocity)
- Completes ~3.8 full rotations

---

## 🔧 Technical Details

### Velocity Parameters:
- **Linear Speed**: 0.3 m/s (forward/backward in x-axis)
- **Strafe Speed**: 0.3 m/s (left/right in y-axis)
  - Left: y = 0.3 (positive)
  - Right: y = -0.3 (negative)
- **Standard Turn (in place)**: 0.5-1.0 rad/s
- **Arc Movement (HandCtrl Three/Four)**: 0.3 rad/s rotation while moving 0.3 m/s forward
- **Circle Movement (FingerCtrl)**: 0.3 m/s forward + 0.5 rad/s rotation
- **Fast Spin**: 2.0 rad/s (HandCtrl & FingerCtrl)
- **Diagonal Speed**: ~0.424 m/s (when combining x and y at 0.3 each, used in diamond pattern)

### pub_vel(x, y, z) Parameters:
- `x`: Linear velocity (forward=positive, backward=negative) in m/s
- `y`: Strafe velocity (left=positive, right=negative) in m/s
- `z`: Angular velocity (right/clockwise=positive, left/counter-clockwise=negative) in rad/s
  - Note: This is robot-centric rotation viewed from above

### Sleep/Duration Times:
- **HandCtrl gestures**: 1.5 seconds for all basic movements
- **FingerCtrl Spin**: 12 seconds
- **FingerCtrl patterns**: Variable (see pattern details above)
- **Pattern pauses**: 0.5s between movements for stability

---

## 🐛 Issues Fixed & Improvements

### ✅ Figure-8 Pattern (Nov 6, 2025)
**Problem**: Pattern didn't return to starting position, imperfect circles  
**Solution**: 
- Added return segment to close the loop
- Precise angular velocity (2π/9 rad/s) for exact 360° circles
- Added stability pauses between segments
- Now creates true closed infinity symbol ∞

### ✅ Rock On Diamond Pattern (Nov 6, 2025)
**Change**: Converted from square to diamond pattern  
**Implementation**:
- Uses diagonal movements (forward+strafe combined)
- Four 45° diagonal sides: ↗↘↙↖
- Faster execution (~14.5s vs ~32s for square)
- More visually impressive rotated square shape

### ✅ Square Pattern Optimization (Nov 6, 2025)
**Improvement**: Refactored with loop and precise math  
**Changes**:
- Used for-loop instead of repeated code
- Precise 90° turns (π/2 radians)
- Reduced from ~32s to ~18s total time

### ✅ Gesture Detection Priority (Earlier)
**Problem**: Generic patterns were catching custom gestures  
**Solution**: Named gestures now checked BEFORE generic finger-count patterns

### ✅ Triangle Rotation (Earlier)
**Problem**: Inconsistent rotation calculations  
**Solution**: Standardized to 1.0 rad/s velocity with proper timing

---

## 💡 Usage Tips

1. **HandCtrl.py**: Quick directional control and short movements (1.5s each)
2. **FingerCtrl.py**: Complex pattern movements and demonstrations (varies by pattern)
3. **Gesture Priority**: Named gestures checked BEFORE generic finger-count patterns
4. **Detection Order in code**:
   - Thumb_down (position-based check first)
   - OK gesture (thumb-to-index angle check)
   - Rock_on (specific 3-finger combination)
   - Three, Four (specific finger patterns)
   - Yes, Shaka (2-finger combinations)
   - Thumb_up, Spin (single-finger gestures)
   - Generic finger counts (fallback)
5. **Sleep Times**: HandCtrl uses 1.5s; FingerCtrl varies (12s for spin, 10-24s for patterns)

---

## 🎯 Best Practices

- **Hold gestures steady** for reliable detection (MediaPipe needs consistent input)
- **HandCtrl mode** (quick movements):
  - Thumb Up (👍) = Strafe Left
  - Shaka (🤙) = Strafe Right
  - Fist (✊) = Backward
  - Five Fingers (🖐️) = Forward
  - Index Only (☝️) = Rotate Right (0.5 rad/s)
  - Peace/Yes (✌️) = Rotate Left (0.5 rad/s)
  - Three (🤟) = Forward + Arc Right (curves while moving)
  - Four (✋) = Forward + Arc Left (curves while moving)
  - Rock On (🤘) = Backward
  - Middle Finger (🖕) = Fast Spin (2.0 rad/s)
  
- **FingerCtrl mode** (patterns):
  - Rock On (🤘) = **Diamond pattern ◇** (diagonal movements, ~14.5s)
  - Peace/Yes (✌️) = **Square pattern** (4 sides with 90° turns, ~18s)
  - Shaka (🤙) = **Infinity symbol ∞** (figure-8, ~19.2s)
  - Middle Finger (🖕) = **Extended fast spin** (12s, ~3.8 rotations)
  - Three (🤟) = **Triangle** (3 sides, 135° turns, ~24s)
  - Four (✋) = **Back & forth** (2 complete cycles, ~10s)
  - Thumb Up (👍) = **Counter-clockwise circle** (13s)
  - OK (👌) = **Clockwise circle** (13s)
  - Thumb Down (👎) = **Forward for 4 seconds**
  - Index + Pinky = **S-pattern** (curved path, ~8s)
  - **Five Fingers = STOP** (halts all movement)

---

## 📝 Gesture Detection Logic

### Named Gestures (in media_library.py):
1. **Thumb_down**: Position-based - thumb tip is lowest point `cyList[4] == max(cyList)`
2. **OK**: Angle-based - thumb touching index `ThumbTOforefinger() < 10°` with 3 other fingers up
3. **Rock_on**: Pattern [1,1,0,0,1] - exactly 3 fingers (thumb + index + pinky)
4. **Three**: Pattern [0,1,1,1,0] - index + middle + ring only
5. **Four**: Pattern [0,1,1,1,1] - all except thumb
6. **Yes**: Pattern [0,1,1,0,0] - index + middle only  
7. **Shaka**: Pattern [1,0,0,0,1] - thumb + pinky only
8. **Thumb_up**: Pattern [1,0,0,0,0] - thumb only
9. **Spin**: Pattern [0,0,1,0,0] - middle finger only

### Generic Patterns (fallback in HandCtrl/FingerCtrl):
- Checked only if no named gesture matches
- Based on total finger count and specific finger combinations

---

**Last Updated**: November 18, 2025  
**Files**: `media_library.py`, `HandCtrl.py`, `FingerCtrl.py`  
**Documentation verified against current codebase**

