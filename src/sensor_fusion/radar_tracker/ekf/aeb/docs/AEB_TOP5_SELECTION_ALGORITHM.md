# AEB Top-5 Selection Algorithm - Detailed Explanation

## Overview
The AEB (Autonomous Emergency Braking) system processes radar tracks and selects the **Top-5 Most Important Objects** based on a multi-factor importance score. This document explains how the scoring algorithm works.

---

## Pipeline Flow

```
Radar Points → EKF Tracker → Active Tracks → Score Calculation → Filtering → Sorting → Top-5 Selection → MIO (Top-1)
```

---

## 1. EKF Tracker (Brief)
- **Input**: Raw radar detection points from front radar (Radar 0)
- **Processing**: 
  - Predict step (motion model)
  - Data association (Mahalanobis distance gating)
  - Update step (Kalman filter)
  - Track management (confirm, delete, create)
- **Output**: List of confirmed `RadarTrack` objects with state estimates (position, velocity, covariance)

---

## 2. Importance Score Calculation (FOCUS AREA)

Each confirmed track is assigned an **Importance Score** (0-100) based on **4 factors**:

### 2.1 Distance Score (MAX: 30 points)

**Formula**:
```c
if (range < 5.0m)           → score = 30
if (5.0m ≤ range < 15.0m)   → score = 30 × (15 - range) / 10
if (15.0m ≤ range < 40.0m)  → score = 15 × (40 - range) / 25
if (range ≥ 40.0m)          → score = 0
```

**Rationale**:
- **Critical zone (0-5m)**: Maximum urgency (30 points) — collision imminent
- **Warning zone (5-15m)**: Linear decay from 30 to 0 points
- **Caution zone (15-40m)**: Linear decay from 15 to 0 points
- **Safe zone (40m+)**: No contribution (0 points)

**Example**:
- Object at 3m → 30 points
- Object at 10m → 30 × (15-10)/10 = **15 points**
- Object at 25m → 15 × (40-25)/25 = **9 points**
- Object at 50m → **0 points**

---

### 2.2 Closing Speed Score (MAX: 30 points)

**Formula**:
```c
rel_speed = ego_speed - track_speed  // Positive = object approaching

if (rel_speed > 10.0 m/s)    → score = 30
if (0.5 m/s < rel_speed ≤ 10.0 m/s) → score = 30 × (rel_speed / 10)
if (rel_speed ≤ 0.5 m/s)     → score = 0
```

**Rationale**:
- **Fast closing (>10 m/s)**: Maximum urgency (30 points) — rapid approach
- **Moderate closing (0.5-10 m/s)**: Linear scale based on closing rate
- **No closing (≤0.5 m/s)**: No contribution (0 points) — object moving away or stationary

**Example** (ego_speed = 20 m/s):
- Track speed = 5 m/s → rel_speed = 15 m/s → **30 points**
- Track speed = 15 m/s → rel_speed = 5 m/s → 30 × (5/10) = **15 points**
- Track speed = 20 m/s → rel_speed = 0 m/s → **0 points**

---

### 2.3 Lateral Position Score (MAX: 25 points)

**Formula**:
```c
lateral = |track.py|  // Absolute distance from vehicle centerline

if (lateral < 1.0m)         → score = 25
if (1.0m ≤ lateral < 3.0m)  → score = 25 × (3 - lateral) / 2
if (lateral ≥ 3.0m)         → score = 0
```

**Rationale**:
- **Directly ahead (<1m)**: Maximum urgency (25 points) — in vehicle path
- **Near path (1-3m)**: Linear decay from 25 to 0 points
- **Off-path (≥3m)**: No contribution (0 points) — unlikely collision

**Example**:
- Object at y = 0.5m → **25 points**
- Object at y = 2.0m → 25 × (3-2)/2 = **12.5 points**
- Object at y = 4.0m → **0 points**

---

### 2.4 Confidence Score (MAX: 15 points)

**Formula**:
```c
score = POE × 15

where POE = Probability of Existence (0.0 - 1.0) from EKF tracker
```

**Rationale**:
- **High confidence (POE ≈ 1.0)**: 15 points — reliable detection
- **Medium confidence (POE ≈ 0.6)**: 9 points — threshold for filtering
- **Low confidence (POE < 0.6)**: Track is filtered out before scoring

**Example**:
- POE = 0.95 → **14.25 points**
- POE = 0.80 → **12.0 points**
- POE = 0.60 → **9.0 points**

---

### 2.5 Total Score

```c
total_score = distance_score + speed_score + lateral_score + confidence_score
total_score = CLAMP(total_score, 0, 100)
```

**Example Calculation**:
```
Object: range=12m, rel_speed=8m/s, lateral=0.8m, POE=0.9

Distance:  30 × (15-12)/10 = 9.0
Speed:     30 × (8/10)     = 24.0
Lateral:   25              = 25.0
Confidence: 0.9 × 15       = 13.5
                           -------
TOTAL:                      71.5 points
```

---

## 3. Filtering

Before sorting, tracks are filtered:
```c
if (!track.confirmed)                               → REJECT (not enough hits)
if (track.POE < 0.6)                                → REJECT (low confidence)
if (track.py < -2.2m || track.py > 2.2m)            → REJECT (outside truck lateral range)
if (importance_score < 5.0)                         → REJECT (not important enough)
```

**Truck Lateral Range**: -2.2m to +2.2m (4.4m total width)
- Ensures only objects **within the truck's path** are considered
- Prevents false positives from objects in adjacent lanes
- Lateral position `py` is measured from vehicle centerline

Only tracks passing **all 4 filters** are considered as candidates.

---

## 4. Sorting and Selection

```c
1. Sort candidates by importance_score (descending)
2. Select top min(5, num_candidates)
3. Store in Top5Objects.objects[0..4]
4. Top5Objects.count = number selected
```

**Result**:
- `objects[0]` = **Top-1** (highest score) → sent via CAN as MIO
- `objects[1]` = **Top-2**
- `objects[2]` = **Top-3**
- `objects[3]` = **Top-4**
- `objects[4]` = **Top-5**

---

## 5. Visualization Integration

**Function**: `drawAEBVerticalView()` + `drawAEBTop1InfoVertical()`

- **Vertical View**: 120m forward × 24m lateral grid
- **Top-1 Info Panel**: Detailed metrics (range, speed, lateral, score)
- **Color Coding**:
  - Top-1: Red (most important)
  - Top-2: Orange
  - Top-3 to Top-5: Yellow

**Display Location**: Lower 2/3 of screen (quadrants 5 & 6)

---

## Summary Table

| Factor           | Max Score | Key Thresholds                          |
|------------------|-----------|-----------------------------------------|
| **Distance**     | 30        | <5m (30), 5-15m (decay), 15-40m (decay) |
| **Closing Speed**| 30        | >10m/s (30), 0.5-10m/s (linear)         |
| **Lateral Pos**  | 25        | <1m (25), 1-3m (decay), >3m (0)         |
| **Confidence**   | 15        | POE × 15 (must be ≥0.6)                 |
| **TOTAL**        | **100**   | Min threshold: 5.0 to be considered     |

---

## Key Insight

The algorithm prioritizes objects that are:
1. **Close** (within 40m, especially <15m)
2. **Approaching fast** (closing speed >0.5 m/s)
3. **In the vehicle path** (lateral offset <3m)
4. **Reliably detected** (POE ≥ 0.6)

This ensures the **Top-1 MIO** is always the most critical threat requiring immediate AEB action.
