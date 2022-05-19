---
title: Lab 13
---

# Lab 13: Localization and Path Planning (real)

[Click here to return to home page](https://slawrence100.github.io/ece4960-fast-robots/)

## Objective
Traverse a series of waypoints on a given map.

## Strategy

I originally planned to use the localization data collection from the previous lab, but after having a lot of difficulty getting it to accurately predict the robot's location, I decided to navigate without it.

I then made a plan:

1. For each waypoint, compute the control required to end up at that waypoint from the current position and face the next waypoint afterwards.
2. Execute the first part of the control by turning by some angle
3. Execute the next part of the control by driving forward some distance
4. Execute the last part of the control by turning again
5. Repeat with the next waypoint until you run out of waypoints to visit.

## Setup

### Computing Controls
To compute the control, we can mostly rely on `compute_control()` from the localization solution we were given. However, we're able to choose the angles. To avoid moving the robot too much (and creating too much uncertainty), I compute the angle so that the robot is always facing the next waypoint here.

```python
# Assume you perfectly go EVERYWHERE, so you can pre-compute all controls
start = ft2meters(start)
waypoints = [ft2meters(pt) for pt in points]

current_loc = (start[0], start[1], 0)
for i, w in enumerate(waypoints):
    sx, sy, sa = current_loc
    dx, dy = w

    da = 0
    # Compute the angle for the next waypoint
    if i+1 < len(waypoints):
        # compute heading angles and angle betwen vectors
        (x1,y1)= (np.cos(np.radians(sa)), np.sin(np.radians(sa)))
        (x2,y2) = waypoints[i+1]
        x2 = x2 - dx
        y2 = y2 - dy
        denom = np.sqrt(x1**2 + y1**2) * np.sqrt(x2**2 + y2**2)
        numer = x1*x2 + y1*y2
        da = np.degrees(np.arccos(numer/denom))

    (a1,x1,a2) = loc.compute_control((dx,dy,da), (sx,sy,sa))

    # move robot according to control with turn_degrees() and move_distance()
```

## Trial 1: P-Control and Euclidian Movement

I knew from past expriements that my robot's turning ability was good, so I hoped that P-control would suffice to move distances. Using PID, I wrote a function to move by a certain number of millimeters:

```cpp
void move_distance(int dist) {
  int tof_dist = get_tof_measurement(distanceSensorTwo, 5); // average of 5 tries
  int target = tof_dist - dist;
  int motor_power;
  int coast_power = 20;

  while (true) {
    tof_dist = get_tof_measurement(distanceSensorTwo, 1);
    motor_power = pid_proportional_fwd * (tof_dist - target);
    // move based on motor_power, omitted for brevity
  }
}
```

However, when I tried doing this, the robot wasn't able to move forward for distances that were farther than 900 millimeters without overshooting, and turning down the P-constant would cause short-distance movement (less than 300 mm) to be unreachable.

## Trial 2: PID control and Euclidian Movement

To solve the problem of moving different distances more reliably, I upgraded to full PID control:

```cpp
void move_distance(int dist) {
  int tof_dist = get_tof_measurement(distanceSensorTwo, 5);
  int target = max(10, tof_dist - dist);
  // ... bookkeeping variables omitted for brevity
  
  pid_past_time_fwd = millis();
  
  while (true) {
    tof_dist = get_tof_measurement(distanceSensorTwo, 1);
    
    // Do PID control
    pid_error = (tof_dist - target);
    pid_dt = (millis() - pid_past_time_fwd);
    pid_integral_error_acc += pid_error * pid_dt;
    motor_power = pid_proportional_fwd * (tof_dist - target);
    if (pid_dt > 0){
      motor_power += pid_integral_fwd * pid_integral_error_acc;
      motor_power += pid_derivative_fwd * (pid_error - pid_past_error_fwd) / pid_dt;
    }
    pid_past_time_fwd = millis();
    pid_past_error_fwd = pid_error;

    central.connected(); // keep connection alive
    
    if (motor_power < -1*pid_min_power_fwd) {
      motor_power = max(-200, -1*pid_min_power_fwd);
      move_backward(-1*motor_power);
      
    } else if (motor_power >= -1*pid_min_power_fwd && motor_power <= pid_min_power_fwd) {
      stop_motors(true);
      return;
      
    } else if (motor_power > pid_min_power_fwd) {
      motor_power = min(200, pid_min_power_fwd);
      move_forward(motor_power);
    }
  }
}
```

This worked much more reliably to move certain distances. However, testing it on a few waypoints on the real course produced this result:

[![Trial 2 results](http://img.youtube.com/vi/_7-30UmWPkM/0.jpg)](http://www.youtube.com/watch?v=_7-30UmWPkM)

It seems that the ToF sensor was not reliable at far ranges where the sensor beam wasn't close to perpendicular to the wall.

## Trial 3: PID Control and Manhattan Movement
There's very little one can do to avoid seeing the walls at a non-perpendicular angle (as the robot will always have some uncertainty from turning and gyroscope drift), so I devised a plan I dubbed "the Manhattan algorithm" after the idea of Manhattan distance. I chose to place intermediate waypoints that would cause the robot to only move in multiples of 90 degrees.

For example, if you started with waypoints like this,
```
(-4,-3) -> (-2, -1) -> (1, -1),
```
I would add waypoints to always make right-angled turns:
```
(-4,-3) -> (-4,0) -> (0,0) -> (1,0) ->(1,-1) -> (-2,-1)
```

Although some of the paths seem convoluted, I also choose the waypoints to minimze the distance to the walls, as the sensor readings are less noisy for closer walls than farther ones.

TODO finish a trial with the Manhattan movement scheme