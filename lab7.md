---
title: Lab 7
---

# Lab 7: Kalman Filter

## Step Response

To get the step response, I wrote some code that would drive the robot into a wall as fast as it could and stop if the robot get "close enough" to the wall to avoid damaging it. This code appears in my control loop, which looks very similar to how I implemented PID control.

Arduino (in `loop()`):
```cpp
if (use_step_response) {
  // Again, use backwards-facing ToF
  motor_power = clip_motor_value(255);
  if (current_tof_front <= step_stop) {
    motor_power = 0;
    stop_motors(true);
  } else {
    move_forward(motor_power);
  }
  pid_motor_power[pid_motor_power_idx] = motor_power;
  pid_motor_power_idx++;
}
```

Jupyter Notebook:
```python
bot = RobotControl(ble)
bot.stop_notify()
bot.set_motor_calibration(1.8)
bot.start_data_collection()
bot.start_step_response(400)
time.sleep(2.5)
bot.stop_step_response()
bot.stop()
bot.stop_data_collection()
```

The python code allows me to do a few things, like choose how close "close enough" is and change the motor calibration.

### An aside: Programmable Calibration Factor
As I tried this, I ran into issues where my car would no longer drive straight. To avoid having to upload new code repeatedly, I created a command for this as well:

Arduino code (in `handle_command()`):
```cpp
case SET_MOTOR_CALIB:
  success = robot_cmd.get_next_value(new_calib_factor);
  if (!success) return;
  motor_calib_factor = new_calib_factor;
  break;
```

Corresponding Python code (in `RobotControl`):
```python
def set_motor_calibration(self, new_val):
  ble.send_command(CMD.SET_MOTOR_CALIB, new_val)
```

The calibration factor often depended on battery life, so it was helpful to be able to change it this way. This is also important for how the motor power is determined; to avoid driving in a non-straight line, the maximum motor power is clipped based on the calibration factor:

```cpp
motor_power = clip_motor_value(255);
...
int clip_motor_value(float val_in) {
  if (val_in >= (255 / motor_calib_factor) ){
    return int(255 / motor_calib_factor);
  } else if (val_in <= MIN_POWER) {
    return 0;
  }
  return round(val_in);
}
```





