---
title: Lab 8
---

# Lab 8: Stunts

## Controlled Stunts

To do the flip, I first tried using PID control to get as close to the wall as possible. However, that would often cause my robot to stop too close to the wall instead; I needed it to have some forward velocity in order for the car to flip. To try to get around that, I set the proportional constant much higher than I normally would. However, that just caused the robot to slam into the wall:

[![P control fail](http://img.youtube.com/vi/K7v6CuM8gUU/0.jpg)](http://www.youtube.com/watch?v=K7v6CuM8gUU)

I didn't implement the Kalman filter on the robot because of how its results weren't reliable enough for my liking, so I then went for a simple approach of timing forward movement and suddenly reversing. This was the same approach I used before when the robots still relied on their remote controls.

```python
bot.move_forward(255)
time.sleep(1)
bot.move_backward(255)
time.sleep(1)
bot.stop()
```

This caused the robot to move forward and then gently move backwards. I suspected that it took too long for bluetooth to send each command, so I eliminated that variability by creating a "sudden flip" command.

```cpp
case SUDDEN_FLIP:
  success = robot_cmd.get_next_value(base_power);
  if (!success) return;
  stop_motors(true);
  move_backward(base_power);
  break;
```

I then called my command in Python code, and after some tries figuring out a good starting position, I got my car to do a flip.

```python
bot.set_motor_calibration(1.8)
bot.start_data_collection()
bot.move_forward(255)
time.sleep(1)
bot.sudden_flip(255, scale=False)
time.sleep(1)
bot.stop()
bot.stop_data_collection()
```

You may notice that the `sudden_flip()` function has an option to scale. This refers to how my movement functions scale the maximum power down to `255 / motor_calibration_factor` in order to ensure straight-line driving. For this flip, I suspected I would need all the power I could get given my bot's tendancy to simply drive backwards instead of flip, so I made a way to turn the scaling off. This may also explain why we get a flip that's more curved than straight.

[![Flip 1](http://img.youtube.com/vi/LqpjQ4E__PY/0.jpg)](http://www.youtube.com/watch?v=LqpjQ4E__PY)

[![Flip 2](http://img.youtube.com/vi/BNiTNU1MDY0/0.jpg)](http://www.youtube.com/watch?v=BNiTNU1MDY0)

## Open-Loop Stunts

I originally planned to try to get my robot to parallel park. After trying a few times and seeing some reliability issues with getting into a specific spot, I tried to increase reliability by removing the tape attached to the wheels that I used before when trying to get the robot to spin on its axis easier.

However, the glue remover I needed to remove the adhesive from the wheels had the unintended side effect of making the table and wheels slippery! That led to my open-loop stunt: The Parking Drift!

[![The parking drift video](http://img.youtube.com/vi/XlI2pmH9c3A/0.jpg)](http://www.youtube.com/watch?v=XlI2pmH9c3A)

The controls I used for this are listed below:
```python
bot.move_duration(100,500) # 100 power, 500 ms
bot.spin(80,-80,900) # 80 L, -80 R, 900 ms
bot.move_duration(100,1800)
bot.spin(100,80,500)
```

This also took a few tries to perfect, which brought rise to this bloopers video:

[![The parking drift blooper video](http://img.youtube.com/vi/nqheZMJb8gQ/0.jpg)](http://www.youtube.com/watch?v=nqheZMJb8gQ)
