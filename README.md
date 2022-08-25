# Importent

## Known Issues
### Simplicity Studio® Version 5.2.0.0

| ID | Issue | Workaround |  |  |
|---|---|---|---|---|
| 705724 | If multiple instances of the PWM software component <br>use the same timer module, the GPIO output pin for <br>the PWM instance and the timer output channel to use <br>cannot be set in Pin Tool. | Create the desired PWM instances and then <br>use a text editor to manually set the PWM pin <br>and channel in the corresponding <br>sl_pwm...config.h header file. |  |  |

The files need to be updated are:
- <project_loc>/config/sl_pwm...config
- <project_loc>/autogen/sl_pwm_init.c
- <project_loc>/autogen/sl_pwm_instances.h
- <project_loc>/gecko_sdk_4.1.1/platform/driver/pwm/

## API documentation

- [GPIOINT - GPIO Interrupt](https://docs.silabs.com/gecko-platform/4.1/driver/api/group-gpioint)
- [PWM Driver](https://docs.silabs.com/gecko-platform/4.1/driver/api/group-pwm)
- [USTIMER - Microsecond Timer](https://docs.silabs.com/gecko-platform/4.1/driver/api/group-ustimer)