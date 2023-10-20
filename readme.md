# LINE Follower Robot

This is an Arduino-based project for a line follower robot with an 8-sensor array. The robot uses PID control to follow a line on the ground and can make sharp turns when necessary.

## Hardware Requirements

- ESP32
- QTR-8RC or QTR-8A sensor array
- SparkFun TB6612 motor driver
- DC motors
- Jumper wires
- Breadboard
- Battery pack

## Software Requirements

- Platform IO
- QTRSensors library
- SparkFun_TB6612 library
- TaskScheduler library
- QuickPID library
- JLed library

## Installation

1. Install the required libraries in the Platform IO.
2. Connect the QTR-8RC or QTR-8A sensor array to the Arduino board according to the pin connections in the `setup()` function.
3. Connect the SparkFun TB6612 motor driver to the Arduino board according to the pin connections in the `Motor` constructor.
4. Connect the DC motors to the motor driver.
5. Upload the `main.cpp` file to the Arduino board.

## Usage

1. Power on the robot.
2. Place the robot on a line on the ground.
3. The robot will follow the line using PID control.
4. If the robot loses the line, it will continue moving forward for a certain number of iterations before attempting to find the line again using the `findWhereLastLineWentFromHistory()` function.
5. The robot can make sharp turns when necessary by adjusting the motor speeds based on the line error.

## Contributing

Contributions to this project are welcome. Please submit a pull request with your changes.

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.
