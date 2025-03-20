# MazeSolverProject

## Overview
MazeSolverProject is a **PlatformIO-based** embedded system project designed to efficiently solve mazes using **optimized FloodFill algorithms**. The project is built for a **maze-solving robot** equipped with:

- **4 HC-SR04 ultrasonic sensors** (front, right, back, left) for obstacle detection.
- **MPU6050 IMU sensor** for dead reckoning and odometry-based movement.
- **Custom movement logic** for precise control, accurate turning, and movement optimization.

This system is designed to operate autonomously in a maze, accurately mapping walls and computing the shortest path to the goal using a **FloodFill algorithm** with optimizations. This was built for the 2023 Technoxian World Robotics Championship 2023 under team RoboManipal.

## Features
- **Optimized FloodFill Algorithm** for efficient path planning.
- **HC-SR04 Ultrasonic Sensor Integration** for real-time obstacle detection.
- **IMU-based Odometry** using an MPU6050 for accurate movement tracking.
- **Dead Reckoning Capabilities** for position estimation.
- **Custom PID-based Movement Control** ensuring precision in turns and navigation.
- **Modular Design** for easy modification and adaptation to different robotic platforms.
- **PlatformIO Support** for streamlined development and deployment.

## Project Structure
```
MazeSolverProject/
│── include/        # Header files for classes and functions
│── src/            # Source files implementing maze-solving algorithms
│── lib/            # Custom libraries (MazeBot, Sensor Integration)
│── test/           # Test cases for verifying correctness
│── platformio.ini  # Configuration file for PlatformIO
│── images/         # (Optional) Contains an image of the robot
```

## Hardware Configuration
- **Motors:** Two independent drive motors (left and right) for movement.
- **Ultrasonic Sensors:** Four HC-SR04 sensors for distance measurement.
- **IMU Sensor:** MPU6050 for accurate angle measurement and dead reckoning.
- **Microcontroller:** Compatible with ESP32 or similar embedded controllers.

## Dependencies
This project requires the following dependencies:
- **PlatformIO**: A development platform for embedded systems.
- **Arduino Framework**: Provides essential libraries for Arduino-based development.
- **Wire Library**: For I2C communication with the MPU6050.

These dependencies are managed through the `platformio.ini` configuration file.

## Installation
To set up and run the MazeSolverProject:

1. **Clone the repository**:
   ```bash
   git clone https://github.com/TarunMurugan/MazeSolverProject.git
   ```

2. **Navigate to the project directory**:
   ```bash
   cd MazeSolverProject
   ```

3. **Open the project using PlatformIO**:
   - If using VS Code, install the PlatformIO extension and open the project folder.
   - If using the CLI, navigate to the project directory and run:
     ```bash
     platformio run
     ```

4. **Build the project**:
   ```bash
   platformio run
   ```

5. **Upload to target hardware (if applicable)**:
   ```bash
   platformio run --target upload
   ```

## Usage
- The bot initializes by calibrating its **MPU6050 IMU**.
- It then begins scanning the environment using **ultrasonic sensors**.
- **Wall mapping** is performed using sensor data, and the **FloodFill algorithm** is executed to navigate the maze.
- The bot autonomously moves through the maze while optimizing movement based on **PID control**.
- The bot maintains an **odometry log** using IMU data for **precise movement tracking and dead reckoning**.

## Custom Library (MazeBot)
The `lib/` folder contains a custom-built `MazeBot` library that integrates:
- **Ultrasonic sensor handling**
- **IMU-based motion control**
- **Movement commands** (forward, turn left, turn right)
- **Wall detection logic**

## Example Robot Setup
An image of the bot:
![image of bot in maze}(images/mazeBot.jpg)
credits: "[Team Robomanipal](https://robomanipal.com/)"


## Contributing
Contributions are welcome! If you'd like to contribute:
- Fork the repository.
- Create a feature branch.
- Commit your changes.
- Submit a pull request for review.

## License
This project is licensed under the **MIT License**. See the `LICENSE` file for details.

## Acknowledgments
Special thanks to contributors and the open-source community for providing valuable resources and tools that make this project possible.

