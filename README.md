# TurtleBot Hector SLAM

Case study for **ICTE4001** (Curtin Colombo, Year 3). The project explores
2D mapping and localisation for a TurtleBot using Hector SLAM, with the robot
simulated in [Webots](https://cyberbotics.com/).

## Repository structure

```
.
├── docs/      Project documentation (reports, notes, diagrams)
├── webots/    Webots simulation files (worlds, controllers, protos)
└── README.md
```

| Folder | Purpose |
| ------ | ------- |
| [`docs/`](docs/) | Written documentation for the case study |
| [`webots/`](webots/) | Webots worlds, robot controllers and related assets |

## Getting started

### Prerequisites

- [Webots](https://cyberbotics.com/#download) (R2023b or newer recommended)
- ROS with the `hector_slam` package, if running SLAM outside Webots

### Clone

```bash
git clone https://github.com/TurtleBot-programming/turtlebot-hector-slam.git
cd turtlebot-hector-slam
```

### Run the simulation

1. Open Webots.
2. Choose **File → Open World** and select a `.wbt` file from `webots/worlds/`.
3. Press **Play** to start the simulation.

> The simulation files are being added; see [`webots/README.md`](webots/README.md).

## Contributing

Create a branch, commit your changes with a clear message, and open a pull
request against `main`.
