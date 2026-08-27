# UsageTemplate

A template module for SlicerCR that demonstrates various robot visualization capabilities and provides example implementations for different robot types.

## Overview

The UsageTemplate module serves as a practical example implementation for the SlicerCR extension. It provides six different robot demonstrations that showcase various visualization techniques and robot control methods using the SlicerRoboViz system.


## Installation

1. Ensure SlicerRoboViz is properly installed (see main README.md)
2. The UsageTemplate module will be available under the "SlicerCR" category
3. No additional setup required - all dependencies are included

## Usage

### File Structure

```
UsageTemplate/
├── UsageTemplate.py          # Main module implementation
├── Resources/
│   ├── UI/
│   │   └── UsageTemplate.ui
│   └── URDF/                 # Demo robot files
│       ├── Demo_1/
│       ├── Demo_2/
│       ├── Demo_3/
│       ├── Demo_4/
│       ├── Demo_5/
│       ├── Demo_6/
│       ├── Demo_7/
│       └── Demo_8/
└── README.md
```

### Running Demonstrations

1. Open the UsageTemplate module in 3D Slicer
2. Select a demo from the dropdown menu (Demo 1-8)
3. Set the desired frequency (Hz) for the animation
4. Click "Start" to begin the demonstration
5. Click "Stop" to end the animation

### Prerequisites

Before running any demo, you need to load the corresponding robot using SlicerRoboViz:
- **Demo n**: Load `Demo_n_Robot` from `Resources/URDF/Demo_n/Demo_n.urdf`



## Performance Tips

1. **Frequency Settings**: Lower frequencies (30-60 Hz) for better performance
2. **Robot Loading**: Load robots using SlicerRoboViz before running demos
3. **Memory Management**: Stop animations when not needed to free resources

## Troubleshooting

### Common Issues

1. **"No such a robot in the scene"**: Load the corresponding robot using SlicerRoboViz first
2. **Demo not working**: Ensure the correct robot is loaded with the expected name
3. **Performance issues**: Reduce frequency or check if multiple demos are running

### Debug Information

The module displays status messages in the Information Window. Check this area for robot loading status and error messages. If else, restart the Slicer.


## Support

For questions about the UsageTemplate module, refer to the main SlicerRobot documentation or contact the BM2 Lab.
