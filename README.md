# A8mini-gimbal-camera-control

- Code for SIYI's A8 mini Gimbal Camera. Allows yaw/pitch control, as well as Auto Center, Zoom, etc.

I've read the [A8 mini docs provided by SIYI](https://shop.siyi.biz/products/siyi-a8-mini) themselves and came up with this handy tool to control the camera.

This also works for other supported camers with the same SDK, like the [A2 mini](https://shop.siyi.biz/products/siyi-a2-mini).

## Setup

- Make sure you setup your camera's default IP address and port under `A8miniControl.h`
- Default IP is `192.168.144.25`
- Default port for control is `37260`
- When using RTSP, stream address is `rtsp://192.168.144.25:8554/main.264`

## Usage

There's the option to control it via cmd line, typing commands indexes, but you can also run it passing cmd line arguments. 

### Examples
After compiling with `gcc A8miniControl.c -o A8miniControl`

#### Via interactive terminal:

Run the code with no arguments: 

`./A8miniControl`

`Insert the command index (0-21):` 

Once you see this, type the command index you wish (list at the end of this README)

#### Via cmd line arguments:

Run the code with the cmd index you want as first argument

- Auto Center    : `./A8miniControl 0`
- Rotate Up      : `./A8miniControl 1`
- Rotate Down    : `./A8miniControl 2`
- Rotate Right   : `./A8miniControl 3`
- Rotate Left    : `./A8miniControl 4`

**Note**: some arguements have parameters fixed by myself, like yaw/pitch velocities. You can change that by checking the docs, and make sure you update the CRC at the end of each command.

### List of currently supported commands:

- 0  - Auto Centering
- 1  - Rotate Up
- 2  - Rotate Down
- 3  - Rotate Right
- 4  - Rotate Left
- 5  - Stop rotation
- 6  - Zoom +1
- 7  - Zoom -1
- 8  - 4.5x
- 9  - Acquire the Max Zoom Value
- 10 - Manual Focus +1
- 11 - Manual Focus -1
- 12 - Take Pictures
- 13 - Record Video
- 14 - Rotate 100 100
- 15 - Gimbal Status Information
- 16 - Auto Focus
- 17 - Acquire Hardware ID
- 18 - Acquire Firmware Version
- 19 - Lock Mode
- 20 - Follow Mode
- 21 - FPV Mode
- 22 - Acquire Attitude Data
- 23 - Set Video Output as HDMI (Only available on A8 mini, restart to take effect)
- 24 - Set Video Output as CVBS (Only available on A8 mini, restart to take effect)
- 25 -  Turn Off both CVBS and HDMI Output (Only available on A8 mini, restart to take effect)
- 26 - Read Range from Laser Rangefinder(Low byte in the front, high byte in the back, available on ZT30)

**Note**: More commands might be supported by the camera but may not be included in the list of implemented commands.

**Disclamer**: SIYI does provide some sample code which was used to build this code.
