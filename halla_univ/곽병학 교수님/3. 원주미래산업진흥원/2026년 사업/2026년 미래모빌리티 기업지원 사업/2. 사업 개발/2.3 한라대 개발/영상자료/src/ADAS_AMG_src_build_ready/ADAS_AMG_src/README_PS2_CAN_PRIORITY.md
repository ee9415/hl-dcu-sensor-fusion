# PS2 / ROS CAN priority modification

Applied behavior:

1. PS2 connected: PS2 steering and speed always have priority. CAN frames are still received for monitoring, but are not applied.
2. PS2 disconnect transition: `Ps2_StopOnLoss()` immediately commands speed 0 and steering center (1515 us).
3. After PS2 is disconnected: the next valid ROS CAN command (ID 0x300, enable=1, within 500 ms) releases manual steering override and controls steering/speed.
4. CAN timeout or enable=0 while PS2 is absent: steering center and speed 0 are applied.

Modified files:
- `0_Src/AppSw/Tricore/Can_SpeedTx.c`
- `0_Src/AppSw/Tricore/ControlLoop.c`

ROS protocol remains unchanged:
- CAN ID: 0x300
- byte 0-1: steering int16 little-endian, degree x10
- byte 2-3: speed int16 little-endian, km/h x100
- byte 4: enable
- byte 5: alive
