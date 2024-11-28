# pioneer_ws
ros source for pioneer rovers

## Setup
Please set ROBOT_ID as environment variable.
``` 
$ export ROBOT_ID=pN
```

Ex.)
```
$ export ROBOT_ID=p2
```

## Usage
### Joy Stick
```
LY: ↕  
RX: ↔   
LB: enable
```

## Message
```
Role:       msg_name            Type
cmd_vel:    /{ROBOT_ID}/cmd_vel geometry_msgs.msgTwist
ch_val:     /{ROBOT_ID}/ch_val  std_msgs.msg.Int32MultiArray
enable:     /{ROBOT_ID}/enable  std_msgs.msg.Bool
```

## Structure
```
[joy_core/joy_node]
  ↓ (joy)
[rover2_joy]  → (enable) / Not used
__↓_(cmd_vel)____________with_Joypad__
[movebase_kinematics]
  ↓ (ch_val)
[cmd_roboteq]
```