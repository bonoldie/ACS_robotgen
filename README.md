# Advanced Control Systems robots generation scripts | UniVR

Build a batch of serial robots URDFs for the ACS master degree course.

To run the example use 
```sh
/usr/bin/python3 src/generator.py --config configs/test_config.json
```

**Install dependencies if needed.**

## Configuration

Example configuration:
```json
{
  "rows": [
    {
      "type": "P",
      "axis": "xz",
      "length": "0.4",
      "mass": "1.2"
    },
    {
      "type": "X",
      "axis": "yz",
      "direction": "xyz",
      "length": "0.4",
      "mass": "1"
    },
    {
      "type": "X",
      "axis": "xy",
      "direction": "z",
      "length": "0.2",
      "mass": "0.5"
    }
  ],
  "mode": "random",
  "count": 30,
  "seed": 1,
  "radius": 0.02,
  "base_height": 0.15,
  "revolute_limit": 3.141592653589793,
  "effort": 100.0,
  "velocity": 1.0
}
```

- ```rows``` an array of joints
  - ```type``` the joint type, (**P**,**R**,**X**) where **X** is one of **P** or **R**
  - ```axis``` the joint action axis (w.r.t. world frame), e.g. **xy** will generate both definitions for the x-acting and y-acting joint
  - ```length``` the link length on the ```direction``` direction (for prismatic joints the direction is the same as ```axis```)
  - ```mass``` the link mass
- ```mode``` the generation mode (**random**, **all**), when **random** is selected a batch of #```count``` random (seeded by ```seed```) robots will be generated
- ```radius```


