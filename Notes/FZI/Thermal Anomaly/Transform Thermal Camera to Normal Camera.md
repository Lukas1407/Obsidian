## tf transforms
```
- header: 
stamp: 
sec: 1713530879 
nanosec: 141914903 
frame_id: inspection_payload_head 
child_frame_id: inspection_payload_thermal_camera 
transform: 
translation: x: 0.0684 y: -0.023 z: -0.03 
rotation: x: -0.5 y: 0.5000000016025516 z: -0.5 w: 0.49999999839744835
```
- The **frame_id** specifies the parent frame (in this case, “inspection_payload_head”).
- The **child_frame_id** specifies the child frame (in this case, “inspection_payload_thermal_camera”).
- The transform Describes the transformation between the parent and child frames.
```
- header:
    stamp:
      sec: 1713530879
      nanosec: 141917120
    frame_id: inspection_payload_tilt
  child_frame_id: inspection_payload_head
  transform:
    translation:
      x: 0.0
      y: 0.0
      z: 0.0
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
```

```
- header:
    stamp:
      sec: 1713539273
      nanosec: 585644483
    frame_id: inspection_payload_pan
  child_frame_id: inspection_payload_tilt
  transform:
    translation:
      x: 0.03
      y: 0.0
      z: 0.152
    rotation:
      x: 0.0
      y: 2.8088027192970366e-06
      z: 0.0
      w: 0.9999999999960552
```
```
- header:
    stamp:
      sec: 1713539273
      nanosec: 685543775
    frame_id: inspection_payload_mount
  child_frame_id: inspection_payload_pan
  transform:
    translation:
      x: 0.0
      y: 0.0
      z: 0.023
    rotation:
      x: 0.0
      y: 0.0
      z: 1.1235210877200554e-06
      w: 0.9999999999993687
```
```
- header:
    stamp:
      sec: 1713530879
      nanosec: 141907542
    frame_id: base
  child_frame_id: inspection_payload_mount
  transform:
    translation:
      x: 0.14253
      y: 0.0
      z: 0.092
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
```
```
- header:
    stamp:
      sec: 1713530879
      nanosec: 141906610
    frame_id: base
  child_frame_id: depth_camera_left_camera
  transform:
    translation:
      x: -0.025
      y: 0.0923
      z: 0.0123
    rotation:
      x: -0.25131071155596757
      y: 0.2590703582772869
      z: 0.6579381045828537
      w: 0.6609409400674415
- header:
    stamp:
      sec: 1713530879
      nanosec: 141906995
    frame_id: base
  child_frame_id: depth_camera_right_camera
  transform:
    translation:
      x: 0.025
      y: -0.0923
      z: 0.0123
    rotation:
      x: 0.24567643143476287
      y: 0.260510589656123
      z: -0.6573691753322627
      w: 0.6630558732395637
```