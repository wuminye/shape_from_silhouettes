# Build
`mkdir build`

`cd build`

`cmake ..`

`make -j4`


The compiled program is in `./bin`

# Run
An example:
`./bin/vacancy_examples /dataset/sport_1_mask/CamPose.inf /dataset/sport_1_mask/Intrinsic.inf /dataset/sport_1_mask/img/0/mask/ ./data/ -1.0 -1.5 -5.0 1.1 0.0 -3.3 0.2`

Mask file **MUST** named as "img_%04d.jpg" 


Parameters:
```
1.Path to CamPose.inf
2.Path to Intrinsic.inf
3.Path to mask folder
4.Path to output folder
5.6.7 bb_min_x bb_min_y bb_min_z
8,9,10 bb_max_x bb_max_y bb_max_z
11. bb_offset

```

# Dependencies
## Mandatory
- Eigen
    https://github.com/eigenteam/eigen-git-mirror
    - Math
## Optional (can be disabled by cmake)
- stb
    https://github.com/nothings/stb
    - Image I/O
- OpenMP (if supported by your compiler)
    - Multi-thread accelaration
