# Build
`mkdir build`
`cd build`
`cmake ..`
`make -j4`

The compiled program is in `./bin`

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
