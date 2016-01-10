# Libfreenect2

A Julia wrapper for libfreenect2 based on Cxx.jl

<div align="center"><img src="examples/depth_streaming_example.gif"></div>

LibFreenect2.jl supports most of libfreenect2 features:

- RGB image transfer
- IR and depth image transfer
- registration of RGB and depth images

Have fun!

## Dependencies

- [Cxx.jl](https://github.com/Keno/Cxx.jl)
- [libfreenect2 (master)](https://github.com/OpenKinect/libfreenect2) (automatically detected or installed)

## Installation

You fist need to install [Cxx.jl](https://github.com/Keno/Cxx.jl). And then, you can install Libfreenect2.jl by:

```jl
Pkg.clone("https://github.com/r9y9/Libfreenect2.jl.git")
Pkg.build("Libfreenect2")
```

This should install Libfreenect2.jl and resolve its binary dependency property. If you do not have libfreenect2 installed, `Pkg.build("Libfreenect2")` will try to install latest libfreenect2.

## How it works

See [examples/multiframe_listener.jl](examples/multiframe_listener.jl) (this is a similar program to Protonect in libfreenec2). Note that it requits [r9y9/OpenCV.jl](https://github.com/r9y9/OpenCV.jl) to visualize color, depth and ir streaming results.
