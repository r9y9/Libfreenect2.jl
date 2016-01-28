__precompile__(false)

module Libfreenect2

export
    # Freenect2
    Freenect2,
    enumerateDevices,
    getDeviceSerialNumber,
    getDefaultDeviceSerialNumber,
    openDevice,
    openDefaultDevice,

    # Packet pipelines
    CpuPacketPipeline,
    OpenGLPacketPipeline,
    OpenCLPacketPipeline,

    # Freenect2Device
    getSerialNumber,
    getFirmwareVersion,
    getColorCameraParams,
    getIrCameraParams,
    setColorFrameListener,
    setIrAndDepthFrameListener,
    start,
    stop,
    close,

    # SyncMultiFrameListener
    SyncMultiFrameListener,
    hasNewFrame,
    waitForNewFrame,
    release,

    # Frame
    FrameType,
    FrameContainer,
    timestamp,
    sequence,
    width,
    height,
    bytes_per_pixel,
    data,
    exposure,
    gain,
    gamma,

    # Registration
    Registration,
    apply,
    getPointXYZRGB


const VERBOSE = Bool(parse(Int, get(ENV, "LIBFREENECT2JL_VERBOSE", "1")))

# Load dependency
VERBOSE && info("Loading deps.jl")
deps = joinpath(dirname(@__FILE__), "..", "deps", "deps.jl")
if isfile(deps)
    include(deps)
else
    error("Libfreenect2 not properly installed. Please run Pkg.build(\"Libfreenect2\")")
end

import Base: convert, getindex, start, close, gamma

VERBOSE && info("Loading Cxx.jl...")
using Cxx

VERBOSE && info("dlopen...")
Libdl.dlopen_e(libfreenect2, Libdl.RTLD_GLOBAL)

function include_headers(top)
    addHeaderDir(top, kind=C_System)
    addHeaderDir(joinpath(top, "libfreenect2"), kind=C_System)
    for name in [
        "frame_listener.hpp",
        "frame_listener_impl.h",
        "libfreenect2.hpp",
        "packet_pipeline.h",
        "registration.h",
        ]
        cxxinclude(joinpath(top, "libfreenect2/$name"))
    end
end

const system_include_top = "/usr/local/include"
const local_include_top = joinpath(Pkg.dir("Libfreenect2", "deps", "usr", "include"))

if isdir(local_include_top)
    VERBOSE && info("Including headers from local path: $local_include_top")
    include_headers(local_include_top)
elseif isdir(joinpath(system_include_top, "libfreenect2"))
    VERBOSE && info("Including headers from system path: $system_include_top")
    include_headers(system_include_top)
else
    error("Cannot find libfreenect2 headers")
end


CpuPacketPipeline() = @cxxnew libfreenect2::CpuPacketPipeline()
OpenGLPacketPipeline() = @cxxnew libfreenect2::OpenGLPacketPipeline()
OpenCLPacketPipeline() = @cxxnew libfreenect2::OpenCLPacketPipeline()

import Cxx: CppEnum
const Libfreenect2FrameType = CppEnum{symbol("libfreenect2::Frame::Type")}

"""libfreenect2::Freenect2"""
const Freenect2 = cxxt"libfreenect2::Freenect2"
Freenect2() = @cxx libfreenect2::Freenect2()

for name in [
    :enumerateDevices,
    ]
    @eval begin
        function $name(f::Freenect2)
            @cxx f->$name()
        end
    end
end

function openDefaultDevice(f::Freenect2, pipeline=Union{})
    if is(pipeline, Union{})
        @cxx f->openDefaultDevice()
    else
        @cxx f->openDefaultDevice(pipeline)
    end
end

function getDeviceSerialNumber(f::Freenect2, idx)
    bytestring(@cxx f->getDeviceSerialNumber(idx))
end

function getDefaultDeviceSerialNumber(f::Freenect2)
    bytestring(@cxx f->getDefaultDeviceSerialNumber())
end

function openDevice(f::Freenect2, name, pipeline=Union{})
    arg1 = isa(name, AbstractString) ? pointer(name) : name

    if is(pipeline, Union{})
        @cxx f->openDevice(arg1)
    else
        @cxx f->openDevice(arg1, pipeline)
    end
end

"""libfreenect2::Freenect2Device*"""
const pFreenect2Device = pcpp"libfreenect2::Freenect2Device"

function getSerialNumber(device::pFreenect2Device)
    bytestring(@cxx device->getSerialNumber())
end

function getFirmwareVersion(device::pFreenect2Device)
    bytestring(@cxx device->getFirmwareVersion())
end

import Base: start, close

for f in [
    :getColorCameraParams,
    :getIrCameraParams,
    ]
    @eval begin
        $f(device::pFreenect2Device) = @cxx device->$f()
    end
end

for f in [
    :start,
    :stop,
    :close
    ]
    @eval begin
        function $f(device::pFreenect2Device)
            r = @cxx device->$f()
            r || error("problem happens in device operation")
        end
    end
end

function setColorFrameListener(device::pFreenect2Device, listener)
    @cxx device->setColorFrameListener(listener)
end

function setIrAndDepthFrameListener(device::pFreenect2Device, listener)
    @cxx device->setIrAndDepthFrameListener(listener)
end

# This is used in the SyncMultiFrameListener constructor, since it requires frame types
# to be specified by unsigned int
const FRAME_COLOR = Cuint(1)
const FRAME_IR    = Cuint(2)
const FRAME_DEPTH = Cuint(4)

module FrameType

import ..Libfreenect2: Libfreenect2FrameType

const COLOR = Libfreenect2FrameType(1)
const IR = Libfreenect2FrameType(2)
const DEPTH = Libfreenect2FrameType(4)

end # module FrameType


function SyncMultiFrameListener(frame_types=FRAME_COLOR | FRAME_IR | FRAME_DEPTH)
    @cxxnew libfreenect2::SyncMultiFrameListener(frame_types)
end

hasNewFrame(listener) = @cxx listener->hasNewFrame()

const FrameMap = cxxt"libfreenect2::FrameMap"

type FrameMapContainer
    handle::FrameMap
end

type FrameContainer
    handle::pcpp"libfreenect2::Frame"
    frame_type::Cuint
    FrameContainer(frame, key=0) = new(frame, key)
end

function FrameContainer(width, height, bytes_per_pixel; key=0)
    frame = icxx"new libfreenect2::Frame($width, $height, $bytes_per_pixel);"
    FrameContainer(frame, key)
end

using CxxStd
function getindex(map::CxxStd.StdMap, key::Libfreenect2FrameType)
    icxx"""
        libfreenect2::Frame* f = $map[$key];
        return f;
    """
end

function getindex(frames::FrameMapContainer, key::Libfreenect2FrameType)
    @assert isa(key, Libfreenect2FrameType)
    FrameContainer(getindex(frames.handle, key), key.val)
end

timestamp(frame::FrameContainer) = icxx"$(frame.handle)->timestamp;"
sequence(frame::FrameContainer) = icxx"$(frame.handle)->sequence;"
width(frame::FrameContainer) = convert(Int, icxx"$(frame.handle)->width;")
height(frame::FrameContainer)= convert(Int, icxx"$(frame.handle)->height;")
bytes_per_pixel(frame::FrameContainer) =
    convert(Int, icxx"$(frame.handle)->bytes_per_pixel;")
data(frame::FrameContainer) = icxx"$(frame.handle)->data;"
exposure(frame::FrameContainer) = icxx"$(frame.handle)->exposure;"
gain(frame::FrameContainer) = icxx"$(frame.handle)->gain;"
gamma(frame::FrameContainer) = icxx"$(frame.handle)->gamma;"


### Frame to Array conversion ###

"""Convenient function to convert Frame* to Array
"""
function _asarray(frame::FrameContainer; do_reshape::Bool=true)
    data_ptr = data(frame)::Ptr{UInt8}

    if frame.frame_type == FRAME_COLOR
        total_length = width(frame) * height(frame) * 4
        array = pointer_to_array(data_ptr, total_length)
        if do_reshape
            array = reshape(array, 4, width(frame), height(frame))
        end
        return array
    elseif frame.frame_type == FRAME_IR || frame.frame_type == FRAME_DEPTH
        total_length = width(frame) * height(frame) * 1
        array = pointer_to_array(convert(Ptr{Float32}, data_ptr), total_length)
        if do_reshape
            array = reshape(array, width(frame), height(frame))
        end
        return array
    else
        error("Cannnot determine type of raw data")
    end
end

function convert(::Type{Array}, frame::FrameContainer)
    _asarray(frame, do_reshape=true)
end
function convert(::Type{Vector}, frame::FrameContainer)
    _asarray(frame, do_reshape=false)
end

# TODO: should use multiple dispatch instead?
function convert{T,N}(::Type{Array{T,N}}, frame::FrameContainer)
    typ = frame.frame_type
    if typ == FRAME_COLOR && T == UInt8 && N == 3
        return _asarray(frame, do_reshape=true)
    elseif typ == FRAME_COLOR && T == UInt8 && N == 1
        return _asarray(frame, do_reshape=false)
    elseif (typ == FRAME_IR || typ == FRAME_DEPTH) && T == Float32 && N == 2
        return _asarray(frame, do_reshape=true)
    elseif (typ == FRAME_IR || typ == FRAME_DEPTH) && T == Float32 && N == 1
        return _asarray(frame, do_reshape=false)
    else
        error("unsupported conversion")
    end
end

# TODO: remove this inefficient glue code
cxx"""
libfreenect2::FrameMap getFrameMap(
    libfreenect2::SyncMultiFrameListener* listener) {
    libfreenect2::FrameMap frames;
    listener->waitForNewFrame(frames);
    return frames;
}
"""
function waitForNewFrame(listener)
    frames = @cxx getFrameMap(listener)
    return FrameMapContainer(frames)
end

function release(frame::FrameContainer)
    icxx"""
    if ($(frame.handle) != nullptr) {
        delete $(frame.handle);
        $(frame.handle) = nullptr;
    }
    """
end

const ColorCameraParams = cxxt"libfreenect2::Freenect2Device::ColorCameraParams"
const IrCameraParams = cxxt"libfreenect2::Freenect2Device::IrCameraParams"

function Registration(irparams::IrCameraParams, cparams::ColorCameraParams)
    @cxxnew libfreenect2::Registration(irparams, cparams)
end

function Base.apply{T<:FrameContainer}(registration, color::T, depth::T,
    undistorted::T, registered::T; enable_filter::Bool=true)
    cframe = color.handle
    dframe = depth.handle
    uframe = undistorted.handle
    rframe = registered.handle
    @cxx registration->apply(cframe, dframe, uframe, rframe, enable_filter)
end

cxx"""
inline uint8_t unsafe_f2uint8(float& rgb, size_t idx) {
    const uint8_t *p = reinterpret_cast<uint8_t*>(&rgb);
    return p[idx];
}
"""
function getPointXYZRGB{T<:FrameContainer}(registration, undistorted::T,
    registered::T, r::Integer, c::Integer)
    uframe = undistorted.handle
    rframe = registered.handle
    x = Ref{Cfloat}()
    y = Ref{Cfloat}()
    z = Ref{Cfloat}()
    rgb = Ref{Cfloat}()
    icxx"""$(registration)->getPointXYZRGB($uframe, $rframe,
        $r, $c, $x, $y, $z, $rgb);"""
    rgbval = rgb[]
    b = @cxx unsafe_f2uint8(rgbval, 0)
    g = @cxx unsafe_f2uint8(rgbval, 1)
    r = @cxx unsafe_f2uint8(rgbval, 2)
    return x[], y[], z[], r, g, b
end

end # module Libfreenect2
