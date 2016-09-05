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
    SyncMultiFrameListenerPtr,
    hasNewFrame,
    waitForNewFrame,
    release,

    # Frame
    FrameMap,
    FrameType,
    FramePtr,
    timestamp,
    sequence,
    width,
    height,
    bytes_per_pixel,
    data,
    exposure,
    gain,
    gamma,

    # release heap-allocated memory
    destroy,

    # Registration
    Registration,
    apply,
    getPointXYZRGB


# Load dependency
deps = joinpath(dirname(@__FILE__), "..", "deps", "deps.jl")
if isfile(deps)
    include(deps)
else
    error("Libfreenect2 not properly installed. Please run Pkg.build(\"Libfreenect2\")")
end

import Base: convert, getindex, start, close, gamma

using Cxx

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
    # info("Including headers from local path: $local_include_top")
    include_headers(local_include_top)
elseif isdir(joinpath(system_include_top, "libfreenect2"))
    # info("Including headers from system path: $system_include_top")
    include_headers(system_include_top)
else
    error("Cannot find libfreenect2 headers")
end


CpuPacketPipeline() = @cxxnew libfreenect2::CpuPacketPipeline()
OpenGLPacketPipeline() = @cxxnew libfreenect2::OpenGLPacketPipeline()
OpenCLPacketPipeline() = @cxxnew libfreenect2::OpenCLPacketPipeline()

import Cxx: CppEnum
const Libfreenect2FrameType = CppEnum{Symbol("libfreenect2::Frame::Type"),Int32}

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

### FrameMap ###

type FrameMap
    handle::cxxt"libfreenect2::FrameMap"
end

function release(jlframes::FrameMap)
    frames = jlframes.handle
    icxx"""
    for (libfreenect2::FrameMap::iterator it = $frames.begin();
         it != $frames.end(); ++it) {
      if (it->second != nullptr) {
        delete it->second;
        it->second = nullptr;
      }
    }
    $frames.clear();
    """
end

### Frame ###

type FramePtr
    handle::pcpp"libfreenect2::Frame"
    frame_type::Cuint

    # Allocated Frame should be set from
    # SyncMultiFrameListener::waitForNewFrame
    FramePtr(frame::pcpp"libfreenect2::Frame", key=0) = new(frame, key)

    function FramePtr(width, height, bytes_per_pixel; key=0)
        frame = icxx"new libfreenect2::Frame($width, $height, $bytes_per_pixel);"
        p = new(frame, key)
        finalizer(p, destroy)
        p
    end
end

function destroy(frame::FramePtr)
    icxx"""
    if ($(frame.handle) != nullptr) {
        delete $(frame.handle);
        $(frame.handle) = nullptr;
    }
    """
end

using CxxStd
function getindex(map::CxxStd.StdMap, key::Libfreenect2FrameType)
    icxx"""
        libfreenect2::Frame* f = $map[$key];
        return f;
    """
end

function getindex(frames::FrameMap, key::Libfreenect2FrameType)
    @assert isa(key, Libfreenect2FrameType)
    FramePtr(getindex(frames.handle, key), key.val)
end

for member in [
    :timestamp,
    :sequence,
    :data,
    :exposure,
    :gain,
    :gamma
    ]
    body = "\$(frame.handle)->$(member);"
    ex = Expr(:macrocall, Symbol("@icxx_str"), body)
    @eval $member(frame::FramePtr) = $ex
end

for member in [
    :width,
    :height,
    :bytes_per_pixel,
    ]
    body = "\$(frame.handle)->$(member);"
    ex = Expr(:macrocall, Symbol("@icxx_str"), body)
    @eval $member(frame::FramePtr) = convert(Int, $ex)
end

### Frame to Array conversion ###

"""Convenient function to convert Frame* to Array
"""
function _asarray(frame::FramePtr; do_reshape::Bool=true)
    data_ptr = data(frame)::Ptr{UInt8}

    if frame.frame_type == FRAME_COLOR
        total_length = width(frame) * height(frame) * 4
        array = unsafe_wrap(Array, data_ptr, total_length)
        if do_reshape
            array = reshape(array, 4, width(frame), height(frame))
        end
        return array
    elseif frame.frame_type == FRAME_IR || frame.frame_type == FRAME_DEPTH
        total_length = width(frame) * height(frame) * 1
        array = unsafe_wrap(Array, convert(Ptr{Float32}, data_ptr), total_length)
        if do_reshape
            array = reshape(array, width(frame), height(frame))
        end
        return array
    else
        error("Cannnot determine type of raw data")
    end
end

function convert(::Type{Array}, frame::FramePtr)
    _asarray(frame, do_reshape=true)
end
function convert(::Type{Vector}, frame::FramePtr)
    _asarray(frame, do_reshape=false)
end

# TODO: should use multiple dispatch instead?
function convert{T,N}(::Type{Array{T,N}}, frame::FramePtr)
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

### SyncMultiFrameListener ###

type SyncMultiFrameListenerPtr
    handle::cxxt"std::shared_ptr<libfreenect2::SyncMultiFrameListener>"

    function SyncMultiFrameListenerPtr(
            frame_types=FRAME_COLOR | FRAME_IR | FRAME_DEPTH)
        new(icxx"""
            std::shared_ptr<libfreenect2::SyncMultiFrameListener>(
                new libfreenect2::SyncMultiFrameListener($frame_types));
        """)
    end
end

# TODO: remove this inefficient glue code
cxx"""
namespace legacy {
libfreenect2::FrameMap getFrameMap(
    libfreenect2::SyncMultiFrameListener* listener) {
    libfreenect2::FrameMap frames;
    listener->waitForNewFrame(frames);
    return frames;
}
}
"""
function waitForNewFrame(listener::SyncMultiFrameListenerPtr)
    frames = icxx"""return legacy::getFrameMap($(listener.handle).get());"""
    return FrameMap(frames)
end

function release(listener::SyncMultiFrameListenerPtr, jlframes::FrameMap)
    frames = jlframes.handle
    # really ugly
    icxx"""
    libfreenect2::FrameMap cxxframes;
    for (libfreenect2::FrameMap::iterator it = $frames.begin();
         it != $frames.end(); ++it) {
        cxxframes[it->first] = it->second;
    }
    $(listener.handle)->release(cxxframes);
    """
end

hasNewFrame(listener::SyncMultiFrameListenerPtr) =
    icxx"$(listener.handle)->hasNewFrame()"

"""libfreenect2::Freenect2"""
typealias Freenect2 cxxt"libfreenect2::Freenect2"
(::Type{Freenect2})() = icxx"libfreenect2::Freenect2();"

for name in [
    :enumerateDevices,
    ]
    ex = Expr(:macrocall, Symbol("@icxx_str"), "\$f.$(name)();")
    @eval $name(f::Freenect2) = $ex
end

function openDefaultDevice(f::Freenect2, pipeline=Union{})
    if is(pipeline, Union{})
        icxx"$f.openDefaultDevice();"
    else
        icxx"$f.openDefaultDevice($pipeline);"
    end
end

function getDeviceSerialNumber(f::Freenect2, idx)
    unsafe_string(icxx"$f.getDeviceSerialNumber($idx);")
end

function getDefaultDeviceSerialNumber(f::Freenect2)
    unsafe_string(icxx"$f.getDefaultDeviceSerialNumber();")
end

function openDevice(f::Freenect2, name, pipeline=Union{})
    arg1 = isa(name, AbstractString) ? pointer(name) : name

    if is(pipeline, Union{})
        icxx"$f.openDevice($arg1);"
    else
        icxx"$f.openDevice($arg1, $pipeline);"
    end
end

"""libfreenect2::Freenect2Device*"""
const Freenect2DevicePtr = pcpp"libfreenect2::Freenect2Device"

function getSerialNumber(device::Freenect2DevicePtr)
    unsafe_string(icxx"$device->getSerialNumber();")
end

function getFirmwareVersion(device::Freenect2DevicePtr)
    unsafe_string(icxx"$device->getFirmwareVersion();")
end

import Base: start, close

for f in [
    :getColorCameraParams,
    :getIrCameraParams,
    ]
    ex = Expr(:macrocall, Symbol("@icxx_str"), "\$device->$f();")
    @eval $f(device::Freenect2DevicePtr) = $ex
end

for f in [
    :start,
    :stop,
    :close,
    ]
    cxxcall = Expr(:macrocall, Symbol("@icxx_str"), "\$device->$f();")
    @eval begin
        function $f(device::Freenect2DevicePtr)
            r = $cxxcall
            r || error("problem happens in device operation")
        end
    end
end

function setColorFrameListener(device::Freenect2DevicePtr,
        listener::SyncMultiFrameListenerPtr)
    icxx"$device->setColorFrameListener($(listener.handle).get());"
end

function setIrAndDepthFrameListener(device::Freenect2DevicePtr,
        listener::SyncMultiFrameListenerPtr)
    icxx"$device->setIrAndDepthFrameListener($(listener.handle).get());"
end

const ColorCameraParams = cxxt"libfreenect2::Freenect2Device::ColorCameraParams"
const IrCameraParams = cxxt"libfreenect2::Freenect2Device::IrCameraParams"

type Registration
    handle::cxxt"std::shared_ptr<libfreenect2::Registration>"

    function Registration(irparams::IrCameraParams, cparams::ColorCameraParams)
        p = icxx"std::shared_ptr<libfreenect2::Registration>(
            new libfreenect2::Registration($irparams, $cparams));"
    new(p)
    end
end

function apply{T<:FramePtr}(registration::Registration, color::T, depth::T,
    undistorted::T, registered::T; enable_filter::Bool=true)
    icxx"$(registration.handle)->apply($(color.handle), $(depth.handle),
        $(undistorted.handle), $(registered.handle), $enable_filter);"
end

cxx"""
inline uint8_t unsafe_f2uint8(float& rgb, size_t idx) {
    const uint8_t *p = reinterpret_cast<uint8_t*>(&rgb);
    return p[idx];
}
"""
function getPointXYZRGB{T<:FramePtr}(registration::Registration, undistorted::T,
    registered::T, r::Integer, c::Integer)
    uframe = undistorted.handle
    rframe = registered.handle
    x = Ref{Cfloat}()
    y = Ref{Cfloat}()
    z = Ref{Cfloat}()
    rgb = Ref{Cfloat}()
    icxx"""$(registration.handle)->getPointXYZRGB($uframe, $rframe,
        $r, $c, $x, $y, $z, $rgb);"""
    rgbval = rgb[]
    b = @cxx unsafe_f2uint8(rgbval, 0)
    g = @cxx unsafe_f2uint8(rgbval, 1)
    r = @cxx unsafe_f2uint8(rgbval, 2)
    return x[], y[], z[], r, g, b
end

# deprecates
@deprecate SyncMultiFrameListener SyncMultiFrameListenerPtr
@deprecate FrameMapContainer FrameMap
@deprecate FrameContainer FramePtr

end # module Libfreenect2
