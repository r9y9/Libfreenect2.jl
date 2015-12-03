module Libfreenect2

export
    # Freenect2
    Freenect2,
    enumerateDevices,
    getDeviceSerialNumber,
    getDefaultDeviceSerialNumber,
    openDevice,
    openDefaultDevice,

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

    FrameType,

    # Frame
    timestamp,
    sequence,
    width,
    height,
    bytes_per_pixel,
    data,
    exposure,
    gain,
    gamma,
    asarray


VERBOSE = true

# Load dependency
VERBOSE && info("Loading deps.jl")
deps = joinpath(dirname(@__FILE__), "..", "deps", "deps.jl")
if isfile(deps)
    include(deps)
else
    error("Libfreenect2 not properly installed. Please run Pkg.build(\"Libfreenect2\")")
end

import Base: getindex, start, close, gamma

VERBOSE && info("Loading Cxx.jl...")
using Cxx

# TODO: should be implemented in CxxStd?
using CxxStd
getindex(map::CxxStd.StdMap, key) = icxx"$map[$key];"
key{K,V}(pair::CxxStd.Pair{K,V}) = K
getindex(pair::CxxStd.Pair, key) = icxx"$pair[$key]"

import Cxx: CppEnum

VERBOSE && info("dlopen...")
for lib in [libfreenect2]
    p = Libdl.dlopen_e(lib, Libdl.RTLD_GLOBAL)
    p == C_NULL && warn("Failed to load: $lib")
end

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

"""libfreenect2::Freenect2
"""
const Freenect2 = cxxt"libfreenect2::Freenect2"
function Freenect2()
    @cxx libfreenect2::Freenect2()
end

for name in [
    :enumerateDevices,
    :openDefaultDevice
    ]
    @eval begin
        function $name(f::Freenect2)
            @cxx f->$name()
        end
    end
end

function getDeviceSerialNumber(f::Freenect2, idx)
    s = @cxx f->getDeviceSerialNumber(idx)
    bytestring(s)
end

function getDefaultDeviceSerialNumber(f::Freenect2)
    s = @cxx f->getDefaultDeviceSerialNumber()
    bytestring(s)
end

function openDevice(f::Freenect2, name)
    if isa(name, AbstractString)
        @cxx f->openDevice(pointer(name))
    elseif isa(name, Integer)
        @cxx f->openDevice(name)
    else
        error("device must be specified by string or integer value")
    end
end

"""libfreenect2::Freenect2Device*
"""
const pFreenect2Device = pcpp"libfreenect2::Freenect2Device"

function getSerialNumber(device::pFreenect2Device)
    s = @cxx device->getSerialNumber()
    bytestring(s)
end

function getFirmwareVersion(device::pFreenect2Device)
    s = @cxx device->getFirmwareVersion()
    bytestring(s)
end

import Base: start, close

for f in [
    :getColorCameraParams,
    :getIrCameraParams,
    :start,
    :stop,
    :close
    ]
    @eval begin
        function $f(device::pFreenect2Device)
            @cxx device->$f()
        end
    end
end

function setColorFrameListener(device::pFreenect2Device, listener)
    @cxx device->setColorFrameListener(listener)
end

function setIrAndDepthFrameListener(device::pFreenect2Device, listener)
    @cxx device->setIrAndDepthFrameListener(listener)
end

"""libfreenect2::SyncMultiFrameListener*
"""
const pSyncFrameListener = pcpp"libfreenect2::SyncMultiFrameListener"

const FRAME_COLOR = 1
const FRAME_IR    = 2
const FRAME_DEPTH = 4

module FrameType

import Cxx: CppEnum

const COLOR = CppEnum{symbol("libfreenect2::Frame::Type")}(1)
const IR = CppEnum{symbol("libfreenect2::Frame::Type")}(2)
const DEPTH = CppEnum{symbol("libfreenect2::Frame::Type")}(4)

end # module FrameType


function SyncMultiFrameListener(frame_types=FRAME_COLOR | FRAME_IR | FRAME_DEPTH)
    frame_types = unsigned(frame_types)
    @cxxnew libfreenect2::SyncMultiFrameListener(frame_types)
end

function hasNewFrame(listener)
    @cxx listener->hasNewFrame()
end

const FrameMap = cxxt"libfreenect2::FrameMap"

type FrameMapContainer
    cppframes::FrameMap
end

type FrameContainer
    cppframe
    ftype
    FrameContainer(frame, key=-1) = new(frame, key)
end

function getindex(frames::FrameMapContainer, key)
    @assert isa(key, CppEnum{symbol("libfreenect2::Frame::Type")})
    FrameContainer(getindex(frames.cppframes, key), key.val)
end

#= TODO:
for member in [
    :timestamp,
    :sequence,
    :width,
    :height,
    :bytes_per_pixel,
    :data,
    :exposure,
    :gain,
    :gamma
    ]
    @eval begin
        function $member(frame::FrameContainer)
            icxx"$(frame.cppframe)->$(member)"
        end
    end
end
=#

function timestamp(frame::FrameContainer)
    icxx"$(frame.cppframe)->timestamp;"
end

function sequence(frame::FrameContainer)
    icxx"$(frame.cppframe)->sequence;"
end

function width(frame::FrameContainer)
    w = icxx"$(frame.cppframe)->width;"
    signed(w)
end

function height(frame::FrameContainer)
    h = icxx"$(frame.cppframe)->height;"
    signed(h)
end

function bytes_per_pixel(frame::FrameContainer)
    b = icxx"$(frame.cppframe)->bytes_per_pixel;"
    signed(b)
end

function data(frame::FrameContainer)
    icxx"$(frame.cppframe)->data;"
end

function exposure(frame::FrameContainer)
    icxx"$(frame.cppframe)->exposure;"
end

function gain(frame::FrameContainer)
    icxx"$(frame.cppframe)->gain;"
end

function gamma(frame::FrameContainer)
    icxx"$(frame.cppframe)->gamma;"
end

function asarray(frame::FrameContainer; do_reshape=true)
    data_ptr = data(frame)::Ptr{UInt8}

    if frame.ftype == FRAME_COLOR
        total_length = width(frame) * height(frame) * 4
        array = pointer_to_array(data_ptr, total_length)
        if do_reshape
            array = reshape(array, 4, width(frame), height(frame))
        end
        return array
    elseif frame.ftype == FRAME_IR || frame.ftye == FRAME_DEPTH
        total_length = width(frame) * height(frame) * 1
        array = pointer_to_array(convert(Ptr{Float32}, data_ptr), total_length)
        if do_reshape
            array = reshape(array, width(frame), height(frame))
        end
        return array
    else
        error("annnot determine type of raw data")
    end
end

# TODO: remove this inefficient glue code
cxx"""
libfreenect2::FrameMap getFrameMap(libfreenect2::SyncMultiFrameListener* listener) {
    libfreenect2::FrameMap frames;
    listener->waitForNewFrame(frames);
    return frames;
}
"""
function waitForNewFrame(listener)
    frames = @cxx getFrameMap(listener)
    return FrameMapContainer(frames)
end

# TODO: remove this
cxx"""
void deleteFrame(libfreenect2::Frame* frame) {
    if (frame != nullptr) {
        delete frame;
        frame = nullptr;
    }
}
"""
function release(frame::FrameContainer)
    @cxx deleteFrame(frame.cppframe)
end

end # module Libfreenect2
