using Libfreenect2
using Base.Test

import Libfreenect2: width, height, bytes_per_pixel, exposure, gain, gamma

@testset "Frame" begin
    frame = FramePtr(512, 424, 4)
    @test width(frame) == 512
    @test height(frame) == 424
    @test bytes_per_pixel(frame) == 4
    @test exposure(frame) == 0.0
    @test gain(frame) == 0.0
    @test gamma(frame) == 0.0
end

@testset "libfreenect2 basics" begin
    f = Freenect2()
    num_devices = enumerateDevices(f)
    @test num_devices > 0

    serial = getDefaultDeviceSerialNumber(f)
    @test !isempty(serial)

    device = openDevice(f, serial)

    firmware_version = getFirmwareVersion(device)
    @test !isempty(firmware_version)

    listener = SyncMultiFrameListenerPtr()
    setIrAndDepthFrameListener(device, listener)
    setColorFrameListener(device, listener)

    start(device)

    # NOTE: must be called after start(device)
    registration = Registration(getIrCameraParams(device),
        getColorCameraParams(device))
    # these are of the same memory layout to color stream
    undistorted = FramePtr(512, 424, 4, key=Libfreenect2.FRAME_COLOR)
    registered = FramePtr(512, 424, 4, key=Libfreenect2.FRAME_COLOR)

    frames = waitForNewFrame(listener)

    stop(device)

    color = frames[FrameType.COLOR]
    ir = frames[FrameType.IR]
    depth = frames[FrameType.DEPTH]

    apply(registration, color, depth, undistorted, registered)

    for frame in [color, ir, depth]
        @test bytes_per_pixel(frame) == 4
    end

    @test width(color) == 1920
    @test height(color) == 1080
    @test width(ir) == 512
    @test height(ir) == 424
    @test width(depth) == 512
    @test height(depth) == 424

    @testset "Convert frame to Julia array" begin
        colorarr = convert(Array{UInt8, 3}, color)
        @test size(colorarr) == (4, 1920, 1080)
        irarr = convert(Array{Float32,2}, ir)
        @test size(irarr) == (512, 424)
        deptharr = convert(Array{Float32,2}, depth)
        @test size(deptharr) == size(irarr)
    end

    release(listener, frames)
    close(device)
end
