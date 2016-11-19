using Libfreenect2
using Cxx
using CVCore
using CVHighGUI
using CVImgProc

isesc(key) = key == 27

f = Freenect2()
num_devices = enumerateDevices(f)
if num_devices <= 0
    error("No device!")
end
device = openDefaultDevice(f, OpenCLPacketPipeline())
listener = SyncMultiFrameListenerPtr()
setIrAndDepthFrameListener(device, listener)
setColorFrameListener(device, listener)

start(device)

# NOTE: must be called after start(device)
registration = Registration(getIrCameraParams(device),
    getColorCameraParams(device))
undistorted = FramePtr(512, 424, 4, key=Libfreenect2.FRAME_DEPTH)
registered = FramePtr(512, 424, 4, key=Libfreenect2.FRAME_COLOR)

try
    while true
        frames = waitForNewFrame(listener)
        color = frames[FrameType.COLOR]
        ir = frames[FrameType.IR]
        depth = frames[FrameType.DEPTH]

        apply(registration, color, depth, undistorted, registered)

        # Convert to Julia Array
        colorarr = convert(Array{UInt8,3}, color)
        irarr = convert(Array{Float32,2}, ir)
        deptharr = convert(Array{Float32,2}, depth)
        registeredarr = convert(Array{UInt8,3}, registered)
        undistortedarr = convert(Array{Float32,2}, undistorted)

        # Scale array to range [0,1]
        scale!(1/65535, irarr)
        scale!(1/4500, deptharr)

        # resize color image since it's a bit large to draw
        colormat = Mat(colorarr)
        colormat = resize(colormat,(Libfreenect2.height(color)/3, Libfreenect2.width(color)/3))

        # imshow("color", colormat)
        # imshow("ir", irarr)
        imshow("depth", deptharr)

        # imshow("registered", registeredarr)
        # imshow("unistored", undistortedarr)

        release(listener, frames)

        key = waitKey(delay=1)
        isesc(key) && break

        # TODO: remove this
        rand() > 0.98 && gc(false)
    end
finally
    stop(device)
    close(device)
    destroyAllWindows()
end
