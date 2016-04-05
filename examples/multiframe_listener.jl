using Libfreenect2
using Cxx
using OpenCV

isesc(key) = key == 27

f = Freenect2()
device = openDefaultDevice(f, OpenGLPacketPipeline())
listener = SyncMultiFrameListenerPtr()
setIrAndDepthFrameListener(device, listener)
setColorFrameListener(device, listener)

start(device)

# NOTE: must be called after start(device)
registration = Registration(getIrCameraParams(device),
    getColorCameraParams(device))
undistorted = FramePtr(512, 424, 4, key=Libfreenect2.FRAME_DEPTH)
registered = FramePtr(512, 424, 4, key=Libfreenect2.FRAME_COLOR)

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
    colormat = cv2.Mat(colorarr)
    colormat = cv2.resize(colormat, (width(color)/3, height(color)/3))

    cv2.imshow("color", colormat)
    cv2.imshow("ir", irarr)
    cv2.imshow("depth", deptharr)

    cv2.imshow("registered", registeredarr)
    cv2.imshow("unistored", undistortedarr)

    release(listener, frames)

    key = cv2.waitKey(delay=1)
    isesc(key) && break

    # TODO: remove this
    rand() > 0.98 && gc(false)
end

stop(device)
close(device)

cv2.destroyAllWindows()
