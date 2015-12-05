using Libfreenect2
using Cxx
using OpenCV

isesc(key) = key == 27

f = Freenect2()
device = openDefaultDevice(f)
listener = SyncMultiFrameListener()
setIrAndDepthFrameListener(device, listener)
setColorFrameListener(device, listener)

start(device)

# NOTE: must be called after start(device)
registration = Registration(getIrCameraParams(device),
    getColorCameraParams(device))
# these are of the same memory layout to color stream
undistorted = FrameContainer(512, 424, 4, key=Libfreenect2.FRAME_COLOR)
registered = FrameContainer(512, 424, 4, key=Libfreenect2.FRAME_COLOR)

while true
    frames = waitForNewFrame(listener)
    color = frames[FrameType.COLOR]
    ir = frames[FrameType.IR]
    depth = frames[FrameType.DEPTH]

    apply(registration, color, depth, undistorted, registered)

    # Convert to Julia Array
    colorarr = convert(Array, color)
    irarr = convert(Array, ir)
    deptharr = convert(Array, depth)
    registeredarr = convert(Array, registered)

    # Scale array to range [0,1]
    scale!(1/65535.0, irarr)
    scale!(1/4500.0, deptharr)

    # resize color image since it's a bit large to draw
    colormat = cv2.Mat(colorarr)
    colormat = cv2.resize(colormat, (width(color)/3, height(color)/3))

    cv2.imshow("color", colormat)
    cv2.imshow("ir", irarr)
    cv2.imshow("depth", deptharr)
    cv2.imshow("registered", registeredarr)

    map(release, [color, ir, depth])

    key = cv2.waitKey(delay=1)
    isesc(key) && break
end

stop(device)
close(device)
