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
while true
    !hasNewFrame(listener) && continue

    frames = waitForNewFrame(listener)
    color = frames[FrameType.COLOR]
    ir = frames[FrameType.IR]
    depth = frames[FrameType.DEPTH]

    # Convert to Julia Array
    colorarr = convert(Array, color)
    irarr = convert(Array, ir)
    deptharr = convert(Array, depth)

    # Scale array to range [0,1]
    scale!(1/65535.0, irarr)
    scale!(1/4500.0, deptharr)

    # resize color image since it's a bit large to draw
    colormat = cv2.Mat(colorarr)
    colormat = cv2.resize(colormat, (width(color)/3, height(color)/3))

    cv2.imshow("color", colormat)
    cv2.imshow("ir", irarr)
    cv2.imshow("depth", deptharr)

    map(release, [color, ir, depth])

    key = cv2.waitKey(delay=1)
    isesc(key) && break
end

stop(device)
close(device)
