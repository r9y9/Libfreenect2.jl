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

    colormat = cv2.Mat(height(color), width(color), cv2.CV_8UC4, data(color))
    irmat = cv2.Mat_{Float32}(height(ir), width(ir), data(ir))
    depthmat = cv2.Mat_{Float32}(height(depth), width(depth), data(depth))

    # resize color image since it's a bit large to draw
    colormat = cv2.resize(colormat, (width(color)/3, height(color)/3))
    cv2.imshow("color", colormat)
    cv2.imshow("ir", irmat ./ 65535.0)
    cv2.imshow("depth", depthmat ./ 4500.0)

    map(release, [color, ir, depth])

    key = cv2.waitKey(delay=1)
    isesc(key) && break
end

stop(device)
close(device)
