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

    @time frames = waitForNewFrame(listener)
    color = frames[FrameType.COLOR]
    ir = frames[FrameType.IR]
    depth = frames[FrameType.DEPTH]

    irarray = asarray(ir)
    color_array = asarray(color);

    map(release, [color, ir, depth])

    key = cv2.waitKey(delay=1)
    isesc(key) && break
end

stop(device)
close(device)
