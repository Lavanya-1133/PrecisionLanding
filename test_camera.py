import cv2
import time

# Camera settings
DEVICE_INDEX = 0        # /dev/video0
WIDTH        = 640
HEIGHT       = 480
FPS          = 30

def main():
    print("="*45)
    print("  Hiwonder icSpring Camera Test")
    print("="*45)

    cap = cv2.VideoCapture(DEVICE_INDEX, cv2.CAP_V4L2)

    # Set properties
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)  # reduce latency

    if not cap.isOpened():
        print("❌ ERROR: Cannot open /dev/video0")
        print("   Check: ls /dev/video*")
        return

    # Read back actual values
    actual_w   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)

    print(f"✅ Camera opened successfully")
    print(f"   Resolution : {actual_w} x {actual_h}")
    print(f"   FPS        : {actual_fps}")
    print(f"   Device     : /dev/video0 (icSpring)")
    print("-"*45)
    print("   Press  Q  to quit")
    print("   Press  S  to save a snapshot")
    print("-"*45)

    frame_count = 0
    fps_display = 0.0
    t_start     = time.time()
    snapshot_n  = 0

    while True:
        ret, frame = cap.read()

        if not ret:
            print("❌ Failed to grab frame")
            break

        frame_count += 1

        # Calculate live FPS every 30 frames
        if frame_count % 30 == 0:
            elapsed     = time.time() - t_start
            fps_display = 30 / elapsed
            t_start     = time.time()

        # Draw overlay info on frame
        overlay = frame.copy()

        cv2.putText(overlay,
                    f"icSpring Camera  {actual_w}x{actual_h}",
                    (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0), 2)

        cv2.putText(overlay,
                    f"FPS: {fps_display:.1f}",
                    (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0), 2)

        cv2.putText(overlay,
                    f"Frame: {frame_count}",
                    (10, 85),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0), 2)

        cv2.putText(overlay,
                    "Q=quit  S=snapshot",
                    (10, HEIGHT - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 0), 1)

        cv2.imshow("Hiwonder icSpring - Camera Test", overlay)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q') or key == ord('Q'):
            print("\n✅ Quit by user")
            break

        if key == ord('s') or key == ord('S'):
            snapshot_n += 1
            filename = f"snapshot_{snapshot_n:03d}.jpg"
            cv2.imwrite(filename, frame)
            print(f"📸 Snapshot saved: {filename}")

    cap.release()
    cv2.destroyAllWindows()
    print(f"\n📊 Total frames captured : {frame_count}")
    print("Camera released. Done.")

if __name__ == "__main__":
    main()