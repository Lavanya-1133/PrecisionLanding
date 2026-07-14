import cv2
import numpy as np
import os

# ── Checkerboard config ──────────────────────────────
CHECKERBOARD   = (5, 7)      # inner corners (cols, rows)
SQUARE_SIZE_MM = 36.2        # mm per square
MIN_CAPTURES   = 20          # minimum good captures needed
DEVICE_INDEX   = 0
WIDTH, HEIGHT  = 640, 480
# ─────────────────────────────────────────────────────

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE_MM

objpoints = []   # 3D world points
imgpoints = []   # 2D image points

cap = cv2.VideoCapture(DEVICE_INDEX, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

if not cap.isOpened():
    print("❌ Cannot open camera")
    exit()

print("="*50)
print("  Camera Calibration — icSpring 640x480")
print("="*50)
print(f"  Checkerboard : {CHECKERBOARD[0]}x{CHECKERBOARD[1]} inner corners")
print(f"  Square size  : {SQUARE_SIZE_MM} mm")
print(f"  Need         : {MIN_CAPTURES} good captures")
print("-"*50)
print("  SPACE = capture  |  Q = quit & calibrate")
print("  Hold board at different angles & distances")
print("="*50)

captured = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Frame grab failed")
        break

    gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    display = frame.copy()

    found, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if found:
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        cv2.drawChessboardCorners(display, CHECKERBOARD, corners2, found)
        cv2.putText(display, "DETECTED - press SPACE to capture",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 0), 2)
    else:
        cv2.putText(display, "No pattern found - adjust board",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 0, 255), 2)

    cv2.putText(display,
                f"Captured: {captured}/{MIN_CAPTURES}",
                (10, 55), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (255, 255, 0), 2)

    cv2.putText(display,
                "SPACE=capture  Q=calibrate & save",
                (10, HEIGHT - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    cv2.imshow("Calibration", display)
    key = cv2.waitKey(1) & 0xFF

    if key == ord(' ') and found:
        objpoints.append(objp)
        imgpoints.append(corners2)
        captured += 1
        print(f"📸 Captured {captured}/{MIN_CAPTURES}")

    if key == ord('q') or key == ord('Q'):
        if captured < 10:
            print(f"❌ Need at least 10 captures, only have {captured}")
        else:
            break

cap.release()
cv2.destroyAllWindows()

if captured >= 10:
    print("\n⏳ Calibrating... please wait")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, (WIDTH, HEIGHT), None, None
    )

    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        mean_error += cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    mean_error /= len(objpoints)

    print("\n" + "="*50)
    print("  ✅ Calibration Complete!")
    print("="*50)
    print(f"  Reprojection error : {mean_error:.4f} px  (good if < 1.0)")
    print(f"  Camera matrix fx   : {mtx[0,0]:.2f}")
    print(f"  Camera matrix fy   : {mtx[1,1]:.2f}")
    print(f"  Principal point cx : {mtx[0,2]:.2f}")
    print(f"  Principal point cy : {mtx[1,2]:.2f}")
    print(f"  Distortion coeffs  : {dist.ravel()}")

    np.savez("camera_calibration.npz",
             camera_matrix=mtx,
             dist_coeffs=dist,
             img_size=(WIDTH, HEIGHT))

    print("\n✅ Saved: camera_calibration.npz")
    print("   Ready for ArUco detection!")