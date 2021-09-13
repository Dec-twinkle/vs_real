import cv2



while(True):
    try:
        img = cv2.imread("../buffer/rgb.bmp")
        cv2.imshow("kinect capture", cv2.resize(img, (192 * 4, 108 * 4)))
        cv2.waitKey(200)
    except Exception:
        continue
