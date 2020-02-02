import cv2

image = cv2.imread('../image/ex2.png')
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

cv2.rectangle(image, (368, 197), (398, 224), (255, 0, 0), 1)
cv2.imshow('original image', image)
cv2.waitKey(0)

rect1 = hsv[315:336, 327:354, :]

h_max = rect1[:, :, 0].max()
s_max = rect1[:, :, 1].max()
v_max = rect1[:, :, 2].max()

h_min = rect1[:, :, 0].min()
s_min = rect1[:, :, 1].min()
v_min = rect1[:, :, 2].min()

print("lower = [%d, %d, %d]" % (h_min, s_min, v_min))
print("upper = [%d, %d, %d]" % (h_max, s_max, v_max))
