import cv2
import numpy as np
#yellow 22/33，red 0/10，green 35/77，blue 100/124

def Color_Detection(color):
    def cv2_imread():
        # img path
        image_path = "./cubes/wood_color_dice1.jpg"
        # read img
        image = cv2.imread(image_path)
        # scale img
        width = int(image.shape[0] / 2)
        height = int(image.shape[1] / 2)
        image = cv2.resize(image, (height, width), interpolation=cv2.INTER_CUBIC)
        return image

    def color_detec(img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        if color =="red":
            lower_hue = 0
            upper_hue = 10
        elif color == "yellow":
            lower_hue = 22
            upper_hue = 33  
        elif color == "green":
            lower_hue = 35
            upper_hue = 77 
        elif color == "blue":
            lower_hue = 100
            upper_hue = 124                     
        lower_hsv = np.array([lower_hue, 50, 20])
        upper_hsv = np.array([upper_hue, 255, 255])
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
        return mask
    
    def search_contours(img,mask):
        contours_count = 0
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if 200 < area < 10000:
                cv2.drawContours(img, [contour], -1, (0, 255, 0), 2)
                contours_count += 1

                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                else:
                    cX, cY = 0, 0
                cv2.circle(img, (cX, cY), 3, (255, 255, 255), -1)
                cv2.putText(img, f"{contours_count}", (cX - 25, cY - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255),
                            2)

        return contours_count

    img = cv2_imread()
    mask = color_detec(img)
    counts = search_contours(img,mask)
    # show img
    cv2.imshow('img', img)
    cv2.imshow('mask', mask)
    print("the number of "+color+" is "+str(counts))
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    Color_Detection("red")
    Color_Detection("yellow")
    Color_Detection("green")
    Color_Detection("blue")