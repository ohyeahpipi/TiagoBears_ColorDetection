import cv2
import numpy as np


def Desktop_Find():
    def cv2_imread():
        # img path
        image_path = "./cubes/cubes_light.png"
        # read img
        image = cv2.imread(image_path)
        # scale img
        width = int(image.shape[0] / 1)
        height = int(image.shape[1] / 1)
        image = cv2.resize(image, (height, width), interpolation=cv2.INTER_CUBIC)
        return image

    def grey_img(img):
        # get grey img
        grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # equalize of the histogram, enhancement contrast
        dst = cv2.equalizeHist(grey)

        # HRESH_BINARY
        ret,thresh1 = cv2.threshold(grey,127,255,cv2.THRESH_BINARY)
        # THRESH_BINARY_INV
        ret,thresh1 = cv2.threshold(grey,127,200,cv2.THRESH_BINARY_INV)
        # THRESH_TRUNC
        ret,thresh1 = cv2.threshold(grey,127,255,cv2.THRESH_TRUNC)
        # THRESH_TOZERO
        ret,thresh1 = cv2.threshold(grey,127,255,cv2.THRESH_TOZERO)
        # THRESH_TOZERO_INV
        ret,thresh1 = cv2.threshold(grey,127,255,cv2.THRESH_TOZERO_INV)

        grey_img = grey
        return grey_img

   
    
    def edge_find(img):
        blurred = cv2.GaussianBlur(img,(11,11),0)
        Kanten = cv2.Canny(blurred,10,20)
        # dilation
        #dilation = cv2.dilate(Kanten,None,iterations=2)
        # erosion
        #erosion = cv2.erode(Kanten,None,iterations=1)
        
        board = Kanten
        return board

    def black_board(img,edge_img):
        lines = cv2.HoughLinesP(edge_img,1,np.pi/180,threshold=12,minLineLength=55,maxLineGap=6)
        for line in lines: 
            x1, y1, x2, y2 = line[0] 
            cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
        return img


    


    

    img = cv2_imread()
    process1 = grey_img(img)
    process2 = edge_find(process1)
    process3 = black_board(img,process2)
    # show img
    cv2.imshow('process2', process2)
    cv2.imshow('process3', process3)
    cv2.waitKey(0)
    cv2.destroyAllWindows()





if __name__ == '__main__':
    Desktop_Find()