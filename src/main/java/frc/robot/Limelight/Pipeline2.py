import cv2
import numpy as np

# global variables go here:
testVar = 0

# To change a global variable inside a function,
# re-declare it the global keyword
def incrementTestVar():
    global testVar
    testVar = testVar + 1
    if testVar == 100:
        print("test")
    if testVar >= 200:
        print("print")
        testVar = 0

def drawDecorations(image):
    cv2.putText(image, 
        'Limelight python script!', 
        (0, 230), 
        cv2.FONT_HERSHEY_SIMPLEX, 
        .5, (0, 255, 0), 1, cv2.LINE_AA)
    
# runPipeline() is called every frame by Limelight's backend.
def runPipeline(image, llrobot):

    a = 50
    b =  40
    ma = 25
    mb = 30
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    #img_threshold = cv2.inRange(img_hsv, (60, 70, 70), (85, 255, 255))
    img_threshold = cv2.inRange(img_hsv, (0,a+128-ma,b+128-mb),(255, a+128+ma, b+128+mb))
#lab(50.22, 57, 42)
    contours, _ = cv2.findContours(img_threshold, 
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  
    largestContour = np.array([[]])
    llpython = [0,0,0,0,0,0,0,0]
    betterContours=[];
    for c in contours:
        x,y,w,h = cv2.boundingRect(c)
        ratio = w/h
        if ratio>=.85 and ratio <=1.25 and w*h >=300:
            betterContours.append(c)


    if len(betterContours) > 0:
        cv2.drawContours(image, betterContours, -1, 255, 2)
        largestContour = max(betterContours, key=cv2.contourArea)
        x,y,w,h = cv2.boundingRect(largestContour)

        cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,255),2)
        llpython = [1,x,y,w,h,9,8,7]  
  
    incrementTestVar()
    drawDecorations(image)
       
    # make sure to return a contour,
    # an image to stream,
    # and optionally an array of up to 8 values for the "llpython"
    # networktables array
    return largestContour, image, llpython