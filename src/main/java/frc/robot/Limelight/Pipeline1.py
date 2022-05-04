import cv2
import numpy as np
import math 

# global variables go here:
testVar = 0
f = 2.9272781257541/305

def center(x,y,w,h):
    return (int(x+w/2),int(y+h/2), int(w), int(h))

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

def distanceSquared(point1, point2):
    return ((point2[1] - point1[1]) * (point2[1] - point1[1]) + (point2[0] - point1[0]) * (point2[0] - point1[0]))

def AnalyzeChains(CurrentCenter, CurrentChain, betterCenters):
    p = 0
    while len(betterCenters) > p:

        # TODO instead of checking distance check horizontal and vertical distances 
        # you want mostly horizontal and only a little vertical
        testContour = betterCenters[p]
        #if distanceSquared(testContour, CurrentCenter) < 2025 :
        #if abs(testContour[1]-CurrentCenter[1])<15 and abs(testContour[0]-CurrentCenter[0])<70:
        desiredDistanceX = testContour[2] + CurrentCenter[2]
        actualDistanceX = abs(testContour[0] - CurrentCenter[0])
        desiredDistanceY = testContour[3] + CurrentCenter[3]
        actualDistanceY = abs(testContour[1] - CurrentCenter[1])

        if actualDistanceX > desiredDistanceX*.8 and actualDistanceX < desiredDistanceX*1.2 and actualDistanceY < desiredDistanceY*.8:
            CurrentChain.append(testContour)
            del betterCenters[p]
            AnalyzeChains(testContour, CurrentChain, betterCenters)

        else:
            p=p+1

def offsetFromCenter(p):
    width = 320
    height = 240
    return (p[0] - width/2 + 0.5, height / 2 - 0.5 - p[1])

def pixelToAngle(pixel,f):
    alpha = math.acos(f / (math.sqrt(pixel * pixel + f * f))) 
    if pixel < 0:
        alpha = alpha * -1
    return alpha*57.296

# runPipeline() is called every frame by Limelight's backend.
def runPipeline(mat, llrobot):
    horizontalAng = 0
    verticalAng = 0
    confidenceNumber = 0
   

    red = cv2.inRange(mat,
                (0, 0, 180),
                (255, 255, 255))

    blue= cv2.inRange(mat,(255,0,0),(255, 255, 255))


    redAndBlue = cv2.bitwise_or(red,red)
    redAndBlue = cv2.bitwise_not(redAndBlue)


    #hsv = cv2.cvtColor(mat, cv2.COLOR_BGR2HSV)
    #inRangeHSV = cv2.inRange(hsv, (61,50,145),(105, 250, 240))

    a = -35
    b =  35
    ma = 40
    mb = 40
    hsv = cv2.cvtColor(mat, cv2.COLOR_BGR2LAB)
    inRangeHSV = cv2.inRange(hsv, (100,a+128-ma,b+128-mb),(255, a+128+ma, b+128+mb)) 
    #lab(60.35, -16.06, 11.98)
    #lab(52.16, 16.59, -66.59)

    inRangeHSV = cv2.bitwise_and(redAndBlue, inRangeHSV);


    contours, _ = cv2.findContours(inRangeHSV, 
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  
    largestContour = np.array([[]])
    llpython = [0,0,0,0,0,0,0,0]
    minHeight = 3
    maxHeight = 25
    minWidth = 6
    maxWidth = 30
    betterCenters = []
    betterContours = []
    
    for c in contours:
        x,y,w,h = cv2.boundingRect(c)
        if w>= minWidth and w<= maxWidth and h >= minHeight and h <= maxHeight and w > h*.9 and w < 3*h:
            betterCenters.append(center(x,y,w,h))
            betterContours.append(c)

    cv2.drawContours(mat, contours, -1, (0,0,255), 1)  

    Chains = []
    

    maxChainLength = -1
    BestChain = []

    minX = 5000
    minY = 5000
    maxX = 0
    maxY = 0

    while len(betterCenters) > 0:
        cc = betterCenters[0]
        CurrentChain = []
        CurrentChain.append(cc)
        del betterCenters[0]
        Chains.append(CurrentChain)
        AnalyzeChains(cc, CurrentChain, betterCenters)


    for c in Chains:
        if len(c)>maxChainLength:
            maxChainLength = len(c)
            BestChain = c


    #cv2.drawContours(mat, contours, -1, new Scalar(0, 0, 255), 5);

    if len(BestChain) > 0:
        for p in BestChain:
            cv2.circle(mat, (p[0], p[1]), 10, (255,0,0), 1)
            if p[0] < minX:
                minX = p[0]
            if p[1] < minY:
                minY = p[1]

            if p[0] > maxX:
                maxX = p[0]
            if p[1] > maxY:
                maxY = p[1]

        tp = (int((minX+maxX)/2), int((minY+maxY)/2))
        offset = offsetFromCenter(tp)
        horizontalAng = pixelToAngle(offset[0], 278.503)
        verticalAng = pixelToAngle(offset[1],258.03)
        if len(BestChain) >= 3:
            cv2.circle(mat, tp, 12, (0,255,0), 1)
            
    confidenceNumber = 0
    confidence = len(BestChain) >= 3
    if confidence:
        confidenceNumber=1


    #print('the confidence is ' + str(confidence))



    #    cv2.rectangle(mat,(x,y),(x+w,y+h),(0,255,255),2)
    llpython = [confidenceNumber,horizontalAng,verticalAng,0,0,0,0,0]  
    

    #incrementTestVar()
    drawDecorations(mat)
       
    # make sure to return a contour,
    # an image to stream,
    # and optionally an array of up to 8 values for the "llpython"
    # networktables array
    return largestContour, mat, llpython