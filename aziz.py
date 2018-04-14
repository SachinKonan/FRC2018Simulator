import cv2
import os

os.chdir(os.path.dirname(os.path.abspath(__file__)))

imgname = 'resized_game.png'

img = cv2.imread(imgname)
print('Shape', img.shape)
imgcopy = img
start = []
end = []
selecting = False

def eventHandler(event, x, y, k, s):
    global selecting, imgcopy, img, start, end

    if(event == cv2.EVENT_LBUTTONDOWN):
        imgcopy = img.copy()
        start = [x,y]
        end = []
        selecting = True

    elif(event == cv2.EVENT_MOUSEMOVE):
        if(selecting):
            imgcopy = img.copy()
            end = [x,y]
            cv2.rectangle(imgcopy, (start[0], start[1]), (end[0], end[1]), (255, 0, 0), 1)

    else:
        if(event == cv2.EVENT_LBUTTONUP):
            if(selecting):
                if(len(start) is 2 and len(end) is 2):
                    cv2.rectangle(imgcopy, (start[0], start[1]), (end[0], end[1]), (255, 0, 0), 1)
                selecting = False
                print('Are you Satisfied w/ the box? press y or n')

cv2.namedWindow("img")
cv2.setMouseCallback("img", eventHandler)

while True:
    cv2.imshow("img", imgcopy)
    key = cv2.waitKey(1) & 0xFF
    # if the 'q' key is pressed, stop the loop

    if(key == ord("y")):
        #print(imgname.split('.')[0] + ','+'images/' + imgname + ',' + '%s,%s,%s,%s' %(start[0], start[1], end[0] - start[0], end[1] - start[1]))
        print('x, y, w, h')
        print('%s, %s, %s, %s' % (start[1], start[0], end[1] - start[1], end[0] - start[0]))
        print(start, end)
    if key == ord("q"):
        break
cv2.destroyAllWindows()
