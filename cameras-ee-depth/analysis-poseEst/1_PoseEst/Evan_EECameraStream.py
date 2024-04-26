import cv2

# camera object
cap = cv2.VideoCapture(0)

# QR code detection - openCV
detector = cv2.QRCodeDetector()

#Infinite Data Checking loop
while True:
    
    # QR Code Image
    _, img = cap.read()
    
    # Detects cube side 'data' of QR code, and establishes the coordinates of the bounding box 
    data, bbox, _ = detector.detectAndDecode(img)
    
    # draws that blue box around the QR code and overlays the image with the data. Box coordinates in tuple(bbox)
    if(bbox is not None):
        for i in range(len(bbox)):
            cv2.line(img, (int(bbox[i][0][0]),int(bbox[i][0][1])), (int(bbox[(i+1) % len(bbox)][0][0]),int(bbox[(i+1) % len(bbox)][0][0])), color=(255,
                     0, 0), thickness=2)
        cv2.putText(img, data, (int(bbox[0][0][0]), int(bbox[0][0][1]) - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    1, (255, 250, 120), 2)
        
        # prints cube side data to terminal
        if data:
            print("data found: ", data)
            
    # Displaying live camera stream to Armpi
    cv2.imshow("code detector", img)
    
    # Adds a break using q
    if(cv2.waitKey(1) == ord("q")):
        break
    
# Destroy applications opened
cap.release()
cv2.destroyAllWindows()