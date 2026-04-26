import cv2 as cv 


#Find out what the aruco markers from rulebook are and update from there

# Parameter Name	    Marker Size	    Number of Markers	
#cv.aruco.DICT_4X4_50	 4×4	                50	
#cv.aruco.DICT_4X4_100	 4×4	                100	
#cv.aruco.DICT_4X4_250	 4×4	                250	
#cv.aruco.DICT_4X4_1000	 4×4	                1000	
#cv.aruco.DICT_5X5_50	 5×5	                50	
#cv.aruco.DICT_5X5_100	 5×5	                100	
#cv.aruco.DICT_5X5_250	 5×5	                250	
#cv.aruco.DICT_5X5_1000	 5×5	                1000	
#cv.aruco.DICT_6X6_50	 6×6	                50	
#cv.aruco.DICT_6X6_100	 6×6	                100	
#cv.aruco.DICT_6X6_250	 6×6	                250	
#cv.aruco.DICT_6X6_1000	 6×6	                1000	
#cv.aruco.DICT_7X7_50	 7×7	                50	
#cv.aruco.DICT_7X7_100	 7×7	                100	
#cv.aruco.DICT_7X7_250	 7×7	                250	
#cv.aruco.DICT_7X7_1000	 7×7	                1000


"""
Current plan
    1) Fetch web feed from camera through this file
    2) Detect and decode aruco 
    3) Send ROS message of the decoded marker to GUI 
    4) Send virtual camera stream to webRTC 

"""
#-------------------------------------------------------------------------------#

def main():                                  #Above are parameters for this arguement
    arucoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_1000)
    arucoParams = cv.aruco.DetectorParameters()
    cap = cv.VideoCapture(0)
    #If no access to the camera 
    if not cap.isOpened():
        print("Cannot open camera")


    
    while True:
        #Initialize camera 
        ret, frame = cap.read()

        if not ret:
            print("Problem receiving frame from camera.")
            break

        #Read image and convert to black and white 
        grayImage = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        
        (corners, ids, rejected) = cv.aruco.detectMarkers(grayImage, arucoDict,
	    parameters=arucoParams)



        detectionImage = cv.cvtColor(grayImage, cv.COLOR_GRAY2BGR)



                                    #change frame --> detectionImage for greyscale 
        cv.aruco.drawDetectedMarkers(frame, corners, ids)
        #Do we want feed to be in color? Change to fram 
        cv.imshow("Detection (q to quit)", frame)


        #to get out of video 
        if cv.waitKey(1) == ord('q'):
            break

main()