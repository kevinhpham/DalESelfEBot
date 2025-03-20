import cv2
import numpy as np

def adjust_levels(image):
    # Convert the image to float32
    image = image.astype(np.float32)
    
    # Normalize the image to the range 0-1
    image = (image - np.min(image)) / (np.max(image) - np.min(image))
    
    # Scale the image to the range 0-252
    image = image * 255
    
    # Convert back to uint8
    image = image.astype(np.uint8)
    
    return image


def posterize(image, k):
    # Convert the image to float32
    Z = image.reshape((-1, 3))
    Z = np.float32(Z)
    
    # Define criteria and apply k-means
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    ret, label, center = cv2.kmeans(Z, k, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
    
    # Convert back to uint8 and make the image
    center = np.uint8(center)
    res = center[label.flatten()]
    res2 = res.reshape((image.shape))
    
    return res2

# Load the pre-trained Haar Cascade classifier for face detection
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

cap = cv2.VideoCapture(1) #Change the index to that of your webcam
if cap:
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        # Resize the frame to improve processing speed
        window_width = 320
        window_height = 240
        frame = cv2.resize(frame, (window_width, window_height))
        #show webcam feed
        frame = adjust_levels(frame)
        cv2.imshow("webcam",frame)
        
        # Posterize the frame
        posterized=frame
        #posterized = posterize(frame, 3)

        # Convert the frame to grayscale
        gray = cv2.cvtColor(posterized, cv2.COLOR_BGR2GRAY)

        # Detect faces in the image
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
        
        # Draw rectangles around the faces
        for (x, y, w, h) in faces:
            #cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
            face_region = gray[y:y+h, x:x+w]
            
            # Apply Canny edge detection to the face region
            edges = cv2.Canny(face_region, 50, 100)
            
            # Replace the face region in the original frame with the edge-detected face region
            frame[y:y+h, x:x+w] = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        
        # Resize the frame back to the original window size for display
        
        # Display the resulting frame
        cv2.imshow('Face Detection with Edges', frame)
        # Display the resulting frame
        
        # Posterize the frame
        #posterized = posterize(gray, 4)
        
        # Display the resulting frame
        #cv2.imshow('posterized', posterized)
        
        # Apply Canny edge detection
        edges = cv2.Canny(gray, 140, 180)
        
        # Display the resulting frame
        cv2.imshow('Edges', edges)
        
        # Break the loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the webcam and close windows
    cap.release()
    cv2.destroyAllWindows()
else:
    print("Please connect a webcam and try again.")