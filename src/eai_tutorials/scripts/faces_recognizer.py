#!/usr/bin/env python
import cv2,os
import numpy as np
from PIL import Image

class Recognizer(object):
    def __init__(self):
        self.shouldMirrorPriview = True
        self.camera = cv2.VideoCapture(0)
        self.detector = cv2.CascadeClassifier('./cascades/haarcascade_frontalface_default.xml')
        self.detector.load('./cascades/haarcascade_frontalface_default.xml')
        self.save_path = 'faces'
        self.names = []
        self.recognizer = None
    
    def collectFacesData(self):
        while True:
            name = raw_input('enter your name (\'q\' for next step): ')
            if name.lower() == 'q':
                break
            sampleNum = 0
            while(True):
                ret, img = self.camera.read()
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                faces = self.detector.detectMultiScale(gray, 1.3, 5)

                for (x,y,w,h) in faces:
                    img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
                    face = cv2.resize(gray[y:y+h, x:x+w], (200,200))
                    cv2.imwrite(self.save_path +'/'+ name +'.'+ str(sampleNum) + ".pgm", face)
                    if self.shouldMirrorPriview:
                        img = np.fliplr(img).copy()
                    cv2.imshow('Collecting Faces Data',img)
                    sampleNum += 1

                if cv2.waitKey(20) & 0xff == 27: #esc
                    print 'Total '+ str(sampleNum) +' images collected for ' + name
                    break
                elif sampleNum > 19:
                    print 'Total '+ str(sampleNum) +' images collected for ' + name
                    break
        cv2.destroyAllWindows()

    def trainFaceRecognizer(self):
        while True:
            print '1: use Eigenfaces method'
            print '2: use Fishersfaces method'
            print '3: use LBPH method'
            #method = raw_input('Choose self.recognizer method (input 1, 2 or 3): ')
            method = '3'
            if method == '1':
                self.recognizer = cv2.face.EigenFaceRecognizer_create()
                break
            elif method == '2':
                self.recognizer = cv2.face.FisherFaceRecognizer_create()
                break
            elif method == '3':
                self.recognizer = cv2.face.LBPHFaceRecognizer_create()
            else:
                continue
            break
        
        print 'training face recognizer, please wait...'
        #get the path of all the files in the folder
        imagePaths=[os.path.join(self.save_path,f) for f in os.listdir(self.save_path)] 
        #create empth face list
        faceSamples=[]
        #create empty ID list
        self.Ids=[]
        #now looping through all the image paths and loading the self.Ids and the images
        for imagePath in imagePaths:
            #loading the image and converting it to gray scale
            pilImage=Image.open(imagePath)#.convert('L')
            #Now we are converting the PIL image into numpy array
            imageNp=np.array(pilImage,'uint8')

            #getting the Id from the image
            name = os.path.split(imagePath)[-1].split(".")[0]
            if name not in self.names:
                self.names.append(name)
            for Id in range(0,len(self.names)):
                if name == self.names[Id]:
                    break

            # append face in the list as well as Id of it
            faceSamples.append(imageNp)
            self.Ids.append(Id)

        self.recognizer.train(np.asanyarray(faceSamples), np.asanyarray(self.Ids))
        #self.recognizer.save('data/trainner.yml')

    def recoginzeFaces(self):
        while True:
            read, img = self.camera.read()
            faces = self.detector.detectMultiScale(img, 1.3, 5)
            for (x,y,w,h) in faces:
                img = cv2.rectangle(img, (x,y), (x+w,y+h), (255,0,0), 2)
                gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
                roi = gray[y:y+h, x:x+w]

                Id, conf = self.recognizer.predict(gray[y:y+h,x:x+w])
                print Id, conf
                if Id in self.Ids and conf>30:
                    self.name = self.names[Id]
                else:
                    self.name = 'Unknown'
                cv2.putText(img, self.name, (x,y-20), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2)

            # if self.shouldMirrorPriview:
            #     img = np.fliplr(img).copy()
            cv2.imshow("Recognizing", img)
            if cv2.waitKey(50) & 0xff == 27: #esc
                break

    def run(self):
        self.collectFacesData()
        self.trainFaceRecognizer()
        self.recoginzeFaces()

    def __del__(self):
        cv2.destroyAllWindows()
        self.camera.release()

if __name__=="__main__":
    Recognizer().run()
