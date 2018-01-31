import json
#import picamera
from watson_developer_cloud import VisualRecognitionV3

#camera = picamera.PiCamera()
visual_recognition = VisualRecognitionV3(
    '2016-05-20',
    api_key='6c9bd2eebc2ca826e9ef864eb8934fdd4c5d259a')

def recognize():
    with open('/tmp/image.jpg', 'rb') as images_file:
        classes = visual_recognition.classify(
            images_file,
            parameters=json.dumps({
                'classifier_ids': ['ChemStorage_613927909'],
                'threshold': 0.6
            }))
    strTheDump=(json.dumps(classes))
    #print(str(strTheDump))
    if 'score' in str(strTheDump):
        return True
        print("Score Returned - proceed")
    else:
        return False
        print("Not a high enough score")

def capture():
    print("I am taking a picture! (Not really)")
    #camera.capture('/tmp/image.jpg')

def main():
    capture()
    if recognize():
        print("Go look at that stuff")
    else:
        print("Not worth checking")

main()
