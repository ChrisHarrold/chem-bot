import json
from watson_developer_cloud import VisualRecognitionV3

visual_recognition = VisualRecognitionV3(
    '2016-05-20',
    api_key='6c9bd2eebc2ca826e9ef864eb8934fdd4c5d259a')

classifiers = visual_recognition.list_classifiers(verbose=True)
print(json.dumps(classifiers, indent=2))
