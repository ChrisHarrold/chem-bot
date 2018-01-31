import json
from watson_developer_cloud import VisualRecognitionV3

visual_recognition = VisualRecognitionV3(
    '2016-05-20',
    api_key='{insert_your_key_here}')

classifiers = visual_recognition.list_classifiers(verbose=True)
print(json.dumps(classifiers, indent=2))
