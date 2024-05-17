import numpy as np
import cv2
import matplotlib.pyplot as plt
import base64
import insightface
from insightface.app import FaceAnalysis


swapper = insightface.model_zoo.get_model('inswapper_128.onnx', download=False, download_zip=False)
analysis = FaceAnalysis(name='buffalo_l')
analysis.prepare(ctx_id=0, det_size=(640, 640))

def swap(source, destination, plot_after=True):

    np_source = np.fromstring(source, np.uint8)
    np_destination = np.fromstring(destination, np.uint8)
    img_src = cv2.imdecode(np_source, cv2.IMREAD_COLOR)
    img_des = cv2.imdecode(np_destination, cv2.IMREAD_COLOR)
    
    src_faces = analysis.get(img_src)
    des_faces = analysis.get(img_des)
    
    img_des_copy = img_des.copy()
    
    if plot_after:
        min_faces = min(len(src_faces), len(des_faces))
        for i in range(0, min_faces):
            img_des_copy = swapper.get(img_des_copy, des_faces[i], src_faces[i], paste_back=True)

        result_img = cv2.imencode('.jpg', img_des_copy)[1].tobytes()

        result_img = base64.b64encode(result_img)

        return result_img