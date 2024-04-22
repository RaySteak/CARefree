from flask import Flask, request
import torch
import pprint
import numpy as np
from matplotlib import pyplot as plt
from transformers import AutoImageProcessor, AutoModelForImageClassification, ViTForImageClassification
from PIL import Image
from io import BytesIO

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

processor = AutoImageProcessor.from_pretrained("dima806/facial_emotions_image_detection")
model = AutoModelForImageClassification.from_pretrained("dima806/facial_emotions_image_detection").to(device)
model.eval()

labels_list = ['sad', 'disgust', 'angry', 'neutral', 'fear', 'surprise', 'happy']

# input = np.zeros((320, 240, 3))
# input = processor(input)
# print(input['pixel_values'][0].shape)

# print(type(model))

# class LoggingMiddleware(object):
#     def __init__(self, app):
#         self._app = app

#     def __call__(self, env, resp):
#         errorlog = env['wsgi.errors']
#         pprint.pprint(('REQUEST', env), stream=errorlog)

#         def log_response(status, headers, *args):
#             pprint.pprint(('RESPONSE', status, headers), stream=errorlog)
#             return resp(status, headers, *args)

#         return self._app(env, log_response)

app = Flask(__name__)
app.config['MAX_CONTENT_LENGTH'] = 512 * 1024

QVGA_GRAYSCALE_WIDTH = 320
QVGA_GRAYSCALE_HEIGHT = 240
QVGA_GRAYSCALE_LENGTH = 76800

QVGA_RGB565_WIDTH = 320
QVGA_RGB565_HEIGHT = 240
QVGA_RGB565_LENGTH = 153600

VGA_GRAYSCALE_WIDTH = 640
VGA_GRAYSCALE_HEIGHT = 480
VGA_GRAYSCALE_LENGTH = 307200

@app.route('/test', methods = ['GET'])
def test():
    return 'asdasdasd'

@app.route('/embed', methods = ['POST'])
def embed_photo():
    if request.headers['Content-Type'] == 'application/jpeg':
        img = Image.open(BytesIO(request.data))
        img = np.array(img)
        img = np.stack((img, img, img), axis = 2)
    elif len(request.data) == QVGA_RGB565_LENGTH:
        img = np.zeros((QVGA_RGB565_HEIGHT, QVGA_RGB565_WIDTH, 3), dtype = np.uint8)
        for i in range(0, QVGA_RGB565_LENGTH, 2):
            high = request.data[i]
            low = request.data[i+1]
            # converting to [0, 255] uint8
            r = high & 0b11111000
            g = ((high & 0b111) << 5) | (low >> 3)
            b = (low & 0b11111) << 3
            img[i // 2 // QVGA_RGB565_WIDTH, i // 2 % QVGA_RGB565_WIDTH] = [r, g, b]
    elif len(request.data) == QVGA_GRAYSCALE_LENGTH:
        img_flattened = np.frombuffer(request.data, dtype = np.uint8)
        img = img_flattened.reshape((QVGA_GRAYSCALE_HEIGHT, QVGA_GRAYSCALE_WIDTH))
        img = np.stack((img, img, img), axis = 2)
    plt.imshow(img)
    plt.show()
    img = torch.Tensor(processor(img)['pixel_values'][0]).to(device)[None, :]
    with torch.no_grad():
        out = model.vit(img)
        feats = out[0][:, 0, :]
        logits = model.classifier(feats).squeeze(0)
        class_idx = torch.argmax(logits).item()
        class_name = labels_list[class_idx]
    feats_bytes = feats.squeeze(0).cpu().detach().numpy().tobytes()
    print("FIFTH VALUE IS ", feats[:, 5][0].item())
    return feats_bytes

if __name__ == '__main__':
    # app.wsgi_app = LoggingMiddleware(app.wsgi_app)
    app.run(host = '0.0.0.0', threaded = True, port = 5000)