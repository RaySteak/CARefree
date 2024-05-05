from flask import Flask, request
import torch
import pprint
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import patches
from transformers import AutoImageProcessor, AutoModelForImageClassification, ViTForImageClassification
from PIL import Image
from io import BytesIO

DEBUG_MODE = False
USE_HTTPS = False
PORT = 5000

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

processor = AutoImageProcessor.from_pretrained("dima806/facial_emotions_image_detection")
model = AutoModelForImageClassification.from_pretrained("dima806/facial_emotions_image_detection").to(device)
model.eval()

labels_list = ['sad', 'disgust', 'angry', 'neutral', 'fear', 'surprise', 'happy']

class LoggingMiddleware(object):
    def __init__(self, app):
        self._app = app

    def __call__(self, env, resp):
        errorlog = env['wsgi.errors']
        pprint.pprint(('REQUEST', env), stream=errorlog)

        def log_response(status, headers, *args):
            pprint.pprint(('RESPONSE', status, headers), stream=errorlog)
            return resp(status, headers, *args)

        return self._app(env, log_response)

app = Flask(__name__)
app.config['MAX_CONTENT_LENGTH'] = 512 * 1024

SQR_GRAYSCALE_WIDTH = 64
SQR_GRAYSCALE_HEIGHT = 64
SQR_GRAYSCALE_LENGTH = 4096

QQQVGA_GRAYSCALE_WIDTH = 80
QQQVGA_GRAYSCALE_HEIGHT = 60
QQQVGA_GRAYSCALE_LENGTH = 4800

QQQVGA_RGB565_WIDTH = 80
QQQVGA_RGB565_HEIGHT = 60
QQQVGA_RGB565_LENGTH = 9600

QVGA_GRAYSCALE_WIDTH = 320
QVGA_GRAYSCALE_HEIGHT = 240
QVGA_GRAYSCALE_LENGTH = 76800

QVGA_RGB565_WIDTH = 320
QVGA_RGB565_HEIGHT = 240
QVGA_RGB565_LENGTH = 153600

VGA_GRAYSCALE_WIDTH = 640
VGA_GRAYSCALE_HEIGHT = 480
VGA_GRAYSCALE_LENGTH = 307200

debug_rect = None

@app.route('/debug/set_rect', methods = ['POST'])
def debug_set_rect():
    global debug_rect
    debug_rect = np.frombuffer(request.data, dtype = np.int32)
    print("RECT IS ", debug_rect)
    return 'OK'

def extract_image(data, content_type):
    if content_type == 'application/jpeg':
        img = Image.open(BytesIO(data))
        img = np.array(img)
        img = np.stack((img, img, img), axis = 2)
    elif len(data) == QVGA_RGB565_LENGTH or len(data) == QQQVGA_RGB565_LENGTH:
        height, width = {
            QVGA_RGB565_LENGTH : (QVGA_RGB565_HEIGHT, QVGA_RGB565_WIDTH),
            QQQVGA_RGB565_LENGTH : (QQQVGA_RGB565_HEIGHT, QQQVGA_RGB565_WIDTH)
            }[len(data)]
        img = np.zeros((height, width, 3), dtype = np.uint8)
        for i in range(0, len(data), 2):
            high = data[i]
            low = data[i+1]
            # converting to [0, 255] uint8
            r = high & 0b11111000
            g = ((high & 0b111) << 5) | (low >> 3)
            b = (low & 0b11111) << 3
            img[i // 2 // width, i // 2 % width] = [r, g, b]
    elif len(data) == QVGA_GRAYSCALE_LENGTH or len(data) == QQQVGA_GRAYSCALE_LENGTH or len(data) == SQR_GRAYSCALE_LENGTH:
        img_flattened = np.frombuffer(data, dtype = np.uint8)
        img = img_flattened.reshape({
            QVGA_GRAYSCALE_LENGTH : (QVGA_GRAYSCALE_HEIGHT, QVGA_GRAYSCALE_WIDTH),
            QQQVGA_GRAYSCALE_LENGTH : (QQQVGA_GRAYSCALE_HEIGHT, QQQVGA_GRAYSCALE_WIDTH),
            SQR_GRAYSCALE_LENGTH : (SQR_GRAYSCALE_HEIGHT, SQR_GRAYSCALE_WIDTH)
            }[len(data)])
        img = np.stack((img, img, img), axis = 2)
    return img

@app.route('/debug/set_image', methods = ['POST'])
def debug_set_image():
    global debug_rect
    img = extract_image(request.data, request.headers['Content-Type'])
    plt.imshow(img)
    if debug_rect is not None:
        plt.gca().add_patch(patches.Rectangle((debug_rect[0], debug_rect[1]), debug_rect[2] - debug_rect[0], debug_rect[3] - debug_rect[1], linewidth = 1, edgecolor = 'r', facecolor = 'none'))
        debug_rect = None
    plt.show()
    plt.gca().clear()
    img = torch.Tensor(processor(img)['pixel_values'][0]).to(device)[None, :]
    with torch.no_grad():
        out = model.vit(img)
        feats = out[0][:, 0, :]
        logits = model.classifier(feats).squeeze(0)
        class_idx = torch.argmax(logits).item()
        class_name = labels_list[class_idx]
    feats_bytes = feats.squeeze(0).cpu().detach().numpy().tobytes()
    print("FIFTH VALUE IS ", feats[:, 5][0].item())
    print(class_name)
    return class_name

@app.route('/predict', methods = ['POST'])
def predict():
    img = extract_image(request.data, request.headers['Content-Type'])
    img = torch.Tensor(processor(img)['pixel_values'][0]).to(device)[None, :]
    with torch.no_grad():
        out = model.vit(img)
        feats = out[0][:, 0, :]
        logits = model.classifier(feats).squeeze(0)
        class_idx = torch.argmax(logits).item() 
        print(f"CLASS IDX IS {class_idx} ({labels_list[class_idx]})")
    return class_idx.to_bytes(1, 'little')

if __name__ == '__main__':
    if DEBUG_MODE:
        app.wsgi_app = LoggingMiddleware(app.wsgi_app)
    if USE_HTTPS:
        app.run(host = '0.0.0.0', threaded = True, port = PORT, ssl_context = 'adhoc')
    else:
        app.run(host = '0.0.0.0', threaded = True, port = PORT)