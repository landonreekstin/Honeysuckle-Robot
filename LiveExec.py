# Author: Landon Reekstin   Date: June 2021
# File to run live execution of evaluation of model trained in Training file using webcam. 
# This file did not reach a fully working state, but has my progress and experiments in having the webcam evaluate the pretrained model from the training file.


from jetcam.usb_camera import USBCamera
from jetcam.csi_camera import CSICamera
import threading
import time
from utils import preprocess
import torch.nn.functional as F
import torch
import torchvision
from IPython.display import display
from jetcam.utils import bgr8_to_jpeg


# camera setup
# for USB Camera (Logitech C270 webcam), uncomment the following line
camera = USBCamera(width=224, height=224, capture_device=0) # confirm the capture_device number

# for CSI Camera (Raspberry Pi Camera Module V2), uncomment the following line
# camera = CSICamera(width=224, height=224, capture_device=0) # confirm the capture_device number

camera.running = True
print("camera created")

# unobserve all callbacks from camera
camera.unobserve_all()


# Model created by Training file
model_dir = "/home/landon/nvdli-data/honeysuckle_model/classification_model.pt"

# load model
device = torch.device('cuda')
# RESNET 18
model = torchvision.models.resnet18(pretrained=True)
model.fc = torch.nn.Linear(512, len(dataset.categories))

model = model.to(device)

model.load_state_dict(torch.load(model_dir))

print("model configured")


# live execution
state = 'stop'

def live(state, model, camera):
    while state == 'live':
        image = camera.value
        preprocessed = preprocess(image)
        output = model(preprocessed)
        output = F.softmax(output, dim=1).detach().cpu().numpy().flatten()
        category_index = output.argmax()
            
def start_live(change):
    if change['new'] == 'live':
        execute_thread = threading.Thread(target=live, args=(state, model, camera))
        execute_thread.start()

print("live execution performed")