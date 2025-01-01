from ultralytics import YOLO
import os
import torch
from distutils.dir_util import copy_tree
import shutil

def get_train_device(multi_gpu=False):
    if torch.cuda.is_available():
        devices = torch.cuda.device_count()
        if devices > 1 and multi_gpu:
            list_devices = []
            for i in range(devices):
                list_devices.append(i)
            return list_devices
        else:
            return 0
    return "cpu"

def update_latest_trained_model(result, latest_dir):
    # check if save_dir exists as class attribute
    if hasattr(result, "save_dir"):
        save_dir = result.save_dir.__str__()

        # remove latest directory if exists
        if os.path.exists(latest_dir):
            shutil.rmtree(latest_dir)
            os.makedirs(latest_dir)
        
        copy_tree(save_dir, latest_dir)

        with open(f"{latest_dir}/source.txt", "w") as f:
            f.write(save_dir)
        
def main():
    current_dir = os.path.dirname(os.path.realpath(__file__))
    current_working_dir = os.getcwd()

    if current_dir != current_working_dir:
        os.chdir(current_dir)

    data_dir = f"{current_dir}/dataset-latest/"
    dataset = f"{data_dir}/data.yaml"
    latest_dir = f"{current_dir}/train-latest/"
    device = get_train_device(multi_gpu=False)

    # Check if data.yaml exists
    if not os.path.exists(dataset):
        raise Exception("data.yaml not found. Please run download.py first.")

    model = YOLO(f"yolov8s-seg.pt")

    # Train
    results = model.train(
        data=dataset, 
        device=device,
        imgsz=(640, 480), 
        epochs=300, 
        batch=16,
        patience=10,                             # Early stopping patience
        # augment=True,                          # Use augmentations, source: https://docs.ultralytics.com/modes/train/#augmentation-settings-and-hyperparameters
        # hsv_h=0.015,                           # Hue augmentation (fraction)
        # hsv_s=0.7,                             # Saturation augmentation (fraction)
        # hsv_v=0.4,                             # Value augmentation (fraction)
        # degrees=10,                            # Image rotation (degrees)
        # translate=0.1,                         # Image translation (fraction)
        # scale=0.5,                             # Image scaling (fraction)
        # shear=0.5,                             # Image shearing (fraction)
        # perspective=0.0,                       # Perspective effect (fraction)
        # flipud=0.0,                            # Vertical flip probability
        # fliplr=0.5,                            # Horizontal flip probability
        # mosaic=1.0,                            # Mosaic augmentation probability
        # mixup=0.5                              # MixUp augmentation probability
    )
    
    update_latest_trained_model(results, latest_dir)

if __name__ == "__main__":
    main()