import cv2
from ultralytics import YOLO
import os
import torch
import cv2.dnn
from ultralytics.utils import ASSETS, yaml_load
from ultralytics.utils.checks import check_yaml
import numpy as np
import time
import torch

CLASSES = None
COLORS = None

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

def export_onnx(model, path):
    model.export(format="onnx", device=get_train_device(), imgsz=640)
    print(f"Exported model to {path}")

def draw_bounding_box(img, class_, confidence, x, y, x_plus_w, y_plus_h):
    label = f"{class_} ({confidence:.2f})"
    # color = COLORS[class_id]
    # random color
    color = (np.random.randint(0, 256), np.random.randint(0, 256), np.random.randint(0, 256))
    cv2.rectangle(img, (x, y), (x_plus_w, y_plus_h), color, 2)
    cv2.putText(img, label, (x - 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

def ultralytics_inference(model, frame, visualize=True):
    results = model(frame, conf=0.6)

    for result in results:
        masks = result.masks  # Masks object for segmentation masks outputs

        if masks is None:
            continue

        for i, mask in enumerate(masks):
            if mask.xy is None or len(mask.xy) == 0:
                continue

            if mask.data is None or len(mask.data) == 0:
                continue

            # # Get the class-specific color
            cls = CLASSES[(i + 1) % len(CLASSES)]
            color = COLORS[i % len(COLORS)]

            mask_frame = np.zeros(frame.shape, dtype=np.uint8)
            contour = mask.xy.pop()
            contour = contour.astype(np.int32)
            contour = contour.reshape(-1, 1, 2)
            _ = cv2.drawContours(mask_frame, [contour], -1, color, cv2.FILLED)

            semantic = mask.data
            semantic_mask = semantic.cpu().squeeze(0).numpy()

            if visualize:
                # apply mask to frame with color
                frame = cv2.addWeighted(frame, 1, mask_frame, 0.5, 0)
                cv2.imshow(f"Semantic {cls}", semantic_mask)
                cv2.imshow(f"Contour {cls}", mask_frame)

    if visualize:
        cv2.imshow("Inference", frame)

def opencv_inference(net, frame):
    global CLASSES, COLORS
    [height, width, _] = frame.shape

    # Prepare a square image for inference
    length = max((height, width))
    image = np.zeros((length, length, 3), np.uint8)
    image[0:height, 0:width] = frame

    # Calculate scale factor
    scale = length / 640

    # Preprocess the image and prepare blob for net
    blob = cv2.dnn.blobFromImage(image, scalefactor=1 / 255, size=(640, 640), swapRB=True)
    net.setInput(blob)

    output_names = net.getUnconnectedOutLayersNames()

    # Perform inference / forward pass
    start = time.time()
    output = net.forward(output_names)
    end = time.time()

    print(f"Time taken: {end - start:.2f}s")
    print(f"Output shape: {output[0].shape} {output[1].shape}, output names: {output_names}, size: {len(output)}")
    

    # output = output[0]  # Remove batch dimension

    # # Get the number of classes, height, and width from the output
    # num_classes, height, width = output.shape

    # for i in range(num_classes):
    #     class_map = output[i]
    #     class_map = cv2.resize(class_map, (width, height))
    #     confidence = np.max(class_map)

    #     cv2.imshow(f"Class {i}", class_map)
    #     print(f"Class {i} confidence: {confidence}")

    #     # Threshold the class map

    # # Display the result
    # cv2.imshow("Segmented Image", image)
    

def main():
    global CLASSES, COLORS

    current_dir = os.path.dirname(os.path.realpath(__file__))
    current_working_dir = os.getcwd()

    if current_dir != current_working_dir:
        os.chdir(current_dir)

    model_path = f"{current_dir}\\train-latest\weights\\best.pt"
    dataset_path = f"{current_dir}\\dataset-latest\\data.yaml"

    if not os.path.exists(model_path):
        raise Exception("best.pt not found. Please run train.py first.")
    
    if not os.path.exists(dataset_path):
        raise Exception("data.yaml not found. Please run download.py first.")
    
    yolo_model = YOLO(model=model_path)

    CLASSES = yaml_load(check_yaml(dataset_path))["names"]
    COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

    onnx_path = f"{current_dir}\\train-latest\weights\\best.onnx"

    if not os.path.exists(onnx_path):
        export_onnx(yolo_model, onnx_path)

    input_source = f"E:\\video6.mp4"  # Webcam by default
    net: cv2.dnn.Net = cv2.dnn.readNetFromONNX(onnx_path)

    if(cv2.cuda.getCudaEnabledDeviceCount() > 0):
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
    else:
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    cap = None

    # Process the input source
    if input_source.isdigit():
        cap = cv2.VideoCapture(int(input_source))  # Webcam
    else:
        cap = cv2.VideoCapture(input_source)  # Video or image

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
            
        # opencv_inference(net, frame)
        ultralytics_inference(yolo_model, frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
