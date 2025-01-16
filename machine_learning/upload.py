from roboflow import Roboflow
import os
from dotenv import load_dotenv

load_dotenv()

def main():
    current_dir = os.path.dirname(os.path.realpath(__file__))

    api_key = os.getenv("ROBOFLOW_API_KEY")
    workspace = os.getenv("ROBOFLOW_WORKSPACE")
    project = os.getenv("ROBOFLOW_PROJECT")
    version = os.getenv("ROBOFLOW_VERSION")

    model_path = f"{current_dir}\\train-latest"

    rf = Roboflow(api_key=api_key)
    project = rf.workspace(workspace).project(project)
    version = project.version(version)

    version.deploy("yolov8", model_path, "weights/best.pt")

if __name__ == "__main__":
    main()