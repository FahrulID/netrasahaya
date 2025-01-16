from roboflow import Roboflow
import os
from dotenv import load_dotenv
from distutils.dir_util import copy_tree
import shutil

load_dotenv()

def update_latest_dataset(downloaded_location, latest_location):
    # remove latest directory if exists
    if os.path.exists(latest_location):
        shutil.rmtree(latest_location)
        os.makedirs(latest_location)
    
    copy_tree(downloaded_location, latest_location)

    with open(f"{latest_location}/source.txt", "w") as f:
        f.write(downloaded_location)

def main():
    current_dir = os.path.dirname(os.path.realpath(__file__))

    api_key = os.getenv("ROBOFLOW_API_KEY")
    workspace = os.getenv("ROBOFLOW_WORKSPACE")
    project = os.getenv("ROBOFLOW_PROJECT")
    version = os.getenv("ROBOFLOW_VERSION")

    dataset_location = f"{current_dir}\dataset\{project}-{version}"

    rf = Roboflow(api_key=api_key)
    project = rf.workspace(workspace).project(project)
    version = project.version(version)
    dataset = version.download(model_format="yolov8", location=dataset_location, overwrite=True)

    latest_location = f"{current_dir}/dataset-latest/"
    update_latest_dataset(dataset_location, latest_location)

if __name__ == "__main__":
    main()