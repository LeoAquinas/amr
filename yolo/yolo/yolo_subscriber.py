import yaml
from yaml.loader import SafeLoader

# Load the YAML file
with open("/home/heisenburg/agv/src/yolo/coco.yaml", mode ='r') as f:
    data = yaml.load(f, Loader = SafeLoader)

print(data['names'][0])