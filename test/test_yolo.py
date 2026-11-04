from ultralytics import YOLO

model = YOLO("yolo26n.pt")  # load a pretrained YOLOv8n model

results = model("https://ultralytics.com/images/bus.jpg")

for result in results:
    xywh = result.boxes.xywh  # 中心点 x、中心点 y、宽度、高度
    xywhn = result.boxes.xywhn  # normalized
    xyxy = result.boxes.xyxy  # 左上角 x、左上角 y、右下角 x、右下角 y
    xyxyn = result.boxes.xyxyn  # normalized
    names = [result.names[cls.item()] for cls in result.boxes.cls.int()]  # 每个框的类别名称
    confs = result.boxes.conf  # 每个框的置信度分数
  
print("Bounding boxes (xyxy):", xyxy)
print("Class names:", names)
print("Confidence scores:", confs)