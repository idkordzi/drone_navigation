from ultralytics import YOLO

if __name__ == "__main__":
  # Load the YOLOv11n model
  model = YOLO("yolo11n.pt")

  # Export the model to ONNX format
  model.export(format="onnx")
