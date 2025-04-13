from ultralytics import YOLO
import cv2

def main():
  
  model = YOLO("yolo11n.pt")
  model = model.eval()
  
  model.info()
  
  img = cv2.imread("src/yolo/dataset/000000017627.jpg", cv2.IMREAD_COLOR)
  
  cv2.imshow("Input preview", img)
  cv2.waitKey(0)
  
  cimg = img.copy()
  
  labels  = {}
  with open("src/yolo/labels/coco.names", "r") as f:
    for ci, line in enumerate(f.read().split('\n')):
      if len(line) > 1:
        labels[ci] = line
  
  results = model(img)
  for result in results:
    for di in range(len(result)):
      xyxy = result.boxes.xyxy[di]
      name = result.boxes.cls.int()[di].numpy()
      conf = result.boxes.conf[di].numpy()
      
      if conf < 0.5: continue
      
      xyxy = [int(el) for el in xyxy]
    
      tx = f"{labels[int(name)]}: {conf*100:0.2f}%"
      ts = cv2.getTextSize(tx, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
      
      cv2.rectangle(cimg, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), (0,0,255), 1)
      cv2.rectangle(cimg, (xyxy[0], xyxy[1]), (xyxy[0]+ts[0], xyxy[1]-ts[1]), (0,0,255), -1)
      cv2.putText(cimg, tx, (xyxy[0], xyxy[1]-1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
      
  cv2.imshow("Output preview", cimg)
  cv2.waitKey(0)
  
  cv2.destroyAllWindows()
  
  print(model.export(format="onnx"))
  

if __name__ == "__main__":
  main()
