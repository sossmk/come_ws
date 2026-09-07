from ultralytics import YOLO

def train_yolo():
    model = YOLO('yolov8n.pt')  # 사용할 가중치 모델 선택

    results = model.train(
        data='C:/home/yoyoyoyo/data.yaml',  # 데이터셋 경로
        epochs=150,
        imgsz=640,
        batch=32,
        project='yolov8_project',
        name='sign',
        verbose=True,
        exist_ok=True,
        augment=True, # 데이터 증강
        iou=0.5
    )
    return results

if __name__ == '__main__':
    results = train_yolo()
    print("✅ 학습이 완료되었습니다.")
    print(results)