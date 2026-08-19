from multiprocessing import freeze_support
from ultralytics import YOLO

def main():
    m = YOLO(r"C:\Users\ANT PC\wro_train\yolo26n.pt")
    m.train(data=r"C:\Users\ANT PC\Downloads\wro_dataset_2985\data.yaml",
            imgsz=320, epochs=100, patience=20, batch=64, workers=8, device=0, seed=0,
            project=r"C:\Users\ANT PC\wro_train\runs", name="y26n_320", exist_ok=True,
            hsv_h=0.015, hsv_s=0.6, hsv_v=0.4, degrees=5.0, translate=0.1,
            scale=0.5, shear=2.0, perspective=0.0, fliplr=0.5, flipud=0.0,
            mosaic=1.0, close_mosaic=10, mixup=0.0, copy_paste=0.0, plots=True)

if __name__ == "__main__":
    freeze_support()
    main()
