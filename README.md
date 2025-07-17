
# 📦 Evaluation of 3D Object Detector using LiDAR & YOLOv8

![Python](https://img.shields.io/badge/Python-3.x-blue?logo=python)
![Open3D](https://img.shields.io/badge/Open3D-3D%20Visualization-orange)
![YOLOv8](https://img.shields.io/badge/YOLOv8-Segmentation-red)
![Status](https://img.shields.io/badge/Status-Completed-brightgreen)

---

##  Table of Contents
1. [Project Overview](#-project-overview)
2. [Authors](#-authors)
3. [Methodology](#-methodology)
    - [Image Segmentation](#1-image-segmentation-and-car-detection-using-yolov8)
    - [LiDAR Projection](#2-projection-of-3d-lidar-point-clouds-onto-segmented-2d-images)
    - [3D Visualization](#3-visualization-of-filtered-lidar-points-in-3d-using-open3d)
    - [Integration with Bounding Boxes](#4-integration-and-visualization-of-3d-bounding-boxes-with-filtered-lidar-points)
4. [Results & Analysis](#-results--analysis)
5. [Evaluation Metrics](#-evaluation-metrics)
6. [Technologies](#-technologies)
7. [Sample Outputs](#-sample-outputs)
8. [How to Run](#-how-to-run)
9. [Future Work](#-future-work)
10. [References](#-references)

---

##  Project Overview
This project evaluates the quality of 3D object detection by analyzing how well **LiDAR point cloud data** fits within **3D bounding boxes**.  
By calculating the number of LiDAR points **inside vs. outside** each detected box, we determine the **reliability and accuracy** of the detector in realistic conditions (sensor noise, distance, reflections).  
The results help improve **object detection algorithms** and enhance **multi-sensor fusion systems**.

---

## 👨‍💻 Authors
- **Mohammed Kumail Abbas** – Matriculation No: 18743947  
- **Gokul Gandikota** – Matriculation No: 23643918  

**Guided by:** Prof. Dr. Stefan Elser  

---

## 🛠 Methodology

### **1) Image Segmentation and Car Detection using YOLOv8**
- Used **YOLOv8-seg** (`yolov8-seg.pt`) on KITTI-360 images.  
- Filtered masks for **Class ID 2 (cars)** only.  
- Overlaid segmentation masks with random colors on both blank and original images.

### **2) Projection of 3D LiDAR Point Clouds onto Segmented 2D Images**
- **Steps:**
  1. **Homogeneous Conversion:**  
     P^LiDAR_h = [X_L, Y_L, Z_L, 1]^T

  2. **Extrinsic Transformation (LiDAR → Camera):**  
     P_cam = P^LiDAR_h · T_lidar→cam

  3. **Rectification:**  
     P_rect3D = P_cam · R_rect00

  4. **Projection to 2D Image Plane:**  
     p_img_h = P_rect3D_h · P_rect00 = [u', v', s]

  5. **Normalization to Pixel Coordinates:**  
     u = u' / s, v = v' / s

  6. **Overlay:**  
     LiDAR points are overlaid on the segmented 2D image.

### **3) Visualization of Filtered LiDAR Points in 3D using Open3D**
- Each LiDAR point was colored based on its corresponding 2D pixel color.  
- Points outside image boundaries or with very low RGB intensity were assigned gray color.

### **4) Integration and Visualization of 3D Bounding Boxes with Filtered LiDAR Points**
- 3D bounding boxes (from ground truth JSON) were transformed into LiDAR coordinates.
- Dense edge points created for better visualization.  
- **Box Assignment:**  
  - Counted how many LiDAR points of each color were inside each box:  
    `inside = (point >= box_min) ∧ (point <= box_max)`  
  - Colors assigned to only one box (with max points).  
  - Boxes with no points removed from the final visualization.

---

## 📊 Results & Analysis
### ✅ **Outcomes**
- LiDAR points successfully projected and matched with bounding boxes.
- High relative point ratios (>0.5) indicate **true positive detections**.

### ⚠️ **Challenges**
1. **Multiple Cars Close to Each Other** – points may fall into the wrong box.  
2. **Surface Materials (glass, metal)** – cause ghost or missing points.  
3. **Distance Effect** – fewer points for far-away cars.

---

##  Evaluation Metrics
1. **Relative Ratio (Stability Indicator):**  
   \[
   Relative\ Ratio = \frac{Points\ Inside}{Points\ Inside + Points\ Outside}
   \]
   - Ratios > 0.5 considered **correct detections**.
2. **Absolute Counts** – help analyze distance-related detection quality.

---

## 🛠 Technologies
- **Python 3.x**  
- **YOLOv8 (Ultralytics)**  
- **OpenCV (cv2)**  
- **NumPy**  
- **Open3D**  
- **Matplotlib**  

---

##  Sample Outputs

| Segmentation | LiDAR Projection | 3D Bounding Boxes |
|--------------|------------------|-------------------|
| [Semantic segmentation
]<img width="567" height="182" alt="image" src="https://github.com/user-attachments/assets/41cb2e4e-abe8-4cbc-88da-85095b0a6397" />
| [Given ground truth table with lidar points
]<img width="562" height="188" alt="image" src="https://github.com/user-attachments/assets/55fee6ef-36a2-463a-801b-ec116b4edba7" />
 | [Creating bounding box ]<img width="567" height="282" alt="image" src="https://github.com/user-attachments/assets/1a59ce21-379d-46e9-8faf-4269dd23181f" />
 |

---

## 🚀 How to Run

```bash
# Clone this repository
git clone https://github.com/<your-username>/<repo-name>.git

# Move into project folder
cd <repo-name>

# Install dependencies
pip install ultralytics opencv-python numpy matplotlib open3d

# Run the script
python evaluation_3d_object_detector.py
