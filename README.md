# Multi_sensor_fusion

---

## Algorithm Steps
1. **Loading Images** – Read ArUco marker & poster image using `cv2.imread()`.
2. **Detecting ArUco Markers** – Use `cv2.aruco` dictionary & `detectMarkers()` to get corner points.
3. **Detecting Poster Corners** – Compute center points & scaled poster corners.
4. **Perspective Transformation** – Align poster using `cv2.getPerspectiveTransform()` & `cv2.warpPerspective()`.
5. **Masking** – Create mask with `np.zeros()` & `cv2.fillPoly()`.
6. **Bitwise Operations** – Combine images with `cv2.bitwise_and()` & `cv2.bitwise_or()`.
7. **Evaluation** – Compare perfectly aligned vs misaligned outputs.

---

##  Results

###  Perfectly Aligned Images
- Poster edges align accurately with wall edges.

###  Misaligned Images
- Caused by:
  - Poor marker detection  
  - Perspective distortion  
  - Environmental lighting conditions

---

##  Technologies & Libraries
- **Python 3.x**
- **OpenCV (cv2)**
- **NumPy**

---


##  How to Run

```bash
# Clone this repository
git clone https://github.com/<your-username>/<repo-name>.git

# Move into project folder
cd <repo-name>

# Install dependencies
pip install opencv-python numpy

# Run the script
python augmented_reality.py
