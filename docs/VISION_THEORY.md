# Vision Theory — Pixel → World

Reference document. Full derivation of the mathematics used in
`CameraModel.m` and `capture_scene.m`.

---

## 1. The problem

Given an RGB image, a colour detection, and a pixel `(u, v)`,
find the 3D point `(x, y, z)` in the world. **Without a depth sensor.**

Facts we already know (not measurements):
- The table is the plane `z = z_table` (usually 0).
- The camera is calibrated and fixed.

The camera does not measure distance. It measures **direction**.
The table plane turns that direction into a point.

---

## 2. Pinhole model — a pixel is a ray

Perspective projection:

u = fx · (X_cam / Z_cam) + cx
v = fy · (Y_cam / Z_cam) + cy


In matrix form:

[u] [fx 0 cx] [X_cam / Z_cam]
[v] = [ 0 fy cy] · [Y_cam / Z_cam]
[1] [ 0 0 1] [ 1 ]
= K



`K` is the **intrinsic matrix**. For a fixed lens and resolution,
it never changes. It is the only thing FOV is used for.

### From FOV to focal length



tan(FOV_x / 2) = (W / 2) / fx
⟹ fx = (W / 2) / tan(FOV_x / 2)



Assuming square pixels: `fy = fx`, `cx = W/2`, `cy = H/2`.

### The inverse: a pixel is a ray in the camera frame


d_cam = K⁻¹ · [u; v; 1] = [ (u − cx) / fx ; (v − cy) / fy ; 1 ]


This is a **direction**. Any point `t · d_cam` (t > 0) projects
to the same pixel. Infinite possibilities. Depth unknown.

---

## 3. Case A — Vertical camera (our scene)

Assume:
- Optical axis = world `−z` (looking down).
- Camera x-axis = world x-axis.
- Camera y-axis = world `−y` (image "up" = world `+y`).
- Camera at `(Cx, Cy, h)` with `h > 0`.

### Side view


● Camera (Cx, Cy, h)
╱│
╱ │
╱ │ h
╱ θ │
╱ │
─────●─────●───────────── table, z = 0
P x = Cx
(x, y, 0)

d = |x − Cx|
tan(θ) = d / h
⟹ d = h · tan(θ)



### The angle comes from the pixel


tan(θ_x) = (u − cx) / fx
tan(θ_y) = (v − cy) / fy



Substituting into `d = h · tan(θ)` — **the tangent cancels**:


x = Cx + h · (u − cx) / fx
y = Cy − h · (v − cy) / fy ← minus sign due to image convention
z = z_table


**This is the final result for a vertical camera.**

### Worked example

| Known | Value |
|---|---|
| W × H | 640 × 480 |
| FOV | 60° ⟹ `f ≈ 554`, `cx = 320`, `cy = 240` |
| Camera height | `h = 0.5 m` |
| Detected pixel | `(u, v) = (400, 200)` |


x = (400 − 320) / 554 · 0.5 = +0.072 m
y = (240 − 200) / 554 · 0.5 = +0.036 m
z = 0


Cube at `(0.072, 0.036, 0)`.

### The y sign

The image has `v` growing **downward** (row 0 is at the top).
The world has `y` growing **upward**. Hence the minus sign.


Image World
┌───────────────┐ ┌───────────────┐
│ ▲ y↑ │ │ ▲ y↑ │
│ │ │ ⇄ │ │ │
│ └──▶ x→ │ │ └──▶ x→ │
└───────────────┘ └───────────────┘
rows grow downward y grows upward
⟹  sign flip


---

## 4. Case B — Tilted camera (general)

If the camera is tilted, each pixel's ray is **rotated**.
We must rotate it back into the world frame, then intersect
with the table plane.

### Step 1: pixel → ray in the camera frame


d_cam = K⁻¹ · [u; v; 1]
d_cam = d_cam / ‖d_cam‖ (unit vector)



### Step 2: camera → world

The camera pose in the world is `(R_wc, t_wc)`:

- `R_wc` = 3×3 rotation matrix (columns = camera axes expressed in world)
- `t_wc` = camera position in world


d_world = R_wc · d_cam



### Step 3: ray–plane intersection

The ray from the camera is:


P(s) = t_wc + s · d_world, s > 0



We want `P(s)_z = z_table`:



t_wc_z + s · d_world_z = z_table
⟹  s = (z_table − t_wc_z) / d_world_z


Therefore:



P_world = t_wc + [(z_table − t_wc_z) / d_world_z] · d_world



If `|d_world_z| < ε`, the ray is parallel to the plane — no intersection.

---

## 5. Case A is a special case of Case B

If the camera is vertical and not rotated about z:



R_wc = [ 1 0 0 ]
[ 0 −1 0 ]
[ 0 0 −1 ]

t_wc = [ Cx ; Cy ; h ]


Then:
	

d_cam = [(u−cx)/fx ; (v−cy)/fy ; 1]
d_world = R_wc · d_cam = [(u−cx)/fx ; −(v−cy)/fy ; −1]
s = (0 − h) / (−1) = h
P_world = [Cx ; Cy ; h] + h · [(u−cx)/fx ; −(v−cy)/fy ; −1]
= [Cx + h·(u−cx)/fx ;
Cy − h·(v−cy)/fy ;
0]



Exactly Case A. ✔

**This is why the code always uses Case B** — it works in both situations.

---

## 6. Full pipeline



┌──────────┐ HSV ┌───────────┐ regionprops ┌──────────────┐
│ Image │───────▶│ Colour │──────────────▶│ Pixel (u,v) │
│ 640×480 │ │ mask │ │ centroid │
└──────────┘ └───────────┘ └──────┬───────┘
│
▼
┌─────────────────────────────┐
│ CameraModel.pixel_to_world │
│ d_cam = K⁻¹ · [u;v;1] │
│ d_world = R_wc · d_cam │
│ s = −t_wc_z/d_w_z │
│ P = t_wc + s·d_w │
└──────────────┬──────────────┘
│
▼
┌───────────────┐
│ (x, y, z) │
│ → FSM │
└───────────────┘



---

## 7. Sign conventions — the only place to be careful

Three coordinate systems are in play:

1. **Image frame**: origin at top-left; `u` → right, `v` → down.
2. **Camera frame**: origin at the lens; `x` → right, `y` → up, `z` → forward.
3. **World frame**: whatever the robot uses (usually `z` up).

The general case (with `R_wc`) handles every sign convention automatically,
because `R_wc` encodes them. **Recommendation: always use the general case.**

To check the sign in 10 seconds:
- Pick a known reference point (e.g. `x = 0.2, y = 0`).
- It should land at `u > cx`, `v ≈ cy`.
- Back-project; you should recover ≈ `(0.2, 0, 0)`.

---

## 8. Validation — the round-trip test

The single most useful debugging step:


Pick a known 3D point: P_truth (ground truth).

Project it to pixels: (u_pred, v_pred) = project(P_truth)

Detect the same object via vision: (u_det, v_det).

If (u_pred, v_pred) ≈ (u_det, v_det), K and R_wc are correct.

Back-project (u_det, v_det) → P_vision.

Check ‖P_vision − P_truth‖ is small (a few mm).



Projection is the **inverse** of back-projection:


P_cam = R_wcᵀ · (P_world − t_wc) world → camera
p_img = K · P_cam
u = p_img(1) / p_img(3)
v = p_img(2) / p_img(3)



If the round-trip fails, the cause is almost always one of:
- `R_wc` transposed (`getObjectMatrix` in CoppeliaSim is row-major)
- `K` wrong (FOV in radians vs. degrees)
- Image flipped (`flip(·, 1)` vs. `flip(·, 2)` — must be `2`)

---

## 9. Practical notes for a real robot

- **Calibrate K** with a checkerboard (`cameraCalibrator` in MATLAB).
  Not needed in simulation; required on a real robot.
- **Measure `t_wc, R_wc`** with hand-eye calibration (Tsai) or a
  4-point homography on the table.
- **The cube has height.** The visible centroid is on the **top face**.
  With a vertical camera, the `(x, y)` of the top face equals that of the
  base, so intersecting with `z = z_table` is correct. With a tilted
  camera, intersect with `z = z_table + cube_height` instead.

---

## 10. What this gives you

This pipeline:
- ✔ Works with a **single RGB webcam** (no depth sensor).
- ✔ Is the **same** on a real robot (only calibration changes).
- ✔ Teaches intrinsics, extrinsics, and back-projection — the three
  concepts you need for **any** vision-based robotics task.
- ✔ Decouples perception from control (FSM unchanged).

*End of document.*

