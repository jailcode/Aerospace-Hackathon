# Rocket Ascent Vector Control – Task Breakdown

## TASK 1 — Core Utility Functions

### 1. Magnitude of Angular Acceleration
- Implement a function to compute the magnitude of angular acceleration.

### 2. Kp Calculation
- Compute **Kp** as:

### 3. Sensor Parsing
- Parse values coming from:
- Gyroscope  
- Accelerometer  

### 4. Time Difference Utility
- Create a function to compute time difference in milliseconds.

### 5. Epoch Timing
- Compute:

### 6. Error Function
- Implement an error function that takes **three parameters**.

### 7. Sensor Signal Waiting
- Add a routine that waits for signals/data from sensors.

---

## TASK 2 — Input Handling

### 1. Gyroscope Function
- Create a function that returns an **int array of size 3** (x, y, z values).

### 2. Sensor Input Types
- Gyroscope → angle values (**double**)  
- Accelerometer → angular acceleration (**double**)

### 3. File-Based Input
- Two input files:
- `gyroscope.txt`
- `accelerometer.txt`
- Read both files **simultaneously**, line-by-line.

---

## TASK 3 — Processing Loop

1. After reading sensor inputs, start the processing phase.  
2. Run **one PD cycle**.  
3. Write output to an output file.  
4. **Output contains:**
 - Two gimbal angles  
 - One fin angle  
5. Repeat the full cycle while:
 - Updating next inputs using previous output  
 - Adding noise to simulate realistic conditions  

---

## TASK 4 — Processing Logic

### What happens inside processing?

1. **Read Expected & Measured Values**
 - Measured = original + noise  
 - Expected = from previous output  

2. **Calculate Error**

3. **Two-Time-Stamp Initialization**
- Steps 1 and 2 are executed **twice in the first loop**  
  → needed to get differential values.

4. **Magnitude Initialization & Update**
- Initialize:
  ```
  magnitude = 1
  ```
- Update each iteration using acceleration values.

5. **Acceleration Note**
- Only acceleration values contribute to magnitude updates.

6. **Run PD Controller**
- Use error + differential to compute control output.

---
