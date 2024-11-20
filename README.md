# Educo - FTC Robot (Team 25222)

Educo is an FTC (FIRST Tech Challenge) robot designed and built by Team 25222 for the India Regionals. The robot features an innovative mechanism for game element manipulation and leverages advanced vision processing for autonomous operations. This repository contains the codebase and documentation for the robot, programmed entirely in Java.

---

## Robot Overview

- **Team Name:** Educo  
- **FTC Team Number:** 25222  
- **Competition:** FIRST Tech Challenge (India Regionals)  
- **Theme:** [ CENTERSTAGE ]

### Key Features
- **Game Element Handling:**
  - Mechanism to pick up Pixels from the human player area.
  - Ability to place Pixels onto the BackDrop.
- **Scoring Performance:**
  - **TeleOp:** 20-30 points per match.
  - **Autonomous:** ~50 points per match.
  - Max points for hanging and drone shooting: 50 points.

- **Mobility:**
  - **Drive Type:** Differential drive.
  - **Challenges:** Less maneuverable compared to mecanum drive but optimized for task execution.

- **Hardware:**
  - 4 REV DC motors.
  - 4 servos for precise manipulations.
  - HuskyLens paired with an OpenCV model for visual recognition.

- **Vision Processing:**
  - Detects the prop location (theme element).
  - Dynamically selects and executes the appropriate autonomous code based on detection.

---

## Programming Details

- **Language:** Java  
- **Vision System:**  
  - HuskyLens for prop detection.  
  - OpenCV model for location identification and decision-making.  
- **Autonomous Mode:**  
  - Fully dynamic based on visual inputs.
  - Adjusts strategy to optimize scoring efficiency.

---

## Challenges and Highlights

1. **Differential Drive Challenges:**  
   The robot featured a differential drive, making precise maneuvering more difficult compared to mecanum drives. Overcoming this limitation was a significant achievement.

2. **Integration of Vision Processing:**  
   Using the HuskyLens and OpenCV, the robot achieved dynamic autonomous behaviors, adapting to the game field in real-time.

3. **Mentorship Experience:**  
   Guiding the team through the design, build, and programming phases was a rewarding experience, culminating in an 11th-place finish among numerous teams.

---

## How to Use the Code

1. Clone the repository:  
   ```bash
   git clone https://github.com/your-username/educo-ftc-robot.git
   cd educo-ftc-robot

2. Build the code and deploy it to the REV Control Hub using the FTC SDK.

3. For vision processing, ensure the HuskyLens is properly configured and connected.

4. Run the robot in Autonomous or TeleOp mode via the FTC Driver Station app.

## Gallery

<div style="display: flex; justify-content: space-between; align-items: center">
  <img src="https://github.com/Ayush-kaithwas/FTC-EDUCO/blob/main/Ftc%20Educo/pic1.jpg" alt="Image 1" width="370" />
  <img src="https://github.com/Ayush-kaithwas/FTC-EDUCO/blob/main/Ftc%20Educo/pic2.jpg" alt="Image 2" width="300" />
</div>

### Left Side Auto
![Robot in Action](https://github.com/Ayush-kaithwas/FTC-EDUCO/blob/main/Ftc%20Educo/FTCLeftAuto-ezgif.com-video-to-gif-converter.gif)


### Right Side Auto
![Robot in Action](https://github.com/Ayush-kaithwas/FTC-EDUCO/blob/main/Ftc%20Educo/FTCRightAuto-ezgif.com-video-to-gif-converter.gif)


### Center Side Auto
![Robot in Action](https://github.com/Ayush-kaithwas/FTC-EDUCO/blob/main/Ftc%20Educo/FTCCenterAuto-ezgif.com-video-to-gif-converter.gif)

## Acknowledgments
- Team 25222: For their dedication and hard work.
- FTC Community: For fostering innovation and collaboration.
- Mentors and Sponsors: For their unwavering support and guidance.
