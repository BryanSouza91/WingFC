# **Modifying WingFC for Other Airframe Types**

Since WingFC handles sensor fusion and PID stabilization, the primary change required for different airframes is the **Mixer Logic**.

## **1\. Traditional "T-Tail" Airplane**

**Difficulty: Low** Control surfaces are independent. You stop "mixing" signals on the same servos.

* **Logic Change:**  
  * CH1 PWM \= rollOutput (Ailerons)  
  * CH2 PWM \= pitchOutput (Elevator)  
  * *Optional:* Add a third PID controller for yawOutput (Rudder).  
* **Code Update:** Initialize a third PWM pin in the FC\_Hardware struct and add a yawPID instance.

## **2\. V-Tail Aircraft**

**Difficulty: Low** Mixes Pitch and Yaw on the rear surfaces, while Roll stays independent.

* **Logic Change:**  
  * Left V-tail \= pitchOutput \+ yawOutput  
  * Right V-tail \= pitchOutput \- yawOutput  
  * Ailerons \= rollOutput

## **3\. Multirotors (Quadcopter)**

**Difficulty: Moderate to High** Motors provide both lift and attitude control.

* **Logic Change:** Each motor is a mix of Throttle \+ Pitch \+ Roll \+ Yaw.  
  * Front-Left \= Throttle \+ Pitch \+ Roll \+ Yaw  
  * Front-Right \= Throttle \+ Pitch \- Roll \- Yaw  
  * Rear-Left \= Throttle \- Pitch \+ Roll \- Yaw  
  * Rear-Right \= Throttle \- Pitch \- Roll \+ Yaw  
* **Hardware Challenge:** Requires 4 high-frequency (400Hz+) PWM outputs and a more robust disarm mechanism.

## **Technical Implementation Strategy**

To make WingFC modular, refactor the loop to use a generic **Mixer Function** or Interface:  
`func ApplyMixer(pitch, roll, yaw, throttle float64) (outputs []float64) {`  
    `// Switch based on airframe config`  
    `// Return an array of PWM pulse widths`  
`}`

### **Key Considerations**

* **PID Tuning:** Every airframe has a different "Moment of Inertia."  
* **Loop Timing:** 200Hz is great for wings; 400Hz+ is preferred for twitchy multirotors.  
* **Center of Gravity (CoG):** Hardware changes always require physical re-balancing.