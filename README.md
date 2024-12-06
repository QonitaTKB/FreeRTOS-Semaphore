# Exercise 8: Eliminating Resource Contention Using a Semaphore to Protect Critical Sections

This project demonstrates the use of **FreeRTOS Semaphores** for synchronization between multiple tasks in a multitasking system. The program focuses on how semaphores manage the execution order of tasks, specifically in controlling the behavior of LEDs on the **STM32F103C8T6**.

---

## **Diagram Task**
https://drive.google.com/file/d/18MD5usXj3OTBPVKLr7SZ-sG0O04YJtgU/view?usp=drive_link

---

## **Required Hardware**
- **STM32F103C8T6** Microcontroller
- LEDs (Red, Green, Blue, Orange)

---

## **Required Software**
- **STM32CubeIDE**
- **FreeRTOS**

---

## **How It Works**

### **Semaphore and Task Initialization**
- The program starts with FreeRTOS initialization.
- A **binary semaphore** is created to synchronize access to the critical section (the LEDs).
- **Four tasks** are created:
  1. **RedTask**  
  2. **GreenTask**  
  3. **BlueTask**  
  4. **OrangeTask**  

### **Task Descriptions**
#### **RedTask**
- Waits for the semaphore to access **LED 2**.
- If the semaphore is available, lights up **LED 2** for **500ms**, then releases the semaphore.

#### **GreenTask**
- Waits for the semaphore to access **LED 1**.
- If the semaphore is available, lights up **LED 1** for **500ms**, then releases the semaphore.

#### **BlueTask**
- Runs only **after the semaphore is released** by another task.
- Uses the semaphore to light up **LED 3** for **500ms**, then releases the semaphore.

#### **OrangeTask**
- A **higher-priority task** that runs continuously **independently of the semaphore**.
- Controls **LED 4**, which blinks with a **50ms delay** to demonstrate normal operation.

---

### **Semaphore Mechanism**
- The **binary semaphore** ensures that only **one task** can access the critical section (LEDs) at a time.
- Tasks that fail to acquire the semaphore enter a **blocked state** until it is released.

---

### **Task Behavior**
#### **Execution Order with Semaphore**
- If the semaphore is available, **RedTask**, **GreenTask**, or **BlueTask** can acquire it in turn.
- Tasks that fail to acquire the semaphore are blocked and wait for it to be released.

#### **OrangeTask**
- Runs independently of the semaphore and executes continuously.
- The **orange LED** blinks rapidly, unaffected by other tasks.

---

## **LED Operation Cycle**

| **Task**       | **LED Controlled** | **Behavior**                                                                 |
|-----------------|--------------------|-------------------------------------------------------------------------------|
| **RedTask**     | LED 2 (Red)        | Lights up for **500ms** if the semaphore is available.                       |
| **GreenTask**   | LED 1 (Green)      | Lights up for **500ms** if the semaphore is available.                       |
| **BlueTask**    | LED 3 (Blue)       | Lights up for **500ms**, but only after the semaphore is released.           |
| **OrangeTask**  | LED 4 (Orange)     | Blinks rapidly with a **50ms delay**, independent of the semaphore.          |

---

## **Normal Operation**
- **LEDs 1, 2, and 3** (Green, Red, Blue) light up alternately based on task execution, ensuring no conflict occurs.
- **LED 4 (Orange)** blinks continuously, indicating the uninterrupted execution of OrangeTask.


---

## **Hardware Overview**
https://drive.google.com/file/d/16Y04Cxs3r6Ddw4P349TiRfD-AzsrOiKe/view?usp=sharing

---

## **Demo Video**
https://drive.google.com/file/d/1saGzvrKWSUveVxIwwUYY13sRDBWhIMzQ/view?usp=drive_link
