Exercise 8 – Demonstrate how to eliminate resource
contention in a selective manner - using a semaphore
to protect the critical code section


This project demonstrates the use of FreeRTOS Semaphores for synchronization between multiple tasks in a multitasking system. The program focuses on how semaphores can manage the execution order of tasks, specifically in controlling the behavior of LEDs on the STM32F103C8T6.

Diagram Task:

Required Hardware:

STM32F401CCU6
LEDs
Required Software:

STM32CubeIDE
FreeRTOS
How It Works:

Semaphore and Task Initialization:

The program begins with FreeRTOS initialization, creating multiple tasks and a semaphore.
These tasks interact with the semaphore to manage access to the critical section or synchronize task execution.
Task Descriptions:

a. RedTask:

Waits for the semaphore to access LED 2.
If the semaphore is available, LED 2 lights up for 500ms before the semaphore is released.
b. GreenTask:

Uses the semaphore to access LED 1.
If the semaphore is available, LED 1 lights up for 500ms.
c. BlueTask:

Runs only after the semaphore is released by another task.
Uses the semaphore to access LED 3, which lights up for 500ms.
d. OrangeTask:

A higher-priority task that runs continuously without relying on the semaphore.
LED 4 blinks with a 50ms delay.
Semaphore Mechanism:

A binary semaphore ensures that only one task can access the critical section (in this case, the LEDs) at a time.
Tasks unable to acquire the semaphore enter a blocked state until it is released by another task.
Task Behavior:

a. Execution Order with Semaphore:

If the semaphore is available, RedTask, GreenTask, or BlueTask can access it in turn.
Tasks that fail to acquire the semaphore enter a blocked state until it is released.
b. OrangeTask:

Always runs independently of the semaphore.
The orange LED blinks rapidly, indicating the task is running normally.
LED Operation Cycle:

LEDs controlled by GreenTask, RedTask, and BlueTask light up alternately without conflict.
The orange LED controlled by OrangeTask blinks continuously without interruption.

Hardware Overview:

Demo video:
