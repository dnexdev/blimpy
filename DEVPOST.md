## Inspiration
We were inspired by the string of papers and research projects on cute floating companions that went viral.
https://www.cnet.com/tech/floating-robots-safe-friendly-human-interaction/
https://arxiv.org/pdf/2504.01293
https://www.youtube.com/watch?v=mDCtz_BN-vc
https://www.youtube.com/watch?v=oHikvpqlAEk

## What it does
Blimpy is a floating helium balloon robot. It detects and follows a person, it can maintain a position and altitude, and it can navigate to fixed positions in a room. Blimpy can take spoken commands and converse with the people around it. For example, "Hey Blimpy, follow me!" or "Blimpy, can you help me out with my math homework?"

## How we built it
We use a weather balloon, two ESP32 microcontrollers, motors, propellers, and some string.

There are two main parts: the balloon itself and the chassis, or gondola. The helium in the balloon provides lift. The chassis holds the breadboard, wiring, propellers, battery, weight and motors.

### The hardware
We used four reversible motors that drive propellers, with each motor controlling a different part of Blimpy's movement. Two motors were mounted at the rear for forward and backward movement. Running them at different power levels also allowed Blimpy to turn. A third motor faced sideways to correct lateral drift, since even small air currents could push Blimpy around. The fourth motor was mounted vertically at the bottom of the chassis to control altitude. Because the balloon was slightly buoyant, this motor sometimes needed to push the gondola down instead of lifting it up.

All four motors needed to run in both directions. Once Blimpy was moving, its inertia kept it going, so it would continue moving for several metres after we cut the power. To slow it down, we had to apply thrust in the opposite direction.

### The software
The chassis' processing is dedicated to moving the motors. The decisions are made on a laptop that connects with it via Bluetooth. The laptop sends velocity commands twenty times a second.

We made a physics simulator with mass, quadratic drag, thrust, gusts, drift, cam latency and packet loss in order to experiment with different setups and strategies.

The laptop runs YOLO on its camera. We set up a mat of four AprilTags to add absolute positioning for Blimpy.

On the voice side of things, we run Qwen3.5-Omni in realtime to listen to the microphone, look into the camera, make conversation with the user, and issue commands. We run a local Qwen model through Ollama that determines whether or not Omni processes the conversation.

## Challenges we ran into
We had issues in making the balloon go where we wanted it to. Since this isn't a rigid drone, we had two soft bodies linked by string and we could only control the smaller one. The inertia of this setup made it difficult to rotate or change position, as it would always attempt to bounce back to the previous state it was in.

When talking to Blimpy, it was difficult for our models to differentiate speakers and parse commands. Fine tuning audio processing took a large chunk of our time.

### Wiring
At first, we soldered all the components directly together. This worked for the first few tests, but it soon became difficult to debug. Whenever a connection failed, we had to desolder part of the circuit before we could test that component separately. We eventually moved everything onto a breadboard so that we could disconnect and test one subsystem at a time.

Our first design used one ESP32-C3 to control two H-bridge boards, with two motors connected to each board. During testing, two of the motors would not start on their own. However, they sometimes started if another motor was already running. We were using the same code and sending the same commands, but the results changed depending on which other motors were active.

We checked the pins with a multimeter, rewired the circuit several times, and asked our mentors for help. The supply voltages and control signals seemed correct, and all four motors worked when we tested them individually.

Eventually, we found that the problem was caused by the ground wiring. All four motors were returning current through the same path on the breadboard. Each motor driver reads its input signals relative to its own ground pin, but the motor current was shifting that reference enough to affect how the signals were interpreted. We rewired the circuit so that each driver board had its own return path to the battery negative. This solved the problem at the time, although a similar issue appeared again later as we added more wiring.

### Two boards
We later replaced the original H-bridge boards with motor driver boards provided by the hardware lab. These boards were more reliable, but they also required more control pins than one ESP32-C3 could provide. Because of this, we divided the system between two ESP32 boards, with one acting as the master and the other as the slave.

After making this change, the master stopped advertising over Bluetooth, so we could no longer connect to it. We first thought this was another power problem, because BLE communication can draw current in short spikes. After more testing, we found that the problem was actually in the code.

### Power
We originally powered the system with a single 3.7 V battery. After changing the design to use two ESP32 boards, this did not seem to provide enough power for the whole system. We therefore added a battery holder with four 1.5 V cells.

During normal operation, we measured a little over 1 V across each motor. The completed gondola weighed approximately 800 g, although this was only an estimate because we did not have a suitable scale.

### Flying Blimpy
Testing on the bench only showed us that the motors could turn. The system behaved very differently after we inflated the 1.1 m balloon and suspended the gondola underneath it. Blimpy often drifted or rotated in directions we did not expect. Its response also changed depending on air currents and how level the gondola was hanging. The larger envelope gave us more lift, but it also created more drag because there was more surface area moving through the air.

After mounting the motors, we did not know exactly which direction each one would spin or how much thrust it would produce. We had to test them one at a time and manually record their direction and approximate strength. Even with the same command, the motors did not produce exactly the same result. This was probably due to differences between the motors, how far each propeller was pushed onto its shaft, and the angle at which each motor was mounted.

We calibrated the motors manually and divided the overall problem into smaller tasks. We started with the vertical motor and adjusted it until Blimpy could move both up and down. We then worked on forward movement, followed by turning and finally travelling over longer distances.

In each round of testing, we changed the power sent to one or two motors, observed the result, and recorded what happened. We repeated this process until Blimpy could move roughly in the direction we wanted. The control was still not very precise, but working on one movement at a time helped us identify which motor or parameter needed to be adjusted.

## Accomplishments that we're proud of
 - Making Blimpy capable of autonomously moving and following instructions!
 - Setting up an agent orchestration layer by way of a local LLM and a cloud model
 - Creating a really cute project!

## What we learned
On the hardware side, we learned to build circuits so that each subsystem can be disconnected and tested on its own, and that ground wiring matters as much as the signal wiring. We also learned that testing on the bench tells you very little about how something will fly, so it helped to divide the problem into one movement at a time and record every result. On the software side, we learned that getting a model to reliably differentiate speakers and parse commands takes far more work than getting it to make conversation.

## What's next for Blimpy
Adding more features and functionality that an assistant should have, like reminders and answering questions about what it sees. We also want to make the control more precise, so that Blimpy can hold a position and follow a person without manual calibration of each motor. Finally, we want to improve the audio processing so that Blimpy works in a noisy room with several people talking.
