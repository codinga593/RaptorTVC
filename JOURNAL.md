<img width="3889" height="2294" alt="recoverability_metric_cross_sections" src="https://github.com/user-attachments/assets/f41910f1-22cb-452c-93f0-ce0e166b3cd1" />---
title: "TVC Rocket Control & Flight Controller"
author: "REPLACE_WITH_YOUR_GITHUB_USERNAME"
description: "My attempt to build and validate an actively controlled rocket: from debugging the simulation and comparing controllers to designing a flight controller that actually fits."
created_at: "2026-06-30"
---


# June 30: I wanted to find where each controller fails

I started this because I was interested in how far an actively controlled rocket could be pushed before its controller stopped being able to recover. Comparing a PID, LQR, NMPC and MLP-tuned NMPC on one nice-looking flight seemed too easy to misunderstand. Almost any controller can look good if I pick one favourable disturbance and plot the cleanest response.

I wanted to know *under what combinations of wind, sensor noise, delay and actuator limits does each controller fail?* I started exploring failure-regime maps and even the maximal closed-loop Lyapunov exponent as a possible way of measuring divergence. I liked the idea of mapping a failure boundary. I still had to find out whether the Lyapunov method would work for this model.

**Total time spent: 2 hours**

# July 1: My first Lyapunov calculation needed fixing

The divergence calculation was more subtle than I originally treated it. If I just watched two trajectories separate and repeatedly added up their separation, I could end up counting the same growth more than once. That would make the result unreliable.

I revised the method to propagate two nearby trajectories for a fixed interval, record the change in separation, and then rescale the perturbation back to its original size before repeating. This is the Benettin-style renormalisation idea. It's the kind of mistake that makes me slightly suspicious of any equation that looks too tidy the first time I write it. I still needed to choose the state norm, perturbation size and convergence checks before interpreting any exponent.

<img width="2560" height="1600" alt="image" src="https://github.com/user-attachments/assets/c936b539-afc3-44ed-ba47-1c953b33cced" />


**Total time spent: 3 hours**


# July 31: Putting together a UKF for state estimation

I needed to estimate the controller's state inputs from sensor measurements. The simulation could supply a perfect quaternion, which bypassed that problem. I worked on a ten-state Unscented Kalman Filter with a quaternion, position and velocity, propagating from inertial measurements and updating from GPS/barometer data.

The part I had to think about was acceleration during launch. An accelerometer doesn't simply point to gravity when the rocket is accelerating under thrust. If I treated it that way, I could push the attitude estimate in the wrong direction. I used a thrust/burn-dependent gate, including the first 2.49 seconds of powered flight, with a further near-1g check before using acceleration for attitude correction. I also had to account for an observability limitation: GPS and barometer measurements alone leave yaw unobservable. The estimator still had limits I needed to work through.

<img width="1954" height="822" alt="image" src="https://github.com/user-attachments/assets/6f1e7dda-fe52-4544-8660-119e73e8e368" />


**Total time spent: 4 hours**

# August 5: Several tiny sign errors were ruining the TVC response

This was a much messier integration problem than I expected. Pitch behaviour looked plausible, but yaw and parts of the accelerometer correction were behaving incorrectly. I traced the issues through the quaternion-error multiplication order, accelerometer reference-frame/sign handling, a yaw sign error, and pitch/yaw test moment pulses that weren't actually connected as intended.

Once those paths were corrected, the recorded tests showed large reductions in rate error (about 94.7% for pitch and 91.4% for yaw in the relevant comparisons). Seeing how much a sign or frame convention could change the loop's behaviour made those mistakes easier to understand. I want to keep the faulty results alongside the corrected ones so I can trace what changed.

<img width="1800" height="560" alt="image" src="https://github.com/user-attachments/assets/9d748529-3ff7-420b-a2af-d935e8c0459e" />

**Total time spent: 3 hours**

# August 20: My wind disturbance wasn't actually disturbing the rocket

I added Dryden turbulence because I wanted to test the controllers in something less forgiving than still air. But initially I had wind signals without a meaningful aerodynamic force path. I could change the wind input and still not be testing what I thought I was testing. That was a pretty fundamental problem for a project about disturbance rejection.

I connected the air-relative velocity to the aerodynamic force calculation, then used the centre-of-pressure offset and rotational damping to form the corresponding moments. This gave the wind input in m/s a physical path through to the loads on the rocket. The early aerodynamic coefficients were provisional, so I still needed to justify or measure them before treating the model as physically validated.

<img width="1454" height="626" alt="image" src="https://github.com/user-attachments/assets/abb16ceb-2b2f-4417-9074-be8ea9285c66" />
<img width="1574" height="894" alt="image" src="https://github.com/user-attachments/assets/e5226fe7-39d1-4058-81ee-597acf3957a4" />


**Total time spent: 2 hours**

# August 20: The estimator handover made my PID command jump

Another problem showed up around the 2.49-second estimator handover. The estimated attitude stepped from roughly 1.0636° to 2.1134°, and derivative action turned that change into a much bigger control command: roughly 0.215° to 5.656°. I originally wanted to tune around the behaviour. Looking at the handover showed me that I needed to deal with the discontinuity and the controller's response to it.

I changed the abrupt accelerometer correction into filtered, thrust-dependent weighting. I also replaced the raw derivative of the attitude error with PI plus explicit body-rate damping. The recorded validation no longer showed the same handover spike. That made me pay more attention to what happens when the estimator changes how it uses its measurements.

**Total time spent: 3.4 hours**

# August 20: I built the first trajectory-divergence test

I came back to the failure-boundary idea from June and wrote a Benettin-style two-trajectory workflow. I needed the nominal and perturbed runs to experience identical disturbance forcing; otherwise I'd be measuring differences in the wind as well as differences caused by the dynamics. I also had to handle quaternion sign/norm conventions in the state separation.

The script used 0.2-second renormalisation intervals and a full closed-loop state representation, and I got a short two-interval smoke test recorded. I still needed to check convergence and stability, work through state scaling and perturbation size, and find out whether the finite-time result said something useful near a controller's failure boundary.

<img width="2070" height="1336" alt="image" src="https://github.com/user-attachments/assets/a2242afb-3595-4958-9451-c64d198eb495" />


**Total time spent: 2 hours**

# August 21: I worked on the actuator dead zone and PID offset

I was still seeing small persistent pitch/yaw errors. Some of this was tied to the actuator dead zone, which I needed to account for while tuning. I used a powered-flight linearisation point (around 0.1 s and 5.2 N in the recorded analysis), added inverse dead-zone compensation, and changed the integral gain from 0.1 to 0.3 while keeping anti-windup in place.

One recorded yaw offset went from about 0.145° to 0.0209°. I was pleased to see it shrink, but I also didn't want to overfit the baseline for one trajectory. For the four-controller comparison, I need to document and freeze the PID configuration so I can reuse the same gains across the runs.

<img width="1178" height="738" alt="image" src="https://github.com/user-attachments/assets/8722fb3e-9270-40e1-a274-8078222de02f" />


**Total time spent: 3 hours**

# August 23: The physics audit invalidated a lot of my earlier confidence

I found several problems in the physics model today. My deterministic gust pulse was too short for its clock: a 0.1-second pulse driven by a 1-second digital clock could simply be missed. I replaced it with a one-cosine translational gust coupled through the aerodynamic model. This also replaced the arbitrary disturbance-moment injection.

Then the wider audit uncovered an even worse issue: the aerodynamic path was being fed Euler angles where it should have received body velocity. One recorded comparison was about 46.9 m/s for the true velocity versus 2.93 from the wrong signal. I also corrected gravity transformation, wind-frame transformation and the rail constraints, including making sure the rocket didn't re-latch after rail exit. The rail check recorded release around 0.53 s and 1.540 m.

It was frustrating to realise earlier numerical results might have been based on incorrect physics. Anything from before these fixes would need to be rerun before I could rely on it in the paper.


**Total time spent: 3.9 hours**

# August 26–27: More gimbal authority actually made things worse

I set up large recoverability sweeps across gust, embedded delay and gimbal limits: the recovered record describes 3,520 cases for a finite-horizon performance measure and 2,420 for RMS error. I had to use normal Simulink mode because the accelerated workflow had produced results I couldn't trust. I also caught an axis-label mistake. The disturbance variable represented peak *gust speed* in m/s, and the label incorrectly described it as an applied moment.

The counterintuitive result was what stuck with me. At around 50.1 ms delay, the recorded 5° authority case gave 6.24° RMS, but allowing 15° authority gave 8.55° RMS. With delay, the extra actuator range could allow more over-correction. At essentially zero delay, both cases had almost identical tiny RMS errors because the controller barely needed to command any gimbal angle. I want to rerun this with fixed seeds and save the traces to check how repeatable that delay-and-authority effect is.

<img width="3889" height="2294" alt="recoverability_metric_cross_sections" src="https://github.com/user-attachments/assets/f00cfc96-d765-4a5d-9d1c-889e585442c4" />


**Total time spent: 3 hours**

# August 29: The NMPC finally had its own actuator states

I rebuilt the NMPC around six prediction states: pitch error, yaw error, the two corresponding body rates, and two gimbal states. The earlier controller setup didn't represent actuator behaviour as explicitly as I wanted. This version included a roughly 30 ms actuator dynamic model, constraints on gimbal command and rate, and a terminal cost derived from a local discrete model.

The recorded 5° recovery case settled to within 0.5° in about 0.81 s, ending with roughly 0.129° error and no reported non-positive solver statuses. I was happy with that recovery test. I still needed to explore the controller's failure envelope using matched disturbances and measure the computation time before deciding whether the optimisation solver could run on embedded hardware.



**Total time spent: 2 hours**

# August 29 onward: Ran a larger monte carlo campaign

I set up a 10,000-run Monte Carlo campaign for PID, varying gust speed, embedded delay and gyro-noise covariance across the cases. The run was designed to work in batches and resume if MATLAB stopped halfway through. That mattered because these simulations weren't cheap, and losing a long campaign to one failure would be awful.

The recovered paper records 10,000 completed cases, with 2,552 exceeding the chosen 5° RMS loss-of-control threshold (74.48% under that criterion). The manuscript also gives slightly different mean RMS values in two places, around 2.411° and 2.4133°. I need to go back to the underlying result files before treating either as final. The threshold itself also needs justification. I used Monte Carlo to find the conditions where the PID controller stopped being dependable.

<img width="2560" height="1600" alt="image" src="https://github.com/user-attachments/assets/b0fd7bde-d707-4ddc-8f8e-8383a94f62e7" />

**Total time spent: 2 hours**

# September 7: I checked the paper's claims against the work

I spent time cleaning the paper's formatting, including equation overflow in the two-column layout. But the more awkward part was auditing what the draft actually claimed. Some sections and abstract statements were ahead of the evidence: there were still missing comparative results for LQR/NMPC/MLP-tuned NMPC.

I also found discrepancies in the documented gust range and some mismatches quaternion-error convention, so I made a list of the gaps to resolve. I need to be able to reproduce the experiments behind the paper's claims.

**Total time spent: 1.5 hours**

# September 13: I found a 20 ms mistake in the paper

I spotted a discrepancy in the embedded-delay value: I'd written 53 ms where it should have been 33 ms. I checked that 33 ms still fell within the delay sweep range, which meant the chosen sweep still covered that operating point. The error was in what I'd documented, but a paper about delay-sensitive control really can't afford careless delay values.

I need to trace the parameters back to the model or script, especially numbers copied between draft sections. I also need to make the correction consistently wherever the old value appears.

<img width="758" height="416" alt="image" src="https://github.com/user-attachments/assets/ca952b79-1844-4857-a601-d508a6c22962" />

**Total time spent: 0.3 hour**

# September 14: I added the PCB views and component table to the report

I put an orthographic PCB view and a component table into the report so a reader could see how the electrical design relates to the physical layout and which components I intended to use.

I still need to label which specific PCB revision this image belongs to. I later started a new simplified flight controller in October, so I need to keep the two revisions clear in the documentation.

<img width="580" height="464" alt="image" src="https://github.com/user-attachments/assets/5b4735b4-8eaa-42c3-a076-f32229f5d335" />
<img width="746" height="536" alt="image" src="https://github.com/user-attachments/assets/8f051f64-b03e-4c63-bcf9-a2262075f2bf" />


**Total time spent: 2 hours**


# September 16: I tried to make the maths and Simulink line up
I added the UKF formulation to the paper and worked the aerodynamic model into the mathematical description, checking the correspondence against Simulink. It's surprisingly easy to get an implementation running and then struggle to explain exactly what equations are being executed.

I wanted someone reading the paper to be able to follow the estimator assumptions and the forces in the simulation without having my model open. I was working on documentation and model consistency here; hardware testing was still to come.

<img width="768" height="934" alt="image" src="https://github.com/user-attachments/assets/0d9c2618-b1f2-463c-bda6-60bc4835191c" />

**Total time spent: 2.5 hours**

# September 17: NMPC Monte Carlo ran into MATLAB function-call problems

I moved into Monte Carlo simulations for NMPC, but the MATLAB code wasn't compiling/executing properly at first. The issue came down to function calls, which I needed to fix before the campaign could run reliably.

I went through the calls, fixed the problems and got the campaign running. The monte carlo took nearly 24 hours to finish even on a decently powerful PC with promising results at 93.66% of runs succeeded, which means flight validation will likely be successful.

<img width="1894" height="406" alt="image" src="https://github.com/user-attachments/assets/0e8f2732-2dd4-46ec-9537-efa05d5d1903" />

**Total time spent: 2 hours**

# September 28: I revisited the DARE derivation

I corrected the Discrete Algebraic Riccati Equation derivation used in the paper for the terminal cost. I needed to check the derivation against the actual discrete linearised system and make sure the equations were consistent with it.

This was mainly a mathematical correctness pass. I don't want the terminal penalty presented as a stability guarantee that the derivation doesn't establish. I still need to check the final expression against the implementation and retain the earlier version so the correction is visible.

<img width="790" height="1056" alt="image" src="https://github.com/user-attachments/assets/a5c4d10b-776c-4403-bb82-cfe2a7797258" />

**Total time spent: 4 hours**

# September 29: I finished an earlier PCB design

I finished the design for an earlier flight-controller PCB. I was glad to have it done, though I still had to consider its fit in the final rocket and check the interface wiring before deciding which board to build.

I would end up reconsidering which controller board to build and then drawing a simpler design in October. I want to put this older revision in the journal. even though I didn't ultimately treat it as the final board.

<img width="1364" height="1020" alt="image" src="https://github.com/user-attachments/assets/0589f14f-e920-4520-a1df-418015b7c626" />


**Total time spent: 1 hour**

# September 30: I expanded the UKF equations in the paper

I spent more time documenting the actual UKF mathematics. I'd already added the estimator formulation earlier in September, but I still needed a clearer explanation of how the sigma-point prediction and measurement update related to the Simulink implementation.

Writing the equations made me look more closely at how the quaternion is handled, how uncertainty is propagated, and when different sensors should be trusted. I was continuing the estimator documentation I'd started on September 16.


**Total time spent: 2 hours**

# October 1: I started reading more seriously about LQR

I began the LQR literature review to understand how to set up a useful classical-control baseline. I needed to understand how previous work defined the linearised plant, chose its states and weights, and described the limits of a fixed-gain solution when the real rocket is nonlinear and changing over time.

The literature review was still in progress. I hadn't established a finished matched LQR experiment from this activity. It was about strengthening the reasoning behind the baseline before drawing conclusions from comparisons.

<img width="2560" height="1600" alt="image" src="https://github.com/user-attachments/assets/12444ca6-6136-4a51-8b53-046e87d3ae1c" />

**Total time spent: 2.3 hours**

# October 2: Turning the wind blocks back into equations

I started translating the Simulink wind/turbulence model into the mathematics for the paper. The simulation already had a physical wind path, but that isn't helpful to a reader if the report just says "Dryden turbulence" and leaves out what the model is actually doing.

I wanted to distinguish the stochastic turbulence process from the deterministic gust and then show how air-relative velocity leads to the aerodynamic forces and moments that the controllers must reject. The translation and derivation were still in progress.

<img width="794" height="1284" alt="image" src="https://github.com/user-attachments/assets/e36372a7-372f-4033-b844-3445d22ea3d4" />


**Total time spent: 2 hours**


# October 4: I began building the failure maps

I started setting up the failure-map analysis I'd been working towards with the earlier controller work. I wanted to show how gust, delay, sensor noise and actuator limits interact and where the probability of successful control starts to collapse. That was getting hard to follow across separate plots.

I was only setting up the pipeline at this point. I still needed to keep the four-controller comparison fair: same plant, disturbance seeds, actuator assumptions, metrics and failure criterion. Otherwise, differences in the simulation setup could skew the maps.


**Total time spent: 1 hour**

# October 8: Four separate sweep charts were getting impossible to compare, so I made condensed ones

I updated the plotting/analysis code to show PID, LQR, NMPC and MLP-tuned NMPC together in each sweep chart. The separate charts were taking up space and making it harder to see the differences that the paper was meant to investigate.

The goal was a condensed set of figures where a reader could compare all four under a common sweep. I'd changed the analysis and visualisation code, but I still needed to check the final charts, the controller results and whether the underlying runs were matched.

<img width="1212" height="1436" alt="image" src="https://github.com/user-attachments/assets/b7b907ea-7a20-4dc2-9647-4ed7e035cfb3" />
<img width="1226" height="1444" alt="image" src="https://github.com/user-attachments/assets/32d502eb-1ea8-4ffe-b4ca-990624c4b0e6" />


**Total time spent: 4 hours**

# October 9: I scrapped the complicated flight-controller plan and drew a simpler schematic in one day

The flight-controller decisions had become a bit of a mess. I had several earlier boards to choose from, but each had a practical problem. One advanced board had incorrect TX/RX crossings and PWM wiring issues; another less advanced board had a faulty buck converter that needed an external bypass and still wasn't properly tested. The ultra-advanced board looked tempting, but it was designed for a different rocket and wouldn't fit this airframe.

So I changed direction and started a **new, simpler flight controller**. I prioritised working electrical connections, board fit and getting the design finished. I reused circuit blocks from my older flight-controller schematics, which helped me finish the new schematic in a day.

I still need to recheck those blocks in the new design, especially power, TX/RX, PWM and connectors. Each block's power and signal connections need to suit this board.

<img width="1784" height="996" alt="image" src="https://github.com/user-attachments/assets/04d25ae4-b40e-4be8-8a6a-f8c2e73af596" />
<img width="1548" height="1326" alt="image" src="https://github.com/user-attachments/assets/4e6bcc70-11d1-4caf-9785-c68c0f8db3ad" />



**Total time spent: 3 hours**

# October 10: The new flight controller finally started looking like a real board

With the new schematic done, I moved into PCB layout. I finished the component placement and began routing the traces. I needed to fit a manufacturable board into the space available in this rocket, so I kept the airframe constraints in mind during layout.

I had to think through where the components and connectors sat and how the signal paths would run between them. Placement was done and routing was underway, with the rest of the routing and validation still ahead.

<img width="878" height="1240" alt="image" src="https://github.com/user-attachments/assets/c58d28f5-38a1-408a-93d4-0b719a3c3da9" />


**Total time spent: 4 hours**

# October 12: I finished routing

I finished routing all the traces on the new flight-controller PCB. Three calendar days after finishing the schematic, I'd completed placement and routing. After spending so long weighing up the older designs, I was glad to have the new board routed.

I still need to run the final ERC/DRC checks, audit power and signal nets, verify the PWM and TX/RX connections, and inspect the mechanical fit before sending it for manufacture. The earlier boards taught me how easily electrical mistakes can survive into the layout. Fabrication, hardware testing and flight testing are still ahead.
<img width="904" height="1256" alt="image" src="https://github.com/user-attachments/assets/b6003fae-e556-48b1-921f-ae14a8029a52" />


**Total time spent: 3 hours**

# October 16: I finished the firmware

I finished the firmware for the flight computer, largely using code blocks from a previous project. I used STM32CubeIDE to build the project. The next step will be to submit the project and order the PCB.

<img width="2560" height="1600" alt="image" src="https://github.com/user-attachments/assets/c9688151-5293-4c70-9d76-f5e6772567e2" />


**Total time spent: 4 hours**
