---
title: "TVC Rocket Control & Flight Controller"
author: "REPLACE_WITH_YOUR_GITHUB_USERNAME"
description: "My attempt to build and validate an actively controlled rocket: from debugging the simulation and comparing controllers to designing a flight controller that actually fits."
created_at: "2026-06-30"
---


# June 30: I decided I didn't just want another controller comparison

I started this because I was interested in how far an actively controlled rocket could be pushed before its controller stopped being able to recover. Comparing a PID, LQR, NMPC and MLP-tuned NMPC on one nice-looking flight seemed too easy to misunderstand. Almost any controller can look good if I pick one favourable disturbance and plot the cleanest response.

So I shifted the question from *which controller produces the prettiest nominal plot?* to *under what combinations of wind, sensor noise, delay and actuator limits does each controller fail?* I started exploring failure-regime maps and even the maximal closed-loop Lyapunov exponent as a possible way of measuring divergence. I liked the idea of getting a boundary instead of a single score, but I hadn't yet shown that the Lyapunov method would work for this model. At this stage it was a research direction, not a result.

![PLACEHOLDER — first research-question sketch or notes comparing the four controllers](images/2026-06-30-research-direction.png)

**Total time spent: 2 hours**

# July 1: My first Lyapunov calculation needed fixing

The divergence calculation was more subtle than I originally treated it. If I just watched two trajectories separate and repeatedly added up their separation, I could end up counting the same growth more than once. That would give me a number, but not necessarily a meaningful one.

I revised the method to propagate two nearby trajectories for a fixed interval, record the change in separation, and then rescale the perturbation back to its original size before repeating. This is the Benettin-style renormalisation idea. It's the kind of mistake that makes me slightly suspicious of any equation that looks too tidy the first time I write it. I still needed to choose the state norm, perturbation size and convergence checks before interpreting any exponent.

![PLACEHOLDER — annotated before/after derivation for trajectory renormalisation](images/2026-07-01-lyapunov-derivation.png)

**Total time spent: 3 hours**

# July 8: Simulink was getting in the way of the actual engineering

I was trying to tune the control system, but the simulation workflow itself was becoming a problem. Scope windows and heavy live visualisation could make MATLAB/Simulink freeze or become painfully awkward to use. I changed how I launched runs and reduced unnecessary live logging so I could focus on the model rather than fighting the interface.

I also realised I couldn't blindly trust a PID Autotuner result. Linearising the rocket at time zero, before thrust had developed, could give a meaningless operating point for a powered-flight controller. I decided to use autotuning for an initial guess only, then test the gains on the nonlinear rocket. That distinction became important later: a controller tuned on the wrong plant can look mathematically legitimate while being practically useless.

![PLACEHOLDER — original Simulink model and a screenshot of the simulation/tuning setup](images/2026-07-08-simulink-workflow.png)

**Total time spent: 3 hours**

# July 31: Putting together a UKF instead of assuming my measurements were perfect

The controller needed a state estimate, not just a perfect quaternion straight from the simulation. I worked on a ten-state Unscented Kalman Filter with a quaternion, position and velocity, propagating from inertial measurements and updating from GPS/barometer data.

The part I had to think about was acceleration during launch. An accelerometer doesn't simply point to gravity when the rocket is accelerating under thrust. If I treated it that way, I could push the attitude estimate in the wrong direction. I used a thrust/burn-dependent gate, including the first 2.49 seconds of powered flight, with a further near-1g check before using acceleration for attitude correction. I also had to confront an observability limitation: GPS and barometer measurements alone don't magically make yaw observable. The estimator design was taking shape, but it wasn't a licence to claim perfect attitude estimation.

![PLACEHOLDER — UKF state vector, measurement paths, or Simulink estimator subsystem](images/2026-07-31-ukf-model.png)

**Total time spent: 4 hours**

# August 5: Several tiny sign errors were ruining the TVC response

This was a much messier integration problem than I expected. Pitch behaviour looked plausible, but yaw and parts of the accelerometer correction were behaving incorrectly. I traced the issues through the quaternion-error multiplication order, accelerometer reference-frame/sign handling, a yaw sign error, and pitch/yaw test moment pulses that weren't actually connected as intended.

Once those paths were corrected, the recorded tests showed large reductions in rate error (about 94.7% for pitch and 91.4% for yaw in the relevant comparisons). The numbers were encouraging, but the more important discovery was *why* it had gone wrong: a reasonable-looking block diagram can conceal a sign or frame convention that reverses the behaviour of the whole loop. I want to preserve the faulty results alongside the corrected ones rather than pretend the first version worked.

![PLACEHOLDER — side-by-side faulty and corrected yaw/pitch response plots](images/2026-08-05-tvc-ukf-sign-fix.png)

**Total time spent: 3 hours**

# August 20: My wind disturbance wasn't actually disturbing the rocket

I added Dryden turbulence because I wanted to test the controllers in something less forgiving than still air. But initially I had wind signals without a meaningful aerodynamic force path. I could change the wind input and still not be testing what I thought I was testing. That was a pretty fundamental problem for a project about disturbance rejection.

I connected the air-relative velocity to the aerodynamic force calculation, then used the centre-of-pressure offset and rotational damping to form the corresponding moments. The distinction matters: wind velocity in m/s isn't an arbitrary torque command, and I shouldn't treat the two as interchangeable. The early aerodynamic coefficients were provisional, so I still needed to justify or measure them rather than simply declare the model physically validated.

![PLACEHOLDER — Dryden block and the added air-relative-velocity/aerodynamic-load path](images/2026-08-20-wind-aerodynamics.png)

**Total time spent: 2 hours**

# August 20: The estimator handover made my PID command jump

Another problem showed up around the 2.49-second estimator handover. The estimated attitude stepped from roughly 1.0636° to 2.1134°, and derivative action turned that change into a much bigger control command: roughly 0.215° to 5.656°. I originally wanted to tune around the behaviour, but the real issue was the discontinuity and the way the controller responded to it.

I changed the abrupt accelerometer correction into filtered, thrust-dependent weighting and moved to PI plus explicit body-rate damping rather than a raw derivative of the attitude error. The recorded validation no longer showed the same handover spike. I learned that a controller's response to estimator *transitions* matters almost as much as its response to genuine disturbances.

![PLACEHOLDER — command spike before and smoother handover after the filter change](images/2026-08-20-estimator-handover.png)

**Total time spent: 3.4 hours**

# August 20: I built the first trajectory-divergence test

I came back to the failure-boundary idea from June and wrote a Benettin-style two-trajectory workflow. I needed the nominal and perturbed runs to experience identical disturbance forcing; otherwise I'd be measuring differences in the wind as well as differences caused by the dynamics. I also had to handle quaternion sign/norm conventions in the state separation.

The script used 0.2-second renormalisation intervals and a full closed-loop state representation, and I got a short two-interval smoke test recorded. That's all I can confidently call it: a smoke test, not proof of convergence or stability. There's still work to do around state scaling, perturbation size and whether the finite-time result says something useful near a controller's failure boundary.

![PLACEHOLDER — run_benettin_lyapunov.m or its first smoke-test output](images/2026-08-20-benettin-smoke-test.png)

**Total time spent: 2 hours**

# August 21: The PID offset wasn't just a gain problem

I was still seeing small persistent pitch/yaw errors. Some of this was tied to the actuator dead zone, so endlessly adjusting gains wasn't the cleanest answer. I used a powered-flight linearisation point (around 0.1 s and 5.2 N in the recorded analysis), added inverse dead-zone compensation, and changed the integral gain from 0.1 to 0.3 while keeping anti-windup in place.

One recorded yaw offset went from about 0.145° to 0.0209°. I was pleased to see it shrink, but I also didn't want to overfit the baseline for one trajectory. If I'm going to compare four controllers, I need a documented PID configuration that I can freeze and reuse, not gains that quietly change every time another plot looks bad.

![PLACEHOLDER — PID before/after dead-zone compensation plot and gain values](images/2026-08-21-pid-deadzone.png)

**Total time spent: 3 hours**

# August 23: The physics audit invalidated a lot of my earlier confidence

This was probably one of the most important debugging days. My deterministic gust pulse was too short for its clock: a 0.1-second pulse driven by a 1-second digital clock could simply be missed. I replaced it with a one-cosine translational gust coupled through the actual aerodynamic model instead of injecting an arbitrary disturbance moment.

Then the wider audit uncovered an even worse issue: the aerodynamic path was being fed Euler angles where it should have received body velocity. One recorded comparison was about 46.9 m/s for the true velocity versus 2.93 from the wrong signal. I also corrected gravity transformation, wind-frame transformation and the rail constraints, including making sure the rocket didn't re-latch after rail exit. The rail check recorded release around 0.53 s and 1.540 m.

It was frustrating to realise earlier numerical results might have been based on incorrect physics. But I'd rather discover that now than build the whole paper around attractive, invalid graphs. Anything from before these fixes would need to be rerun before I relied on it.

![PLACEHOLDER — before/after aerodynamic wiring screenshot with the incorrect Euler-angle input highlighted](images/2026-08-23-physics-audit.png)

**Total time spent: 3.9 hours**

# August 26–27: More gimbal authority actually made things worse

I set up large recoverability sweeps across gust, embedded delay and gimbal limits: the recovered record describes 3,520 cases for a finite-horizon performance measure and 2,420 for RMS error. I had to use normal Simulink mode because the accelerated workflow had produced results I couldn't trust. I also caught an axis-label mistake: the disturbance variable represented peak *gust speed* in m/s, not an applied moment.

The counterintuitive result was what stuck with me. At around 50.1 ms delay, the recorded 5° authority case gave 6.24° RMS, but allowing 15° authority gave 8.55° RMS. More actuator range didn't automatically mean more control; with delay, it could mean more over-correction. At essentially zero delay, both cases had almost identical tiny RMS errors because the controller barely needed to command any gimbal angle. I want to rerun this with fixed seeds and save the traces, because this could become a much more interesting result than a generic claim that NMPC beats PID.

![PLACEHOLDER — delay-versus-authority RMS cross-section and time histories](images/2026-08-26-delay-authority-sweep.png)

**Total time spent: 3 hours**

# August 29: The NMPC finally had its own actuator states

I rebuilt the NMPC around six prediction states: pitch error, yaw error, the two corresponding body rates, and two gimbal states. The earlier controller setup didn't represent actuator behaviour as explicitly as I wanted. This version included a roughly 30 ms actuator dynamic model, constraints on gimbal command and rate, and a terminal cost derived from a local discrete model.

The recorded 5° recovery case settled to within 0.5° in about 0.81 s, ending with roughly 0.129° error and no reported non-positive solver statuses. I was happy to have a meaningful recovery test, but a successful test case is not the same as knowing the controller's failure envelope. I still needed matched disturbances and, eventually, actual computational timing rather than assuming an optimisation solver would fit on embedded hardware.

![PLACEHOLDER — six-state NMPC diagram and 5-degree recovery plot](images/2026-08-29-nmpc-six-state.png)

**Total time spent: 2 hours**

# August 29 onward: I stopped judging PID from only a few hand-picked runs

I set up a 10,000-run Monte Carlo campaign for PID, varying gust speed, embedded delay and gyro-noise covariance instead of repeatedly using one favourable scenario. The run was designed to work in batches and resume if MATLAB stopped halfway through. That mattered because these simulations weren't cheap, and losing a long campaign to one failure would be awful.

The recovered paper records 10,000 completed cases, with 2,552 exceeding the chosen 5° RMS loss-of-control threshold (74.48% under that criterion). But the manuscript also gives slightly different mean RMS values in two places, around 2.411° and 2.4133°. I need to go back to the underlying result files before treating either as final. The threshold itself also needs justification. The point of Monte Carlo wasn't to make a big sample-count claim; it was to find where the apparently good controller stopped being dependable.

![PLACEHOLDER — raw Monte Carlo result chart, histogram or parameter sampling plot](images/2026-08-29-pid-monte-carlo.png)

**Total time spent: 2 hours**

# September 7: I had to make the paper match the work, not the other way around

I spent time cleaning the paper's formatting, including equation overflow in the two-column layout. But the more awkward part was auditing what the draft actually claimed. Some sections and abstract statements were ahead of the evidence: there were still missing comparative results for LQR/NMPC/MLP-tuned NMPC, and hardware-in-the-loop or flight validation couldn't be claimed just because those were goals.

I also found discrepancies in the documented gust range and quaternion-error convention. It would have been tempting to keep polishing the writing and ignore them. Instead, I made a list of the gaps to resolve. I want the final paper to reflect experiments I can reproduce, even if that means a less impressive abstract for now.

![PLACEHOLDER — paper revision or annotated list of inconsistent claims](images/2026-09-07-paper-audit.png)

**Total time spent: 1.5 hours**

# September 13: I found a 20 ms mistake in the paper

I spotted a discrepancy in the embedded-delay value: I'd written 53 ms where it should have been 33 ms. I checked that 33 ms still fell within the delay sweep range, which meant the chosen sweep still covered that operating point. The error was in what I'd documented, but a paper about delay-sensitive control really can't afford careless delay values.

It was a reminder to trace every headline parameter back to the model or script rather than trusting numbers that have been copied between draft sections. I need to make the correction consistently wherever the old value appears.

![PLACEHOLDER — corrected delay value in manuscript beside sweep range](images/2026-09-13-delay-correction.png)

**Total time spent: 1 hours**

# September 14: I added the PCB views and component table to the report

I put an orthographic PCB view and a component table into the report. The board isn't just a pretty render: it should let someone see how the electrical design relates to the physical layout and what components I intended to use.

I still need to label which specific PCB revision this image belongs to. I later decided to start a new simplified flight controller in October, so I don't want to blur the early board documentation and the later redesign into a fictional single, seamless build.

![PLACEHOLDER — original PCB orthographic view and component table from the report](images/2026-09-14-original-pcb-view.png)

**Total time spent: 2 hours**

# September 15: An NMPC result looked too good, which turned out to be a warning

I was checking the aerodynamic model because something about the steady-state behaviour didn't add up. Even baseline NMPC was removing a persistent error in a situation where I expected some steady-state offset. Rather than immediately celebrating the controller, I went back to the forces in the plant.

The problem was that I had represented transient aerodynamic effects without the relevant steady-state force contribution. In other words, the controller was being tested against a model that was missing part of the physics. I worked through that discrepancy and verified the aerodynamic modelling rather than claiming a controller achievement based on an incomplete plant. This was another example of why I can't assess the control algorithm separately from the model it is controlling.

![PLACEHOLDER — steady-state force/model correction and before/after response](images/2026-09-15-steady-state-aerodynamics.png)

**Total time spent: 3 hours**

# September 16: I tried to make the maths and Simulink tell the same story

I added the UKF formulation to the paper and worked the aerodynamic model into the mathematical description, checking the correspondence against Simulink. It's surprisingly easy to get an implementation running and then struggle to explain exactly what equations are being executed.

This mattered to me because a nice Simulink block diagram alone isn't a reproducible methodology. I wanted the estimator assumptions and the forces in the simulation to be understandable to someone reading the paper without my model open in front of them. This was a documentation and model-consistency step, not a claim that the hardware had been tested.

![PLACEHOLDER — paper UKF/aerodynamic equations next to corresponding Simulink blocks](images/2026-09-16-ukf-aero-maths.png)

**Total time spent: 2.5 hours**

# September 17: NMPC Monte Carlo ran into MATLAB function-call problems

I moved into Monte Carlo simulations for NMPC, but the MATLAB code wasn't compiling/executing properly at first. The issue came down to function calls. This was one of those situations where the control theory could be fine and the experiment would still go nowhere because the surrounding code couldn't run reliably.

I went through the calls, fixed the problems and got the campaign running. I haven't recorded final Monte Carlo statistics for that day, so I'm not going to invent any. What I did get was a working route toward testing NMPC across a much larger set of cases than a single recovery simulation.

![PLACEHOLDER — MATLAB error/function-call fix and NMPC run output](images/2026-09-17-nmpc-monte-carlo-debugging.png)

**Total time spent: 2 hours**

# September 28: I revisited the DARE derivation

I corrected the Discrete Algebraic Riccati Equation derivation used in the paper for the terminal cost. The equations had to agree with the actual discrete linearised system, not just look like a familiar continuous-time LQR formula rearranged onto the page.

This was mainly a mathematical correctness pass. I don't want the terminal penalty presented as a stability guarantee that the derivation doesn't establish. I still need to check the final expression against the implementation and retain the earlier version so the correction is visible.

![PLACEHOLDER — annotated corrected DARE / terminal-cost equations](images/2026-09-28-dare-correction.png)

**Total time spent: 4 hours**

# September 29 : I finished an earlier PCB design

I reached a completed design state for an earlier flight-controller PCB. At the time, finishing the board was a milestone, but that wasn't the end of the hardware decision-making. A finished design on-screen isn't necessarily a board that fits the final rocket or has every interface wired correctly.

I would end up reconsidering which controller board to build and then drawing a simpler design in October. I want this older revision in the journal because it shows actual work done, even though I didn't ultimately treat it as the final board. **I still need to confirm the exact date and identify the board files for this entry.**

![PLACEHOLDER — completed earlier PCB render or KiCad layout](images/2026-09-29-earlier-pcb-complete.png)

**Total time spent: 1 hours**

# September 30 : I expanded the UKF equations in the paper

I spent more time documenting the actual UKF mathematics. I'd already added the estimator formulation earlier in September, but I still needed a clearer explanation of how the sigma-point prediction and measurement update related to the Simulink implementation.

Writing the equations forced me to inspect assumptions that are easy to hide inside software blocks: how the quaternion is handled, how uncertainty is propagated, and when different sensors should be trusted. This entry is an extension of the September 16 work, not a claim that I invented an entirely new estimator that day.

![PLACEHOLDER — expanded UKF derivation or edited manuscript page](images/2026-09-30-ukf-equations.png)

**Total time spent: 2 hours**

# October 1 : I started reading more seriously about LQR

I began the LQR literature review because I couldn't just put LQR into the controller comparison as a token classical-control baseline. I needed to understand how previous work defined the linearised plant, chose its states and weights, and described the limits of a fixed-gain solution when the real rocket is nonlinear and changing over time.

The literature review was still in progress. I hadn't established a finished matched LQR experiment from this activity. It was about strengthening the reasoning behind the baseline before drawing conclusions from comparisons.

![PLACEHOLDER — literature notes, reference list or annotated LQR derivation](images/2026-10-01-lqr-literature.png)

**Total time spent: 2.3 hours**

# October 2 : Turning the wind blocks back into equations

I started translating the Simulink wind/turbulence model into the mathematics for the paper. The simulation already had a physical wind path, but that isn't helpful to a reader if the report just says "Dryden turbulence" and leaves out what the model is actually doing.

I wanted to distinguish the stochastic turbulence process from the deterministic gust and then show how air-relative velocity leads to the aerodynamic forces and moments that the controllers must reject. At this point I was still translating and documenting the model, not claiming the derivation was fully finalised.

![PLACEHOLDER — wind/turbulence Simulink blocks paired with draft equations](images/2026-10-02-wind-model-equations.png)

**Total time spent: 2 hours**

# October 3: I compiled code to MEX

I started compiling the simulation/controller code to MEX to get faster runs. Parameter sweeps and Monte Carlo campaigns sound manageable when they're just numbers in a plan; once I actually start running thousands of cases, every bit of overhead hurts.

Getting the code onto a faster execution path was about making the experimental workload practical. I still need to keep the MEX output consistent with the original MATLAB implementation and record a proper before/after benchmark rather than assuming that faster compilation means identical results.

![PLACEHOLDER — MEX build success output or timed MATLAB-versus-MEX test](images/2026-10-03-mex-compilation.png)

**Total time spent: 3 hours**

# October 4: I began building the failure maps 

I started setting up the failure-map analysis, which was really the reason for a lot of the earlier controller work. Instead of producing endless isolated plots, I wanted a way to show how gust, delay, sensor noise and actuator limits interact and where the probability of successful control starts to collapse.

I was only setting up the pipeline at this point. The hard part would be keeping the four-controller comparison fair: same plant, disturbance seeds, actuator assumptions, metrics and failure criterion. I don't want an impressive-looking map that mostly measures differences in how I configured the simulations.

![PLACEHOLDER — early failure-map script, grid definition or draft heatmap](images/2026-10-04-failure-map-setup.png)

**Total time spent: 1 hours**

# October 8: Four separate sweep charts were getting impossible to compare, so I made condensed ones

I updated the plotting/analysis code so each sweep could compare PID, LQR, NMPC and MLP-tuned NMPC together rather than presenting one controller's sweep at a time. Individual charts were taking up space and making it harder to see the differences that the paper was meant to investigate.

The goal was a condensed set of figures where a reader could compare all four under a common sweep. This was a change to the analysis and visualisation code; I hadn't yet confirmed the final charts or every controller result. Getting the presentation right only helps if the underlying runs are matched and valid.

![PLACEHOLDER — old separate charts beside the new four-controller chart draft](images/2026-10-08-four-controller-plots.png)

**Total time spent: 4 hours**

# October 9: I scrapped the complicated flight-controller plan and drew a simpler schematic in one day

The flight-controller decisions had become a bit of a mess. I had several earlier boards to choose from, but each had a practical problem. One advanced board had incorrect TX/RX crossings and PWM wiring issues; another less advanced board had a faulty buck converter that needed an external bypass and still wasn't properly tested. The ultra-advanced board looked tempting, but it was designed for a different rocket and wouldn't fit this airframe. An impressive schematic is useless if I can't install it in the actual vehicle.

So I changed direction and started a **new, simpler flight controller** rather than trying to rescue the most complicated design. I deliberately prioritised working electrical connections, board fit and speed over extra features or appearance. I reused circuit blocks from my older flight-controller schematics instead of redrawing everything from zero, and I finished the new schematic in a day.

That speed came from reusing previous work, not from inventing an entire electronics design overnight. Reused blocks still need to be rechecked in their new context—especially power, TX/RX, PWM and connectors—because copying a circuit doesn't automatically copy its correctness.

![PLACEHOLDER — new simplified schematic overview, with reused functional blocks highlighted](images/2026-10-09-new-simple-fc-schematic.png)

**Total time spent: 2 hours**

# October 10: The new flight controller finally started looking like a real board

With the new schematic done, I moved into PCB layout. I finished the component placement and began routing the traces. I was trying to get an actually manufacturable board into the rocket's available space rather than repeating the mistake of having a sophisticated design that was for the wrong airframe.

I had to treat placement and routing as functional engineering, not just making the PCB look tidy. Components, connectors and signal paths need to make sense physically. The layout was complete enough to start routing, but it wasn't a finished or validated board yet.

![PLACEHOLDER — KiCad component-placement screenshot before or during first routing](images/2026-10-10-fc-placement-routing-start.png)

**Total time spent: 4 hours**

# October 12: I finished routing

I finished routing all the traces on the new flight-controller PCB. Three calendar days after finishing the schematic, I had gone through placement and routing. Considering how long I'd spent weighing up flawed older designs, this was a big shift toward actually getting a board built.

But I don't want to confuse **routed** with **working**. I still need to run the final ERC/DRC checks, audit power and signal nets, verify the PWM and TX/RX connections, inspect the mechanical fit, and make sure the design is genuinely ready to manufacture. The earlier boards taught me that electrical mistakes can survive surprisingly far into a design. For now, I have a completed route—not a tested flight computer, not a fabricated PCB, and definitely not a successful flight. That's the next stage.

![PLACEHOLDER — fully routed KiCad PCB, ideally front/back copper views](images/2026-10-12-fc-routing-complete.png)

**Total time spent: 3 hours**
