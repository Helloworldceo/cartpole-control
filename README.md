# Cart-Pole Control Lab

A hands-on control systems playground: balance the classic inverted-pendulum-on-a-cart with a hand-tuned **PID** controller, then switch to an optimal **LQR** controller computed live in your browser — same nonlinear physics, two very different design philosophies.

No install, no build step, no dependencies. It's a single HTML file — open it and it runs.

![Initial view](screenshots/1-initial.png)

## Quick start

1. Download or clone this repo.
2. Open `index.html` in any modern browser (double-click it, or drag it into a browser window).
3. Click **Start**.

That's it — the pole starts tilted very slightly and the default PID controller immediately begins balancing it.

## Step by step

### 1. Balance it with PID

The **PID** tab is selected by default. Click **Start** and watch the pole settle upright.

- **Push the pole** with the `⟵ Push` / `Push ⟶` buttons to disturb it and watch the controller recover.
- Drag the **Kp / Ki / Kd** sliders while it's running — higher `Kp` reacts to tilt more aggressively, `Kd` damps oscillation, `Ki` corrects any steady-state bias.
- Notice the **Kx / Kdx** sliders — these are what pull the cart itself back toward the center of the track. Turn them to `0` and push the pole: the *angle* still balances perfectly, but the cart slowly drifts off-track. That's not a bug — it's a real, well-known limitation of a naive PID design here (angle control and position control fight each other over the same actuator), and it's exactly the problem the next controller solves properly.

![PID recovering from a push](screenshots/2-pid-recovering.png)

### 2. Switch to LQR

Click the **LQR** tab. Under the hood, the app:

1. Numerically linearizes the true nonlinear cart-pole equations around the upright position (via finite differences — it doesn't rely on a hand-derived formula).
2. Discretizes that linear model at the simulation's timestep.
3. Solves the discrete-time algebraic Riccati equation by iterating the backward Riccati recursion to convergence.
4. Produces a single gain vector **K** such that `F = -K · (state − equilibrium)` optimally balances *all four state variables at once* — cart position, cart velocity, pole angle, and pole angular velocity — trading off state error against control effort.

Push the pole exactly as hard as before. This time both the angle **and** the cart position recover together.

- The **Q** sliders weight how much the controller cares about each state variable being off-target (position, velocity, angle, angular velocity).
- The **R** slider weights how much it "costs" to use force at all — raise it and the controller gets gentler (and slower); lower it and it gets more aggressive.
- **K is recomputed live** every time you touch a Q/R slider or a physical parameter — watch the numbers in the "Live Readouts" panel change.

![LQR recovering from a push, position and angle together](screenshots/3-lqr-recovering.png)

### 3. Experiment with the physics itself

The **Physical Parameters** sliders change the actual simulated system — cart mass, pole mass, pole length. Try making the pole much longer or the pole much heavier relative to the cart, then compare how much harder PID has to work versus how LQR just re-solves itself automatically.

### 4. Read the chart

The strip chart below the simulation plots angle (blue) and cart position (orange) over a rolling 12-second window — useful for eyeballing overshoot, settling time, and oscillation as you retune gains.

## How the physics works

The simulation integrates the standard nonlinear, frictionless rigid-rod-on-cart model with RK4 at a fixed timestep (decoupled from the animation frame rate, so it behaves consistently regardless of your monitor's refresh rate). State is `[x, ẋ, θ, θ̇]`; the only input is a horizontal force `F` on the cart.

Everything — physics, linear algebra (a hand-rolled small-matrix library), the Riccati solver, rendering, and UI — lives in `index.html`, with no external libraries. Open it in a text editor if you want to see exactly how any of it works; it's organized top-to-bottom as `Physics → Linear algebra → LQR → Simulation state → Rendering → Main loop → UI wiring`.

## Related projects

This is one of three linked projects exploring control, learning, and strategy:

- **[cartpole-rl](../cartpole-rl)** — the *same* cart-pole, balanced instead by a reinforcement-learning agent that starts with no model of the physics at all and learns purely from trial and error. Worth trying right after this one — LQR solves the system instantly with linear algebra; the RL agent needs thousands of episodes to reach a comparable policy.
- **[game-theory-lab](../game-theory-lab)** — a different flavor of strategic reasoning: the Iterated Prisoner's Dilemma and evolutionary game theory.
