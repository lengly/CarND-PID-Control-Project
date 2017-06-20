# CarND-PID-Control-Project


### Describe the effect each of the P, I, D components had in your implementation.

- P component is Cross Track Error (CTE). It tell us the correct steering angel direction. For example it tell us we need turn left or turn right. But P component will cause a lot of overshoot and oscillations.

- D component controls the speed we change our steering angel. If CTE changes too fast, than let's slow down a little. It makes our car more stable.

- I component is the duration of CTE. It just like memory.


### Describe how the final hyperparameters were chosen.

First I just manual tuning the parameters. After that I know the magnitude of them. Then I use twiddle to fine tuning them.