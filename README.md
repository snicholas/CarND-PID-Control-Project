# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Reflections on PID
The behaviour is what I was expecting, the car was able, after some manual trial and error tuning, to drive through the track without going outside the lane. To find the actual values I also implemented the twiddle alghorithm, that I used at varoius stages to fine tune the values. 
Anyway the followed trajectory is a bit unstable especially on turns.
The three parameters are:
- P is proportional to the value of the current error
- I integrates the past error values over times
- D is an estimate of the future trends of the error