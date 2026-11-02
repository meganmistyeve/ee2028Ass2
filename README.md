Our EE2028 Assignment 2 project (Sem 1, AY26/27) at NUS. We're wrote the brains for an STM32 "attack drone" that scouts the wasteland, zaps zombies, and hopefully survives.
## Story
	In the year 6202, Earth was on the brink of extinction. A mysterious alien virus had swept across the planet, transforming every living being it touched into a mindless, flesh-eating zombie. Cities that were once bustling with life now lay in ruins, the echoes of human civilisation drowned out by the agonising moans of the infected.
	
	Shielded from the chaos outside stood our beloved laboratory in E4-03-07, NUS, Singapore, a sanctuary for the remnants of humanity. The young engineers inside this lab had dedicated themselves to finding a cure. Our development of attacking drones using STM32 might be the last flicker of hope in a world consumed by darkness.
## What the drone does

- **Senses everything:** temperature, air pressure, humidity, motion, spin and compass direction. It reports back to the Command Centre (our PC) every second.
- **Raises the alarm** if anything looks off.
- **Charges a laser gun** with button presses, then fires automatically once it has enough juice.
- **Knows when it's been knocked over** and calls for help.
## MODES
#  standby
	does nothing but send sensor readings.
	double press enters battlemode
#  battlemode
	 charge laser gun and destroy zombies
	 upside down means rescue mode.
	 double press reenters standby

## The Rules

1. **Double press** to switch between Standby and Battle mode.
2. **Single press** in Battle mode to charge the laser.
3. **Double press within 10 seconds to rescue a flipped drone.** Miss it, and the drone sends one last goodbye and goes dark forever to keep the lab's location secret. No pressure.

## Team

- [@meganmistyeve](https://github.com/meganmistyeve)
- [@saulbudie1](https://github.com/saulbudie1)



