### Project 2: Morse Code for SOS and OK (Using CC3220 LaunchPad and Code Composer Studio in C)

#### Summarize the project and what problem it was solving.
- The project is designed to send Morse code signals for "SOS" and "OK" by blinking LEDs. The red LED simulates a DOT (which is about 500 ms) and the green LED simulates a DASH which is three times as long as a DOT. There are also spaces inbetween letters (500ms) and words (3500ms).
- The board used also allows the user to switch between "SOS" and "OK" by pressing a single button. After the button is pressed the LEDs will finish the word they are on and then switch over to the other.

#### What did you do particularly well?
- I think that I handled the states pretty well. Instead of doing each letter and hard coding it, I made a set of variables that represented specific amounts of time and that translated to the board how long to keep an LED on or off and which order to do so.
- I also commented on all lines.
- Lastly debuggin was absolutely terrible. It look me so long to get this to work properly, I started over so many times. I eventually had the code output what it was doing, for how long, and in what order as it did it. EX: Green LED turning on for 1500ms -> Green LED turning off for 500ms, etc.

#### Where could you improve?
- I could improve so much. An example would be the states and how I coded it. I think I did it really well for my skill set, but I'm sure there are better ways I could have done it, just like any other project.

#### What tools and/or resources are you adding to your support network?
- I used Code Composer Studio and a CC3220 launchpad board by TI as well as the C language.

#### What skills from these projects will be particularly transferable to other projects and/or coursework?
- This project helped me learn how to handle Timers and hardware testing.

#### How did you make these projects maintainable, readable, and adaptable?
- I tried to make the code pretty simple and use as little lines as possible.
- I also commented on every line for this project and I beleive that helped me understand it and helped anyone else that will look at it understand as well.
