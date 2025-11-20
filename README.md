# Useless Box
Jorge Larach _Finished 04/25/25_

This is my repository for my Useless "Party" Box project. I started and finished this project before I had a personal Github account so there unfortunately isn't really a commit history. That said, there is a "Notes" folder in this repo containing resources I used/wrote which helped a lot during development, as well as some pictures.

<img src="./Notes/Photos/Nov 19 Photo 1.jpg"
     alt="Useless Box Project"
     style="width:100%; height:auto%;">

Below are all the parts I used making this project:
* STM32 Nucleo-F401RE
* MG995 servo
* SPDT toggle switch
* 6V 2000 mAh NiMH battery
* Small breadboard
* 8 Ohm speaker
* 3 RGB LEDs
* Foam blocks
* 2 Hinges
* Wood for laser cutting box and hand carving servo arms

I started this project in February, while I was enrolled in an Embedded Microcomputing Class during my last semester at Trinity University in San Antonio, TX. My goal for this project was to cover concepts we'd learn thoughout the class, like interrupt handling and timers, as well as to refamiliarize myself with STM32 development, having written on some during my summer internship at Helios Technologies in 2023. I enjoyed my time there, but Spring of this year was where my passion for embedded development really took off. 

I always thought Useless Boxes were a pretty fun novelty toy, and felt as though it would be a great opportunity to apply many concepts we covered in class. The original idea was just to make a standard useless box, but as the semester progressed, I kept adding more things to the list of things I wanted to add. Ultimately, I came up with a concept for it, which is that inside the box there is actually a party happening, made apparent by the lights and music (Trap Queen by Fetty Wap, of course). As soon as you flip the switch though, the lights and music cut out and a little arm swings out to flip it back, which it when the party resumes. I also engraved a favorite meme of mine on the inside of the lid, Pepsi Dog. 

<img src="./Notes/Photos/Pepsi Dog.jpg"
     alt="Useless Box Project"
     style="width:100%; height:auto%;">

Working on this project was genuinely a lot of fun. I ran into a lot of issues early on, for example my first attempt at controlling a servo absolutely fried my first board, a Nucleo-L476RG, because I tried powering the servo directly from the 5V pin. I also fried a couple LEDs and partially melted a toggle switch, but after the first two weeks it was relatively smooth sailing. I worked on this project between my apartment and Trinity's Makerspace, where I learned to use many of the woodworking machines and laser cutter, and eventually submitted it as my final project for an Intro to Making class I was taking. 

<img src="./Notes/Photos/Cardboard Box Prototype.JPG"
     alt="Useless Box Project"
     style="width:100%; height:auto%;">

When I finished this project, I was left wanting to work on some improvements to it, like:
* Design new patterns/routines.
* Have a "power" switch in the back, which physically cuts power to motors and turns off music and lights through software.
* Figure out how to power the MCU with the battery so that it doesn't have to be plugged in to a computer to work.

After I finished this project, I started researching what my next one would be, and not long after I began work on a Ball and Beam system, which you can check out [here](https://github.com/JorgeLarach/Ball-and-Beam "B&B Project Link").

