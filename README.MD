This repo contains hardware and software files for building and operating a behavior rig that dispenses food pellets to a mouse. 
The food pellets are positioned in front of the mouse by an apparatus containing two stepper motors controlled by an Arduino. 
The mouse sits in the "mouse house" or mouse enclosure to access the food pellets. 
There is a gap between the "mouse house" and the mechanism that presents food pellets, so the mouse has to reach for the food pellet with the forelimb and forepaw.
Pellet orientation is not controlled during loading or pellet presentation to the mouse. 

# FAQs

* **This is for scientists.**
* **This is a behavior rig that presents food pellets to a mouse.**
* **The components of the rig are controlled by an Arduino.**
* **This is for scientists interested in studying forelimb reaching behaviors in mice, designed by a neuroscientist.** 
* **This repo contains materials related to my post-doctoral work.** 
Note: I am a scientist, not an engineer. 
Certainly, there are non-optimal components of this design. 
This is not fancy. Yes, one step is to glue on a paintbrush.
Please feel free to improve upon and share this design, but ...
* **Please acknowledge this Github page in any publications relying on or adapting this design. Thank you!**
* **This is DIY and use-at-your-own-risk material.** 
You will need to adapt the code and designs to your own goals.
I cannot answer support requests.
Building this rig based on these materials requires a decent understanding of Arduino, ThorLabs parts, and generally good spatial reasoning.
* **Known Issues**
This rig works well for my needs. I run this rig for 5 hours per day without jammed pellets. 
However, there are several caveats.
First, the rig does not work well unless all components are exactly properly positioned and very well-secured.
Second, mis-loading of pellets does occur. This occurs for about 1 in 50 pellets (e.g., pellet not in the proper position when presented to the mouse or pellet missing).
Third, jammed pellets are infrequent, but this does occur. Usually, I observe jammed pellets only when a screw is loose or a rig component is improperly positioned.
The rig requires periodic maintenance.
Note that commercial food hoppers (pellet dispensers) do exist. 


# Parts List

Only the top 2 sections of this list, "Components for maneuvering pellets" and "Arduino components", contain components required to build the behavior rig.
I use the other components for other features.

[Parts List](./parts_list.html)

## Notes on 3D-printed parts

These are the 3D-printed parts:
[Pellet loader container](./parts/pellet_loader_container.stl)
[Pellet loader conical piece](./parts/pellet_loader_conical_piece.stl)
[Chute and blocker (these two mechanisms are combined into one piece)](./parts/chute_and_blocker.stl)
[Perch rings to fit around ThorLabs posts](./parts/perch_rings.stl)
[Perch middle piece, where mouse's forepaws rest, should be cut to correct size and fit into 2 perch rings](./parts/perch_connector.stl)

I've also included the .ipt versions of these .stl files, in case you want to modify the designs with CAD.

I printed these pieces using Black V4 resin on a Form 2 FormLabs printer, but other resins and printers should work.

## Notes on laser-cut parts

These are the parts cut out of acrylic sheets:
[One file including all of the parts to be cut from 1/8 inch black acrylic](./parts/behaviorBox_with_pellet_presenter.ai)
[Pellet presenter wheel alone](./parts/pellet_presenter_wheel.ai)
[Pellet loader parts to be cut from 1/16 inch clear acrylic](./parts/loader_two_plastic_parts.ai)

Note that the pellet presenter wheel has raster engraving (black filled sections). All other lines indicate clean cuts through the acrylic.

The file [with labels](./parts/behavior_box_parts_with_labels.ai) is not for laser cutting. It is instructional only and includes labels on the pieces.


# Building Reaching Rig

[How to build rig](./BUILDRIG.MD)


# Control code for Arduino

An example of the Arduino code that I use to control the behavior box is in this repo. 
Note that you may need to debug the code with your behavior box.
In particular, be aware that, if Arduino turns on a pull-up resistor, the HIGH and LOW states will be flipped for certain pin outs.


# Instructions

First, you should have a clear understanding of your experimental needs. 
Use these materials as a starting point for finding your own solutions and approach.
You will need to adapt/problem-solve depending on your goals.


# How to Help

In the spirit of open-source science, please publish your own improvements and updates to Github. Happy experimenting!