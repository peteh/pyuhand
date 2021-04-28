# pyuhand
## About pyuhand
Pyhand is a python library to control uhand2 produced by Shenzhen company HiWonder. 

I wrote it to allow simple control of the hand using python applications. 

I am in now way affilliated with the company and this is purely a hobby project of mine. 

The library provides an abstraction of the hand in which each axis can be commanded the range [0, 100]. 

## Installation
### From Git Repository
To install, clone the repository and run 
```bash
git clone https://github.com/peteh/pyuhand.git
cd pyuhand
python setup.py install
```

## Moving Fingers and Wrist
First create an instant of your uhand by giving it the correct serial connection address. 

```python
import pyuhand
uhand = pyuhand.UHand("/dev/ttyUSB0") # on Windows you must use COMx as address
```

To set a finger to a specific position, just command a value of 0 (closed) to 100 (open) to it. To command the hand to move, you need to call execute after you set the axes you want to move to your target positions. 
```python
# Axis
#  1: thumb
#  2: index finger
#  3: middle finger
#  4: ring finger
#  5: pinky
#  6: wrist

uhand.setTargetPercent(1, 100) # open thumb
uhand.setTargetPercent(2, 0) # close index finger
# all other fingers will stay in their original position

uhand.execute(200) # reach the goal position for all fingers in 200ms
```

## Complex Motions
Uhand2 comes with a Windows application that can create complex motions and save them to xml files. The pyuhand library provides you with the ability to replay these recorded motions as well. Some samples can be found in the *motions* folder. 

```python
import pyuhand
uhand = pyuhand.UHand("/dev/ttyUSB0")

# read the motion from the file into the Motion class
motionFile = "motions/01 1 Finger.xml"):
motion = pyuhand.Motion.fromFile(motionFile)

# execute the motion on the hand, the method will block 
# until the motion is finished
uhand.executeMotion(motion)
```
