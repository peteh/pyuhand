# pyuhand - control your uhand
# Copyright (C) 2021  Pete <github@kthxbye.us>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import pyuhand
import glob
import time
uhand = pyuhand.UHand("/dev/ttyUSB0")

# play all motion files with a second of sleep inbetween
for motionFile in glob.glob("motions/*.xml"):
    motion = pyuhand.Motion.fromFile(motionFile)
    uhand.executeMotion(motion)
    time.sleep(1)
