#!/usr/bin/env python
# creates a relay to a python script source file, acting as that file.
# The purpose is that of a symlink
with open("/home/viki/PhantomX_Gesture/src/gestureLibrary/gestureLibrary.py", 'r') as fh:
    exec(fh.read())
