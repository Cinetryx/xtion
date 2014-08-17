#!/usr/bin/env python2

import os

files = os.listdir( './Time/' )

for file in files:
    fileName = file[2:]
    os.rename( './Time/' + file, './Time/' + fileName )
