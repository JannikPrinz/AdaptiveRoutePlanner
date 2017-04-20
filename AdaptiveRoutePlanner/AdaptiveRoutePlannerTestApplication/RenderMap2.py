#!/usr/bin/env python2

from mapnik import *

mapfile = 'mapnik_style.xml'
map_output = 'mymap.png'

m = Map(4*1024,4*1024)
load_map(m, mapfile)
bbox=(Envelope( 10.0,47.5,11.1,48.1 ))

m.zoom_to_box(bbox)
print "Scale = " , m.scale()
render_to_file(m, map_output)