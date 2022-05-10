#!/usr/bin/sh
for file in ./*.dxf;do ogr2ogr -f GeoJSON ${file%.dxf} $file; done

