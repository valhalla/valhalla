#!/usr/bin/env python
import sys
import tempfile
from os import system

#this is all kinds of quick and dirty, but until we can submit a pr to osm to have the bbox extract
#give you presorted osm files this will have to do

def is_primitive(xml):
  return xml.startswith('<node') or xml.startswith('<way') or xml.startswith('<relation')

def predicate(xml):
  if is_primitive(xml):
    id = xml[xml.find('id="') + 4:]
    id = id[:id.find('"')]
    return xml[1:xml.find(' ')] + id.zfill(20)
  else:
    return '#' + xml

if __name__ == "__main__":

  #check for a file
  if len(sys.argv) < 2:
    raise "Provide a file to sort"

  #for each file
  for file_name in sys.argv[1:]:

    #sanitize the file
    system('sed -e "s/^\s\+//g" -e "s/\s\+$//g" ' + file_name + ' | tr -d \'\\n\' > tmp.osm')
    system('sed -e "s@><node@>\\n<node@g" -e "s@><way@>\\n<way@g" -e "s@><relation@>\\n<relation@g" tmp.osm > ' + file_name)

    #try to open it
    with open(file_name, 'r') as in_file, open('tmp.osm', 'w') as out_file:
      #read the lines into memory
      lines = []
      for line in in_file:
        lines.append(line)
      #sort them by id and type
      lines.sort(key=predicate)
      #save them to a tmp file
      for line in lines:
        out_file.write(line)

    #put the file back in its sorted form
    system('mv tmp.osm ' + file_name)
    system('sed -i -e "s@><@>\\n<@g" -e "/<\\/osm>/d" ' + file_name)
    system('echo "</osm>" >> ' + file_name)
