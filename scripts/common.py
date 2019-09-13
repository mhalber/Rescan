from os import listdir
from os.path import join, splitext
import subprocess


def list_sequences( list_filename ):
  with open( list_filename ) as f:
    contents = f.readlines()
  sequences = [ line.rstrip() for line in contents ]
  return sequences

def list_subsequences( sequence_name ):
  files = listdir(sequence_name)
  subsequences = [splitext(f)[0] for f in files if splitext(f)[1] == '.ply']
  subsequences.sort()
  return subsequences

def append_cmd( cmds, cmd ):
  cmds.append([cmd])

def display_cmds( cmds ):
  for cmd in cmds:
    cmd = [ field.replace('\\', '/') for field in cmd ] #tmp
    line = ' '.join(cmd)
    print(line)

def run_cmds( cmds ):
  for cmd in cmds:
    subprocess.run( cmd, shell=True, check=True )

def get_binaries(binary_folder, requested_binaries):
  binaries = [ splitext(f)[0] for f in listdir(binary_folder) ]
  binary_dict = dict((el,0) for el in requested_binaries)
  success = True
  for key in binary_dict:
    if key in binaries:
      binary_dict[key] = join(binary_folder, key)
    else:
      success = False
  return success, binary_dict

def get_scripts(script_folder, requested_scripts):
  scripts = [ f for f in listdir(script_folder) ]
  script_dict = dict((el,0) for el in requested_scripts)
  success = True
  for key in script_dict:
    if key in scripts:
      script_dict[key] = join(script_folder, key)
    else:
      success = False
  return success, script_dict