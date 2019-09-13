import argparse
import subprocess

def setup_parser():
  parser = argparse.ArgumentParser(description='Run Poisson Reconstruction')
  parser.add_argument('poisson_recon', help='Path to PoissonRecon binary')
  parser.add_argument('trimmer', help='Path to Trimmer binary')
  parser.add_argument('input_ply', help='Path to input .ply file')
  parser.add_argument('output_ply', help='Path to output .ply file')
  parser.add_argument('--trim_lvl', default=2)
  return parser

def run_poisson_recon( poisson_recon_exe, input_ply, output_ply ):
  # NOTE(maciej): Possibly allow for command line args, but this seems to work well.
  cmd = [poisson_recon_exe,
          '--in', input_ply,
          '--out', output_ply,
          '--width', str(0.01),
          '--scale', str(1.25),
          '--pointWeight', str(0.1),
          '--samplesPerNode', str(5),
          '--colors',
          '--data', str(1.5),
          '--density',
          '--verbose']
  print('POISSON_CMD:', ' '.join( cmd ) )
  completed_process = subprocess.run( cmd, capture_output=True, check=True )
  poisson_stdout = completed_process.stdout.decode('UTF-8').splitlines()
  # for line in poisson_stdout:
  #   print(line)
  # print("")
  for line in reversed(poisson_stdout):
    tokens = line.split(' ')
    if tokens[0] == 'Cycle[0]' and tokens[-1] != '0':
      depth_token = tokens[1]
      open_bracket_pos = depth_token.find('[')
      backslash_pos = depth_token.find('/')
      if backslash_pos < 0:
        depth_token = tokens[2]
        open_bracket_pos = depth_token.find('[')
        backslash_pos = depth_token.find('/')
      count = int(depth_token[open_bracket_pos+1:backslash_pos])
      return count

def run_surface_trimmer( trimmer_exe, input_ply, output_ply, trim_lvl, subs_lvl ):
  cmd = [trimmer_exe,
          '--in', input_ply,
          '--out', output_ply,
          '--trim', str(trim_lvl - subs_lvl),
          '--smooth', str(0)]
  print('TRIMMER_CMD:',' '.join( cmd ))
  subprocess.run( cmd, capture_output=True, check=True )

if __name__ == "__main__":
  parser = setup_parser()
  args = parser.parse_args()
  trim_lvl = run_poisson_recon(args.poisson_recon, args.input_ply, "temp.ply")
  run_surface_trimmer( args.trimmer, "temp.ply", args.output_ply, trim_lvl, args.trim_lvl )

