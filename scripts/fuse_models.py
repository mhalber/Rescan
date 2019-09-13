import argparse
from os import listdir
from os.path import isfile, join
from run_poisson_reconstruction import run_poisson_recon
from run_poisson_reconstruction import run_surface_trimmer

def setup_parser():
  parser = argparse.ArgumentParser(description="Runs Poisson Reconstruction on pointclouds in specified folder")
  parser.add_argument('model_folder', help='Path to pointcloud folder')
  parser.add_argument('poisson_recon', help='Path to PoissonRecon binary')
  parser.add_argument('trimmer', help='Path to Trimmer binary')
  return parser

def fuse_models( poisson_recon_exe, trimmer_exe, model_folder ):
  model_paths = [join(model_folder, f) for f in listdir(model_folder) if f.endswith('.ply') and not 'temp' in f]
  tmp_ply_path = join(model_folder, "temp.ply")
  tmp_ply_path = tmp_ply_path.replace('\\', '/')
  for path in model_paths:
    path = path.replace('\\', '/')
    print('FUSE_MODELS: Working on %s' % (path) )
    trim_lvl = run_poisson_recon(poisson_recon_exe, path, tmp_ply_path)
    print( 'TRIM LVL:', trim_lvl - 1.5 )
    run_surface_trimmer(trimmer_exe, tmp_ply_path, path, trim_lvl, 1.5)

if __name__=='__main__':
  parser = setup_parser()
  args = parser.parse_args()
  fuse_models(args.poisson_recon, args.trimmer, args.model_folder)