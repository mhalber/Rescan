import argparse
from os.path import join, exists, basename, dirname
import common


def setup_parser():
  parser = argparse.ArgumentParser(description="Runs Rescan Segmentation Pipeline.")
  parser.add_argument('scene_list', help='List of scenes to run on')
  parser.add_argument('binary_folder', help='Path to Rescan Binaries')
  parser.add_argument('script_folder', help='Path to Rescan Scripts')
  return parser

def append_segment_transfer_cmd( cmds, bin, sequence_name, subsequence_name ):
  input_rsdb  = join( sequence_name, subsequence_name + '_pp.rsdb' )
  output_rsdb = join( sequence_name, subsequence_name + '.rsdb' )
  cmd = [bin, input_rsdb, '-o', output_rsdb, '-v' ]

  cmds.append(cmd)
  return output_rsdb

def append_seg2rsdb_cmd( cmds, bin, sequence_name, subsequence_name):
  input_ply = join(sequence_name, 'gt_segmentation', subsequence_name + '.ply' )
  input_rsdb = 'nyu40_classes.txt'
  output_rsdb = join(sequence_name, subsequence_name + '.rsdb')
  cmd = [bin, input_ply, input_rsdb, output_rsdb, '-v']
  cmds.append(cmd)
  return output_rsdb

def append_pose_proposal_cmd( cmds, bin, sequence_name, subsequence_name, prev_rsdb ):
  input_rsdb  = prev_rsdb 
  input_ply   = join( sequence_name, 'gt_segmentation', subsequence_name + '.ply' )
  output_rsdb = join( sequence_name, subsequence_name + '_pp.rsdb' )
  cmd = [ bin, input_rsdb, input_ply, output_rsdb, '-v' ]
  cmds.append(cmd)

def append_fuse_models_cmd( cmds, script, pr_bin, st_bin, sequence_name, subsequence_name):
  model_folder = join( sequence_name, subsequence_name )
  cmd = ['python', script, model_folder, pr_bin, st_bin ]
  cmds.append(cmd)
  return 1

def main():
  parser = setup_parser()
  args   = parser.parse_args()

  found_all, binary_paths = common.get_binaries( args.binary_folder, ['seg2rsdb', 'pose_proposal', 'segment_transfer', 'PoissonRecon', 'SurfaceTrimmer'] )
  if not found_all:
    print('Missing a binary!')
    print(binary_paths)
    return 0

  found_all, script_paths = common.get_scripts( args.script_folder, ['fuse_models.py'] )
  if not found_all:
    print('Missing a binary!')
    print(script_paths)
    return 0
  
  cmds = []
  sequences_folder = dirname( args.scene_list )
  sequences_names = common.list_sequences( args.scene_list )
  
  for sequence_name in sequences_names:
    # if sequence_name != 'encintas_one':
      # continue
    sequence_path = join(sequences_folder, sequence_name, "gt_segmentation")
    subsequence_names = common.list_subsequences( sequence_path )
    prev_rsdb = append_seg2rsdb_cmd( cmds, binary_paths['seg2rsdb'], sequence_name, subsequence_names[0] )
    for subsequence_name in subsequence_names[1:]:
      append_pose_proposal_cmd( cmds, binary_paths['pose_proposal'], sequence_name, subsequence_name, prev_rsdb )
      prev_rsdb = append_segment_transfer_cmd(cmds, binary_paths['segment_transfer'], sequence_name, subsequence_name )
      append_fuse_models_cmd( cmds, script_paths['fuse_models.py'], binary_paths['PoissonRecon'], binary_paths['SurfaceTrimmer'], sequence_name, subsequence_name )
    cmds.append([])

  common.display_cmds(cmds)
  common.run_cmds(cmds)

if __name__ == '__main__':
  main()
