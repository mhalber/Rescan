#define MSH_STD_IMPLEMENTATION
#define MSH_PLY_IMPLEMENTATION
#define MSH_ARGPARSE_IMPLEMENTATION
#define MSH_VEC_MATH_IMPLEMENTATION
#define MSH_GEOMETRY_IMPLEMENTATION
#define MSH_HASH_GRID_IMPLEMENTATION
#define RS_POINTCLOUD_IMPLEMENTATION
#define RS_DISTANCE_FUNCTION_IMPLEMENTATION
#define RS_DATABASE_IMPLEMENTATION
#define FILEPATH_HELPERS_IMPLEMENTATION
#define HASHTABLE_IMPLEMENTATION
#define ICP_IMPLEMENTATION

#include <cassert>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <cstdarg>
#include <cstddef>
#include <cstdbool>
#include <cstdio>
#include <cstdlib>
#include <cfloat>
#include <cctype>

#include "msh/msh_std.h"
#include "msh/msh_argparse.h"
#include "msh/msh_vec_math.h"
#include "msh/msh_geometry.h"
#include "msh/msh_ply.h"
#include "msh/msh_hash_grid.h"
#include "mg/hashtable.h"
#include "icp.h"
#include "filepath_helpers.h"
#include "rs_pointcloud.h"
#include "rs_database.h"
#include "rs_distance_function.h"
#include "pose_proposal.h"

// Let's ask compiler nicely to generate code for these functions:
template int* msh_array__grow<int>(int* arr, unsigned long long new_len, unsigned long long elem_size );
template rs_object_placement* msh_array__grow<rs_object_placement>(rs_object_placement* arr, unsigned long long new_len, unsigned long long elem_size );
template pose_proposal* msh_array__grow<pose_proposal>(pose_proposal* arr, unsigned long long new_len, unsigned long long elem_size );
template pose_proposal** msh_array__grow<pose_proposal*>(pose_proposal** arr, unsigned long long new_len, unsigned long long elem_size );
template mark* msh_array__grow<mark>(mark* arr, unsigned long long new_len, unsigned long long elem_size );

typedef struct options
{
  char* rsdb_filename;
  char* scene_filename;
  char* output_filename;
  bool print_verbose;
} options_t;

typedef struct mgs_matching_options
{
  int n_best_results;
  float score_search_radius;
} mgs_matching_opts_t;

void save_pose_proposals( const char* filename, 
                          msh_array(msh_array(pose_proposal_t)) results )
{
  if( !filename ) return;

  FILE* fp = fopen(filename, "wb");
  if( !fp ) { printf( "Could not open file %s\n", filename); return; }

  int n_result_arrays = (int)msh_array_len(results);
  fwrite( &n_result_arrays, sizeof(int), 1, fp );
  for( int i = 0 ; i < n_result_arrays; ++i )
  {
    int n_results = (int)msh_array_len(results[i]);
    fwrite( &n_results, sizeof(int), 1, fp );
  }

  for( int i = 0; i < n_result_arrays; ++i )
  {
    int n_results = (int)msh_array_len(results[i]);
    for( int j = 0; j < n_results; ++j)
    {
      msh_mat4_t pose = results[i][j].xform;
      float score = results[i][j].score;
      fwrite( &pose.col[0].x, sizeof(float), 16, fp );
      fwrite( &score, sizeof(float), 1, fp );
    }
  }
  fclose(fp);
}

int 
parse_arguments(int argc, char** argv, options_t* opts, mgs_opts_t *mgs_opts )
{
  // Defaults
  opts->rsdb_filename      = NULL;
  opts->scene_filename     = NULL;
  opts->output_filename    = NULL;
  opts->print_verbose      = 0;
  mgs_init_opts( mgs_opts );
  
  // Command line
  msh_argparse_t parser;
  msh_ap_init( &parser, "pose_prroposal", 
               "This programs output pose proposals using mutliresolution grid search" );
  msh_ap_add_string_argument( &parser, "rsdb_filename", NULL, "Input .rsdb filename", 
                              &opts->rsdb_filename, 1 );
  msh_ap_add_string_argument( &parser, "scene_filename", NULL, "Reconstuction .ply file"
                              " to which align meshes from .rsdb file", 
                              &opts->scene_filename, 1 );
  msh_ap_add_string_argument( &parser, "output_filename", NULL, "Name of the output rsdb file", 
                              &opts->output_filename, 1 );
  msh_ap_add_bool_argument( &parser, "--verbose", "-v", "Print verbose information",
                            &opts->print_verbose, 0 );

  return msh_ap_parse( &parser, argc, argv );
}

int main( int argc, char** argv )
{
  uint64_t t1, t2;
  options_t opts;
  mgs_opts_t mgs_opts;
  if( !parse_arguments( argc, argv, &opts, &mgs_opts ) ) { return 0; }
  
  // Load in the database
  rsdb_t* rsdb = rsdb_init();
  msh_cprintf( opts.print_verbose, "IO: Loading the database %s...\n", opts.rsdb_filename );
  t1 = msh_time_now();
  int rsdb_err = rsdb_load( rsdb, opts.rsdb_filename, 1 );
  
  rsdb->model_folder         = create_model_folder_name( opts.output_filename );
  t2 = msh_time_now();
  if( !rsdb_err && opts.print_verbose )
  {
    printf( "IO: Read an rsdb file %s in %fms.\n", 
             opts.rsdb_filename, msh_time_diff_ms( t2, t1 ) );
    printf( "IO:   N. Objects:      %d\n", msh_array_len( rsdb->objects ) ); 
    printf( "IO:   N. Scenes:       %d\n", msh_array_len( rsdb->scenes ) ); 
    printf( "IO:   N. Arrangements: %d\n", msh_array_len( rsdb->arrangements ) );
  }
  else { exit(-1); }


  t1 =  msh_time_now();
  rs_scene_t scn = rsdb_scene_init( opts.scene_filename, msh_array_len( rsdb->scenes ), 1 );
  t2 = msh_time_now();
  if( scn.shape && opts.print_verbose )
  {
    printf( "IO: Read a scene %s .ply file in %fms.\n", 
            opts.scene_filename, msh_time_diff_ms( t2, t1 ) );
    printf( "IO:   N. Points:      %d\n", (int32_t)scn.shape->n_pts[0] ); 
  }
  else { printf( "Could not open %s\n", opts.scene_filename ); exit(-1); }
  scn.pose_proposal_filename = create_pose_proposal_filename( opts.output_filename );
  rsdb_add_scene( rsdb, &scn );

  // Generate pose proposals
  msh_array( msh_array( pose_proposal_t ) ) proposed_poses = NULL;
  mgs_propose_poses( rsdb, scn.shape, &proposed_poses, &mgs_opts, opts.print_verbose );
  mgs_non_maxima_suppresion( rsdb, &proposed_poses, opts.print_verbose, 0.2f );


  // Copy poses from previous arrangements
  for( int32_t arrangement_idx = 0; arrangement_idx < msh_array_len( rsdb->arrangements ); ++arrangement_idx )
  {
    msh_array(rs_obj_plcmnt_t) arrangement = rsdb->arrangements[arrangement_idx];
    for( size_t arrangement_idx = 0; arrangement_idx < msh_array_len( arrangement ); ++arrangement_idx )
    {
      rs_obj_plcmnt_t placement = arrangement[arrangement_idx];
      pose_proposal_t proposal = {.xform = placement.pose, .score = 10.0 };
      msh_array_push( proposed_poses[placement.object_idx], proposal );
    }
  }

  // Refine the coarse poses with icp
  int32_t search_lvl = 1;
  int32_t max_n_neigh = 32;
  for( size_t i = 0; i < msh_array_len(proposed_poses); ++i )
  {
    if( rsdb_is_object_static( rsdb, i ) ) { continue; }
    int32_t lvl = 1;
    rs_object_t obj = rsdb->objects[i];
    tmp_score_calc_storage_t storage = allocate_tmp_calc_storage( obj.shape->n_pts[lvl], 
                                                                  scn.shape->n_pts[lvl], max_n_neigh );
    msh_array( pose_proposal_t ) cur_proposals = proposed_poses[i];
    char* class_name = rsdb_get_class_name( rsdb, obj.class_idx );
    printf("POSE_PROPOSAL:   Refining poses for object %s.%03d\n", class_name, obj.uidx );

    // NOTE(maciej): This rebuilds seach index for each transformation...
    for( size_t j = 0; j < msh_array_len(cur_proposals); ++j )
    {

      msh_mat4_t xform = proposed_poses[i][j].xform;

      icp_align( obj.shape->positions[lvl+1], obj.shape->normals[lvl+1], obj.shape->n_pts[lvl+1],
                 scn.shape->positions[lvl+1], scn.shape->normals[lvl+1], scn.shape->n_pts[lvl+1],
                 &xform, msh_mat4_identity(), 0.10f, msh_deg2rad(60.0f), 0);

      float score = mgs_compute_object_alignment_score( obj.shape, scn.shape, search_lvl, lvl, xform, &storage );
        proposed_poses[i][j].score = score;
        proposed_poses[i][j].xform = xform;
    }
    free_tmp_calc_storage(&storage);
  }
  mgs_non_maxima_suppresion( rsdb, &proposed_poses, opts.print_verbose, 0.2f );
  mgs_sort_poses( &proposed_poses, opts.print_verbose );
  t2 = msh_time_now();
  printf( "POSE_PROPOSAL: Computed poses in %fs.\n", msh_time_diff_sec( t2, t1) ); 

  t1 = msh_time_now();
  int save_err = rsdb_save( rsdb, opts.output_filename, 1 );

  save_pose_proposals( scn.pose_proposal_filename, proposed_poses );
  t2 = msh_time_now();
  if( save_err )
  {
    printf( "IO: Could not save file %s\n", opts.output_filename );
  }
  else if( opts.print_verbose )
  {
    printf( "IO: Databse %s saved in %fms.\n", opts.output_filename, 
                                               msh_time_diff_ms( t2, t1 ) );
    printf( "IO:   N. Objects:      %d\n", msh_array_len( rsdb->objects ) ); 
    printf( "IO:   N. Scenes:       %d\n", msh_array_len( rsdb->scenes ) ); 
    printf( "IO:   N. Arrangements: %d\n", msh_array_len( rsdb->arrangements ) ); 
  }
  return 0;
}
